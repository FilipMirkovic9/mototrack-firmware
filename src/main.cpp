#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// =============================================================================
// Debug flags (override via -D<FLAG>=1 in build_flags; defaults noted below)
// =============================================================================
#ifndef GPS_DEBUG
#define GPS_DEBUG 0
#endif

// Temporary: confirms the rev2 axis remap. Default ON until confirmed on
// bench, then remove.
#ifndef IMU_DEBUG
#define IMU_DEBUG 1
#endif

// =============================================================================
// Pin definitions (rev2)
// =============================================================================
// I2C (IMU)
#define I2C_SDA_PIN    4
#define I2C_SCL_PIN    5

// Buttons (active LOW, INPUT_PULLUP)
#define BTN_UP_PIN     8
#define BTN_DOWN_PIN   3
#define BTN_SELECT_PIN 38

// Power/charger monitor - active LOW = USB present
// NOTE: rev1 had LED_POWER on pin 21; rev2 repurposes pin 21 as PGOOD_PIN.
//       LED_POWER has been removed.
#define PGOOD_PIN      21

// LEDs - active high
#define LED_STATUS    17
#define LED_CHARGE    18

// Battery ADC (VBAT/2 voltage divider on BATT_ADC_PIN)
#define BATT_ADC_PIN   1

// GPS (MAX-M10S, UART)
#define GPS_RX_PIN    15   // ESP32 RX <- GPS TXD
#define GPS_TX_PIN    16   // ESP32 TX -> GPS RXD

// IMU interrupt
#define IMU_INT1_PIN   2

// SPI bus — shared between display and flash; two CS lines: DISP_CS=12, FLASH_CS=9
#define SPI_SCLK_PIN  14
#define SPI_MOSI_PIN  13
#define SPI_MISO_PIN  47

// Flash (W25Q512)
#define FLASH_CS_PIN   9

// W25Q512 commands
#define FLASH_CMD_JEDEC_ID    0x9F
#define FLASH_CMD_WRITE_EN    0x06
#define FLASH_CMD_READ_SR1    0x05
#define FLASH_CMD_SECTOR_ERASE 0x20
#define FLASH_CMD_PAGE_PROG   0x02
#define FLASH_CMD_READ        0x03

// W25Q512 expected JEDEC ID
#define FLASH_MFR   0xEF
#define FLASH_TYPE  0x40
#define FLASH_CAP   0x20

// Test sector address (sector 0, safe for testing)
#define FLASH_TEST_ADDR 0x000000

// =============================================================================
// Display (Sharp LS027B7DH01)
// =============================================================================
#define DISP_SCLK     SPI_SCLK_PIN  // shared SPI bus (pin 14)
#define DISP_MOSI     SPI_MOSI_PIN  // shared SPI bus (pin 13)
#define DISP_CS       12
#define DISP_EXTCOMIN 11
#define DISP_DISP     10
#define DISP_WIDTH    400
#define DISP_HEIGHT   240
#define DISP_BYTES_PER_LINE  (DISP_WIDTH / 8)
#define DISP_SPI_HZ   1000000

#define SHARP_M0_WRITE  0x01
#define SHARP_M1_VCOM   0x02
#define SHARP_M2_CLEAR  0x04

static uint8_t framebuffer[DISP_HEIGHT][DISP_BYTES_PER_LINE];
static uint8_t fb_dirty[DISP_HEIGHT];
static bool    vcom_state = false;

// =============================================================================
// IMU (LSM6DSO)
// =============================================================================
#define LSM_ADDR      0x6A
#define REG_WHO_AM_I  0x0F
#define REG_CTRL1_XL  0x10
#define REG_CTRL2_G   0x11
#define REG_OUTX_L_G  0x22
#define WHO_AM_I_VAL  0x6C

#define ACCEL_SCALE   (4.0f    / 32768.0f)
#define GYRO_SCALE    (2000.0f / 32768.0f)
#undef  DEG_TO_RAD
#undef  RAD_TO_DEG
#define DEG_TO_RAD    (M_PI / 180.0f)
#define RAD_TO_DEG    (180.0f / M_PI)

// =============================================================================
// IMU axis transform - single source of truth for raw sensor -> board frame
// (fwd, left, up).
//
// rev2 mounting (IMU faces UP). Signs derived from bench data, not guessed:
// a left lean put +0.41g on raw X (lateral) with raw Y ~0 (roll axis), so
// body Y must = -raw X for left lean -> negative roll. Z is not negated.
// =============================================================================
static void imuTransformAxes(int16_t ax_raw, int16_t ay_raw, int16_t az_raw,
                              int16_t gx_raw, int16_t gy_raw, int16_t gz_raw,
                              float &ax, float &ay, float &az,
                              float &gx, float &gy, float &gz) {
    ax =  ay_raw * ACCEL_SCALE;             // body X = forward (raw Y)
    ay = -ax_raw * ACCEL_SCALE;             // body Y = left    (-raw X)
    az =  az_raw * ACCEL_SCALE;             // body Z = up      (raw Z)
    gx =  gy_raw * GYRO_SCALE * DEG_TO_RAD;
    gy = -gx_raw * GYRO_SCALE * DEG_TO_RAD;
    gz =  gz_raw * GYRO_SCALE * DEG_TO_RAD;
}

// =============================================================================
// Madgwick
// =============================================================================
#define SAMPLE_HZ   100.0f
#define DT          (1.0f / SAMPLE_HZ)
#define BETA        0.1f

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// =============================================================================
// BLE
// =============================================================================
#define BLE_DEVICE_NAME     "MotoTrack"
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define LEAN_CHAR_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_NOTIFY_INTERVAL 100

// Payload = 16 bytes, little-endian (ESP32 native):
//   [0..3] float lean   [4..7] float pitch   [8..11] float battPct
//   [12..15] uint32 powerFlags
#define PWRFLAG_USB_PRESENT  (1u << 0)   // bit0: USB present (PGOOD == LOW)
// bits 1..31 reserved, keep 0

static BLECharacteristic *pLeanCharacteristic = nullptr;
static bool bleClientConnected              = false;
static bool bleClientWasPreviouslyConnected = false;
static BLEServer *pServer                   = nullptr;

// =============================================================================
// Shared state
// =============================================================================
static volatile float g_leanAngle  = 0.0f;
static volatile float g_pitchAngle = 0.0f;
static volatile float g_ax = 0.0f;
static volatile float g_ay = 0.0f;
static volatile float g_az = 0.0f;
static volatile float g_battPct    = 0.0f;

// Gyro bias (raw units, sensor axes, pre-scale/pre-remap) from startup calibration
static float g_gyroBiasX = 0.0f;
static float g_gyroBiasY = 0.0f;
static float g_gyroBiasZ = 0.0f;

// =============================================================================
// Screens / track mode
// UI skeleton only - nav/accel/lap/session/GPS/storage fields below are
// placeholders until GPS NMEA parsing and flash ride-logging exist.
//
// Main MODE cycle is NAV -> LAP -> ACC -> TRIP -> NAV (4 screens). STATUS is
// a side door (SELECT long-press), not part of the cycle.
// =============================================================================
enum ScreenId {
    SCREEN_NAV = 0,   // default on power-up
    SCREEN_LAP,
    SCREEN_ACC,
    SCREEN_TRIP,
    SCREEN_STATUS,    // reachable only via SELECT long-press
    SCREEN_COUNT
};
#define MAIN_SCREEN_COUNT 4  // NAV, LAP, ACC, TRIP - STATUS is excluded from cycling
static ScreenId g_currentScreen = SCREEN_NAV;

enum TrackState { TRACK_IDLE, TRACK_ARMED };
static TrackState g_trackState = TRACK_IDLE;

// =============================================================================
// Buttons
// MODE (BTN_UP): short press cycles NAV/LAP/ACC/TRIP forward, long press
// arms/disarms track mode. SELECT: short press cycles the same 4 screens
// backward; long press jumps to STATUS (a short press on either button from
// STATUS returns to NAV). BTN_DOWN is not used by this UI yet.
// =============================================================================
#define BTN_DEBOUNCE_MS   50
#define BTN_LONGPRESS_MS 600
static uint32_t btnDownLastMs = 0;

struct ButtonHoldState {
    bool     held         = false;
    uint32_t pressStartMs = 0;
    bool     longFired    = false;
};
static ButtonHoldState btnModeState;
static ButtonHoldState btnSelectState;

// =============================================================================
// SPI buses
// Display uses HSPI. Flash shares HSPI with separate CS.
// =============================================================================
static SPIClass dispSPI(HSPI);

// =============================================================================
// Flash driver (W25Q512)
// Note: shares HSPI bus with display. Must manage CS carefully.
// Display CS is active HIGH (Sharp protocol).
// Flash CS is active LOW (standard SPI).
// =============================================================================
static void flashSelect()   { digitalWrite(FLASH_CS_PIN, LOW);  }
static void flashDeselect() { digitalWrite(FLASH_CS_PIN, HIGH); }

static void flashWaitReady() {
    // Poll SR1 BUSY bit until clear
    dispSPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    flashSelect();
    dispSPI.transfer(FLASH_CMD_READ_SR1);
    while (dispSPI.transfer(0x00) & 0x01) {
        // still busy
    }
    flashDeselect();
    dispSPI.endTransaction();
}

static void flashWriteEnable() {
    dispSPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    flashSelect();
    dispSPI.transfer(FLASH_CMD_WRITE_EN);
    flashDeselect();
    dispSPI.endTransaction();
}

static void flashSectorErase(uint32_t addr) {
    flashWriteEnable();
    dispSPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    flashSelect();
    dispSPI.transfer(FLASH_CMD_SECTOR_ERASE);
    dispSPI.transfer((addr >> 16) & 0xFF);
    dispSPI.transfer((addr >> 8)  & 0xFF);
    dispSPI.transfer( addr        & 0xFF);
    flashDeselect();
    dispSPI.endTransaction();
    flashWaitReady();
}

static void flashPageProgram(uint32_t addr, const uint8_t *data, uint16_t len) {
    // len must be <= 256 (one page), addr must be page-aligned
    flashWriteEnable();
    dispSPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    flashSelect();
    dispSPI.transfer(FLASH_CMD_PAGE_PROG);
    dispSPI.transfer((addr >> 16) & 0xFF);
    dispSPI.transfer((addr >> 8)  & 0xFF);
    dispSPI.transfer( addr        & 0xFF);
    for (uint16_t i = 0; i < len; i++) dispSPI.transfer(data[i]);
    flashDeselect();
    dispSPI.endTransaction();
    flashWaitReady();
}

static void flashRead(uint32_t addr, uint8_t *buf, uint16_t len) {
    dispSPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    flashSelect();
    dispSPI.transfer(FLASH_CMD_READ);
    dispSPI.transfer((addr >> 16) & 0xFF);
    dispSPI.transfer((addr >> 8)  & 0xFF);
    dispSPI.transfer( addr        & 0xFF);
    for (uint16_t i = 0; i < len; i++) buf[i] = dispSPI.transfer(0x00);
    flashDeselect();
    dispSPI.endTransaction();
}

static bool flashTest() {
    Serial.println("Flash: starting read/write/erase test...");

    // Step 1: verify JEDEC ID
    dispSPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    flashSelect();
    dispSPI.transfer(FLASH_CMD_JEDEC_ID);
    uint8_t mfr  = dispSPI.transfer(0x00);
    uint8_t type = dispSPI.transfer(0x00);
    uint8_t cap  = dispSPI.transfer(0x00);
    flashDeselect();
    dispSPI.endTransaction();

    if (mfr != FLASH_MFR || type != FLASH_TYPE || cap != FLASH_CAP) {
        Serial.printf("Flash: JEDEC ID wrong (0x%02X%02X%02X)\n", mfr, type, cap);
        return false;
    }
    Serial.println("Flash: JEDEC ID correct");

    // Step 2: erase sector 0 (4KB)
    Serial.println("Flash: erasing sector 0...");
    flashSectorErase(FLASH_TEST_ADDR);

    // Step 3: verify erased (all 0xFF)
    uint8_t eraseCheck[64];
    flashRead(FLASH_TEST_ADDR, eraseCheck, 64);
    for (int i = 0; i < 64; i++) {
        if (eraseCheck[i] != 0xFF) {
            Serial.printf("Flash: erase verify failed at byte %d (got 0x%02X)\n", i, eraseCheck[i]);
            return false;
        }
    }
    Serial.println("Flash: erase verified OK");

    // Step 4: write test pattern
    uint8_t writeData[64];
    for (int i = 0; i < 64; i++) writeData[i] = (uint8_t)(i ^ 0xA5);
    Serial.println("Flash: writing test pattern...");
    flashPageProgram(FLASH_TEST_ADDR, writeData, 64);

    // Step 5: read back and verify
    uint8_t readData[64];
    flashRead(FLASH_TEST_ADDR, readData, 64);
    for (int i = 0; i < 64; i++) {
        if (readData[i] != writeData[i]) {
            Serial.printf("Flash: verify failed at byte %d (wrote 0x%02X read 0x%02X)\n",
                          i, writeData[i], readData[i]);
            return false;
        }
    }
    Serial.println("Flash: write/read verify OK");

    // Step 6: erase again to leave clean
    flashSectorErase(FLASH_TEST_ADDR);
    Serial.println("Flash: sector erased clean after test");
    Serial.println("Flash: ALL TESTS PASSED");
    return true;
}

// =============================================================================
// Display driver
// =============================================================================
static inline void dispSelect()   { digitalWrite(DISP_CS, HIGH); delayMicroseconds(6); }
static inline void dispDeselect() { delayMicroseconds(6); digitalWrite(DISP_CS, LOW); delayMicroseconds(6); }

void displayInit() {
    pinMode(DISP_CS,       OUTPUT); digitalWrite(DISP_CS, LOW);
    pinMode(DISP_DISP,     OUTPUT); digitalWrite(DISP_DISP, LOW);
    pinMode(DISP_EXTCOMIN, OUTPUT); digitalWrite(DISP_EXTCOMIN, LOW);

    dispSPI.begin(SPI_SCLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, DISP_CS);

    delay(10);
    digitalWrite(DISP_DISP, HIGH);
    delay(10);

    dispSPI.beginTransaction(SPISettings(DISP_SPI_HZ, LSBFIRST, SPI_MODE0));
    dispSelect();
    dispSPI.transfer(SHARP_M2_CLEAR);
    dispSPI.transfer(0x00);
    dispDeselect();
    dispSPI.endTransaction();

    for (int y = 0; y < DISP_HEIGHT; y++) {
        memset(framebuffer[y], 0xFF, DISP_BYTES_PER_LINE);
        fb_dirty[y] = 1;
    }
    Serial.println("Display: initialised");
}

void displaySetPixel(int x, int y, bool white) {
    if (x < 0 || x >= DISP_WIDTH || y < 0 || y >= DISP_HEIGHT) return;
    uint8_t &b = framebuffer[y][x / 8];
    uint8_t  m = 0x01 << (x % 8);
    if (white) b |=  m;
    else       b &= ~m;
    fb_dirty[y] = 1;
}

void displayFillRect(int x, int y, int w, int h, bool white) {
    for (int row = y; row < y + h; row++)
        for (int col = x; col < x + w; col++)
            displaySetPixel(col, row, white);
}

void displayClear(bool white) {
    uint8_t fill = white ? 0xFF : 0x00;
    for (int y = 0; y < DISP_HEIGHT; y++) {
        memset(framebuffer[y], fill, DISP_BYTES_PER_LINE);
        fb_dirty[y] = 1;
    }
}

void displayFlush() {
    bool any = false;
    for (int y = 0; y < DISP_HEIGHT; y++) if (fb_dirty[y]) { any = true; break; }
    if (!any) return;

    uint8_t mode = SHARP_M0_WRITE | (vcom_state ? SHARP_M1_VCOM : 0);

    dispSPI.beginTransaction(SPISettings(DISP_SPI_HZ, LSBFIRST, SPI_MODE0));
    dispSelect();
    dispSPI.transfer(mode);
    for (int y = 0; y < DISP_HEIGHT; y++) {
        if (!fb_dirty[y]) continue;
        dispSPI.transfer((uint8_t)(y + 1));
        dispSPI.writeBytes(framebuffer[y], DISP_BYTES_PER_LINE);
        dispSPI.transfer(0x00);
        fb_dirty[y] = 0;
    }
    dispSPI.transfer(0x00);
    dispDeselect();
    dispSPI.endTransaction();
}

void displayToggleVCOM() {
    vcom_state = !vcom_state;
    digitalWrite(DISP_EXTCOMIN, vcom_state ? HIGH : LOW);
}

// =============================================================================
// 5x7 font
// =============================================================================
static const uint8_t font5x7[][5] = {
    {0x3E,0x51,0x49,0x45,0x3E}, // 0
    {0x00,0x42,0x7F,0x40,0x00}, // 1
    {0x42,0x61,0x51,0x49,0x46}, // 2
    {0x21,0x41,0x45,0x4B,0x31}, // 3
    {0x18,0x14,0x12,0x7F,0x10}, // 4
    {0x27,0x45,0x45,0x45,0x39}, // 5
    {0x3C,0x4A,0x49,0x49,0x30}, // 6
    {0x01,0x71,0x09,0x05,0x03}, // 7
    {0x36,0x49,0x49,0x49,0x36}, // 8
    {0x06,0x49,0x49,0x29,0x1E}, // 9
    {0x00,0x00,0x00,0x00,0x00}, // space
    {0x00,0x00,0x2F,0x00,0x00}, // .
    {0x08,0x08,0x08,0x08,0x08}, // -
    {0x7F,0x09,0x09,0x09,0x06}, // P
    {0x00,0x41,0x7F,0x41,0x00}, // I
    {0x7F,0x02,0x0C,0x02,0x7F}, // M
    {0x3E,0x41,0x41,0x41,0x22}, // C
    {0x7F,0x09,0x19,0x29,0x46}, // R
    {0x7C,0x12,0x11,0x12,0x7C}, // A
    {0x7F,0x49,0x49,0x49,0x41}, // E (was mislabeled "F" - this bitmap is E's, not F's)
    {0x7F,0x40,0x40,0x40,0x40}, // L
    {0x3E,0x41,0x41,0x41,0x3E}, // O
    {0x3F,0x40,0x38,0x40,0x3F}, // W
    {0x3E,0x41,0x5D,0x41,0x3E}, // U
    {0x46,0x49,0x49,0x49,0x31}, // S
    {0x7F,0x49,0x49,0x49,0x36}, // B
    {0x07,0x05,0x07,0x00,0x00}, // degree
    {0x7F,0x41,0x41,0x41,0x3E}, // D
    {0x01,0x01,0x7F,0x01,0x01}, // T
    {0x3F,0x40,0x40,0x40,0x3F}, // V
    {0x7F,0x04,0x08,0x10,0x7F}, // N (fixed: diagonal was running bottom-left to top-right - backwards, like a mirrored N/Cyrillic И. Verified geometrically: rows should increase left-to-right for a proper N diagonal, and this is now internally consistent with Z's diagonal running the opposite way, as expected for two visually-related but distinct letterforms)
    {0x3E,0x41,0x49,0x49,0x7A}, // G
    {0x7F,0x09,0x09,0x09,0x01}, // F (correct bitmap; the old "F" slot above actually held E's shape and the old copy here had a typo'd 4th byte, 0x01 instead of 0x09)
    {0x20,0x40,0x41,0x3F,0x01}, // J (not bench-verified - lower confidence, no reference file to check against)
    {0x7F,0x08,0x08,0x08,0x7F}, // H
    {0x63,0x14,0x08,0x14,0x63}, // X
    {0x46,0x49,0x49,0x49,0x31}, // S dup
    {0x7F,0x20,0x18,0x20,0x7F}, // W dup
    {0x07,0x08,0x70,0x08,0x07}, // Y
    {0x61,0x51,0x49,0x45,0x43}, // Z
    {0x7C,0x12,0x11,0x12,0x7C}, // A dup
    {0x36,0x41,0x00,0x41,0x36}, // ()
    {0x7F,0x41,0x41,0x22,0x1C}, // D dup
    {0x7F,0x01,0x01,0x01,0x01}, // L/G
    {0x7F,0x08,0x14,0x22,0x41}, // K (fixed: old bytes had bit6 set in nearly every column - a spurious bottom bar - instead of a proper second diagonal; new value is two diagonals radiating from a vertex on the stroke, spreading evenly to the top-right and bottom-right corners)
    {0x3E,0x41,0x41,0x41,0x41}, // C dup
    {0x7F,0x44,0x44,0x44,0x38}, // R dup
};

static int charToFontIndex(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    switch (c) {
        case ' ': return 10; case '.': return 11; case '-': return 12;
        case 'P': return 13; case 'I': return 14; case 'M': return 15;
        case 'C': return 16; case 'R': return 17; case 'A': return 18;
        case 'E': return 19; case 'L': return 20; case 'O': return 21;
        case 'W': return 22; case 'U': return 23; case 'S': return 24;
        case 'B': return 25; case 'D': return 27; case 'T': return 28;
        case 'V': return 29; case 'N': return 30; case 'G': return 31;
        case 'F': return 32; case 'J': return 33; case 'H': return 34;
        case 'X': return 35; case 'Y': return 38; case 'Z': return 39;
        case 'K': return 44;
        // 'Q' is still unmapped (falls through to space): no reference file
        // to verify a reconstructed bitmap against, so left out rather than
        // guessed. Add it once a known-good glyph is available.
        default:  return 10;
    }
}

void displayDrawText(int x, int y, const char *text, int scale) {
    int cx = x;
    while (*text) {
        int fi = charToFontIndex(*text++);
        for (int col = 0; col < 5; col++) {
            uint8_t colData = font5x7[fi][col];
            for (int row = 0; row < 7; row++) {
                bool white = !((colData >> row) & 1);
                for (int sy = 0; sy < scale; sy++)
                    for (int sx = 0; sx < scale; sx++)
                        displaySetPixel(cx + col*scale + sx, y + row*scale + sy, white);
            }
        }
        cx += (5 + 1) * scale;
    }
}

// =============================================================================
// Big numerals - bitmap glyphs (24x34px native) for primary readouts (speed,
// lean, lap times), replacing an earlier 7-segment renderer that looked
// blocky/"digital clock" by construction.
//
// Rasterized directly from Arial Bold (not hand-authored arc/stroke paths -
// an earlier attempt at that produced visibly malformed digits on the actual
// panel, e.g. "3" as an unrecognizable blob and "9" with a stray tail reading
// as "Q", because it was only ever checked via ASCII art/hole-scan and never
// rendered as a real image). Supersampled 4x then box-filtered down to
// 24x34 for anti-alias-correct thresholding. See gen_digits3.py in
// scratchpad. Supports 0-9 and '-' only. Rendered nearest-neighbor at an
// integer scale (1 = native 24x34px).
// =============================================================================
#define BIGNUM_W 24
#define BIGNUM_H 34
#define BIGNUM_ROW_BYTES 3

static const uint8_t BIG_DIGIT_BITMAP[11][BIGNUM_H][BIGNUM_ROW_BYTES] = {
{ {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x3C,0x00}, {0x00,0xFF,0x00}, {0x01,0xFF,0x80}, {0x03,0xFF,0xC0}, {0x07,0xE7,0xC0}, {0x07,0xC3,0xE0}, {0x07,0x83,0xE0}, {0x0F,0x81,0xE0}, {0x0F,0x81,0xE0}, {0x0F,0x81,0xF0}, {0x0F,0x81,0xF0}, {0x0F,0x81,0xF0}, {0x0F,0x81,0xF0}, {0x0F,0x81,0xF0}, {0x0F,0x81,0xF0}, {0x0F,0x81,0xF0}, {0x0F,0x81,0xF0}, {0x0F,0x81,0xE0}, {0x0F,0x83,0xE0}, {0x07,0x83,0xE0}, {0x07,0xC3,0xE0}, {0x07,0xFF,0xC0}, {0x03,0xFF,0xC0}, {0x01,0xFF,0x80}, {0x00,0xFF,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00} }, // 0
{ {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x06,0x00}, {0x00,0x0F,0x00}, {0x00,0x1F,0x00}, {0x00,0x3F,0x00}, {0x00,0x7F,0x00}, {0x00,0xFF,0x00}, {0x03,0xFF,0x00}, {0x07,0xFF,0x00}, {0x07,0xDF,0x00}, {0x07,0x9F,0x00}, {0x06,0x1F,0x00}, {0x00,0x1F,0x00}, {0x00,0x1F,0x00}, {0x00,0x1F,0x00}, {0x00,0x1F,0x00}, {0x00,0x1F,0x00}, {0x00,0x1F,0x00}, {0x00,0x1F,0x00}, {0x00,0x1F,0x00}, {0x00,0x1F,0x00}, {0x00,0x1F,0x00}, {0x00,0x1F,0x00}, {0x00,0x1F,0x00}, {0x00,0x1F,0x00}, {0x00,0x1F,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00} }, // 1
{ {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0xFF,0x00}, {0x03,0xFF,0xC0}, {0x07,0xFF,0xE0}, {0x07,0xFF,0xE0}, {0x0F,0x83,0xE0}, {0x0F,0x81,0xF0}, {0x0F,0x81,0xF0}, {0x00,0x01,0xF0}, {0x00,0x01,0xE0}, {0x00,0x03,0xE0}, {0x00,0x07,0xE0}, {0x00,0x0F,0xC0}, {0x00,0x1F,0x80}, {0x00,0x3F,0x00}, {0x00,0x7E,0x00}, {0x00,0xFC,0x00}, {0x01,0xF8,0x00}, {0x03,0xF0,0x00}, {0x03,0xE0,0x00}, {0x07,0xFF,0xE0}, {0x0F,0xFF,0xF0}, {0x0F,0xFF,0xF0}, {0x0F,0xFF,0xF0}, {0x1F,0xFF,0xF0}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00} }, // 2
{ {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x3C,0x00}, {0x01,0xFF,0x00}, {0x03,0xFF,0x80}, {0x07,0xFF,0xC0}, {0x07,0xC7,0xC0}, {0x0F,0x83,0xE0}, {0x0F,0x83,0xE0}, {0x00,0x03,0xE0}, {0x00,0x03,0xC0}, {0x00,0x0F,0xC0}, {0x00,0x3F,0x80}, {0x00,0x3F,0x00}, {0x00,0x3F,0xC0}, {0x00,0x07,0xE0}, {0x00,0x03,0xE0}, {0x00,0x01,0xF0}, {0x00,0x01,0xF0}, {0x00,0x01,0xF0}, {0x0F,0x01,0xF0}, {0x0F,0x81,0xF0}, {0x0F,0x83,0xE0}, {0x07,0xFF,0xE0}, {0x07,0xFF,0xC0}, {0x03,0xFF,0x80}, {0x00,0xFF,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00} }, // 3
{ {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x07,0xC0}, {0x00,0x0F,0xC0}, {0x00,0x0F,0xC0}, {0x00,0x1F,0xC0}, {0x00,0x3F,0xC0}, {0x00,0x3F,0xC0}, {0x00,0x7F,0xC0}, {0x00,0xFF,0xC0}, {0x00,0xF7,0xC0}, {0x01,0xE7,0xC0}, {0x03,0xE7,0xC0}, {0x03,0xC7,0xC0}, {0x07,0x87,0xC0}, {0x0F,0x87,0xC0}, {0x1F,0x07,0xC0}, {0x1F,0xFF,0xF8}, {0x1F,0xFF,0xF8}, {0x1F,0xFF,0xF8}, {0x1F,0xFF,0xF8}, {0x00,0x07,0xC0}, {0x00,0x07,0xC0}, {0x00,0x07,0xC0}, {0x00,0x07,0xC0}, {0x00,0x07,0xC0}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00} }, // 4
{ {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x01,0xFF,0xE0}, {0x01,0xFF,0xE0}, {0x01,0xFF,0xE0}, {0x03,0xFF,0xE0}, {0x03,0xFF,0xE0}, {0x03,0xFF,0xE0}, {0x03,0xC0,0x00}, {0x03,0xC0,0x00}, {0x03,0xC0,0x00}, {0x07,0xFF,0x00}, {0x07,0xFF,0xC0}, {0x07,0xFF,0xE0}, {0x07,0xFF,0xE0}, {0x07,0xC3,0xF0}, {0x00,0x01,0xF0}, {0x00,0x01,0xF0}, {0x00,0x00,0xF0}, {0x00,0x00,0xF0}, {0x0F,0x81,0xF0}, {0x0F,0x81,0xF0}, {0x07,0xC3,0xF0}, {0x07,0xFF,0xE0}, {0x03,0xFF,0xC0}, {0x01,0xFF,0x80}, {0x00,0xFF,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00} }, // 5
{ {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x1E,0x00}, {0x00,0xFF,0x80}, {0x01,0xFF,0xC0}, {0x03,0xFF,0xE0}, {0x03,0xE3,0xE0}, {0x07,0xC1,0xF0}, {0x07,0xC1,0xE0}, {0x07,0x80,0x00}, {0x0F,0x80,0x00}, {0x0F,0x9E,0x00}, {0x0F,0xBF,0x80}, {0x0F,0xFF,0xC0}, {0x0F,0xFF,0xE0}, {0x0F,0xC3,0xE0}, {0x0F,0xC1,0xF0}, {0x0F,0x81,0xF0}, {0x0F,0x80,0xF0}, {0x0F,0x80,0xF0}, {0x0F,0x80,0xF0}, {0x07,0xC1,0xF0}, {0x07,0xC1,0xF0}, {0x03,0xFF,0xE0}, {0x03,0xFF,0xE0}, {0x01,0xFF,0xC0}, {0x00,0x7F,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00} }, // 6
{ {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x0F,0xFF,0xF0}, {0x0F,0xFF,0xF0}, {0x0F,0xFF,0xF0}, {0x0F,0xFF,0xF0}, {0x0F,0xFF,0xE0}, {0x00,0x03,0xC0}, {0x00,0x07,0x80}, {0x00,0x07,0x80}, {0x00,0x0F,0x00}, {0x00,0x1F,0x00}, {0x00,0x1E,0x00}, {0x00,0x3E,0x00}, {0x00,0x3C,0x00}, {0x00,0x7C,0x00}, {0x00,0x78,0x00}, {0x00,0x78,0x00}, {0x00,0xF8,0x00}, {0x00,0xF8,0x00}, {0x00,0xF0,0x00}, {0x00,0xF0,0x00}, {0x00,0xF0,0x00}, {0x01,0xF0,0x00}, {0x01,0xF0,0x00}, {0x01,0xF0,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00} }, // 7
{ {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x7C,0x00}, {0x01,0xFF,0x00}, {0x03,0xFF,0xC0}, {0x07,0xFF,0xC0}, {0x07,0xC3,0xE0}, {0x07,0x83,0xE0}, {0x07,0x81,0xE0}, {0x07,0x81,0xE0}, {0x07,0x83,0xE0}, {0x07,0xC3,0xC0}, {0x03,0xFF,0xC0}, {0x01,0xFF,0x00}, {0x01,0xFF,0x80}, {0x07,0xFF,0xC0}, {0x07,0xC3,0xE0}, {0x0F,0x81,0xE0}, {0x0F,0x81,0xF0}, {0x0F,0x01,0xF0}, {0x0F,0x81,0xF0}, {0x0F,0x81,0xF0}, {0x0F,0x83,0xE0}, {0x07,0xE7,0xE0}, {0x07,0xFF,0xC0}, {0x03,0xFF,0xC0}, {0x00,0xFF,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00} }, // 8
{ {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x78,0x00}, {0x01,0xFF,0x00}, {0x03,0xFF,0x80}, {0x07,0xFF,0xC0}, {0x0F,0xCF,0xC0}, {0x0F,0x83,0xE0}, {0x0F,0x03,0xE0}, {0x0F,0x03,0xE0}, {0x0F,0x01,0xF0}, {0x0F,0x01,0xF0}, {0x0F,0x03,0xF0}, {0x0F,0x83,0xF0}, {0x0F,0xC7,0xF0}, {0x07,0xFF,0xF0}, {0x03,0xFF,0xF0}, {0x01,0xFD,0xF0}, {0x00,0x01,0xF0}, {0x00,0x01,0xE0}, {0x00,0x03,0xE0}, {0x0F,0x83,0xE0}, {0x0F,0x87,0xE0}, {0x07,0xFF,0xC0}, {0x07,0xFF,0x80}, {0x03,0xFF,0x00}, {0x01,0xFE,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00} }, // 9
{ {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x7F,0x80}, {0x00,0xFF,0x80}, {0x00,0xFF,0x80}, {0x00,0xFF,0x80}, {0x00,0xFF,0x80}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00}, {0x00,0x00,0x00} }, // -
};

static int bigDigitIndex(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c == '-') return 10;
    return -1;
}

// Draws one glyph at native size * scale (nearest-neighbor blit, no
// interpolation - scale 1 is native pixels, so still not "stretched").
// Returns advance width in pixels.
static int displayDrawBigGlyph(int x, int y, char c, int scale) {
    int idx = bigDigitIndex(c);
    int advance = (BIGNUM_W + 3) * scale;
    if (idx < 0) return advance; // unsupported char: blank advance, no guess

    for (int row = 0; row < BIGNUM_H; row++) {
        for (int col = 0; col < BIGNUM_W; col++) {
            uint8_t b = BIG_DIGIT_BITMAP[idx][row][col / 8];
            if (!((b >> (7 - (col % 8))) & 0x01)) continue;
            for (int sy = 0; sy < scale; sy++)
                for (int sx = 0; sx < scale; sx++)
                    displaySetPixel(x + col*scale + sx, y + row*scale + sy, false);
        }
    }
    return advance;
}

// Draws a string of digits/'-'. Returns total pixel width drawn.
static int displayDrawBigNumber(int x, int y, const char *text, int scale) {
    int cx = x;
    while (*text) cx += displayDrawBigGlyph(cx, y, *text++, scale);
    return cx - x;
}

// =============================================================================
// Circular arc gauge (monochrome: thin full ring + thicker arc for the
// filled portion, since there's no color/shading to distinguish fill vs
// track on a 1-bit panel).
// =============================================================================
static void displayDrawArc(int cx, int cy, int r, int thickness, float startDeg, float endDeg) {
    for (float deg = startDeg; deg <= endDeg; deg += 1.0f) {
        float rad = (deg - 90.0f) * DEG_TO_RAD; // 0deg = top, sweeps clockwise
        float c = cosf(rad), s = sinf(rad);
        for (int rr = r - thickness; rr <= r; rr++) {
            displaySetPixel(cx + (int)(rr * c), cy + (int)(rr * s), false);
        }
    }
}

static void displayDrawGauge(int cx, int cy, int r, float pct, const char *label) {
    displayDrawArc(cx, cy, r, 2, 0.0f, 359.0f);                    // thin track ring
    if (pct > 0.0f) displayDrawArc(cx, cy, r, 8, 0.0f, 360.0f * (pct / 100.0f)); // filled arc

    char pctStr[8];
    snprintf(pctStr, sizeof(pctStr), "%d", (int)pct);
    int scale = 3;
    int textW = (int)strlen(pctStr) * (5 + 1) * scale;
    displayDrawText(cx - textW/2, cy - (7*scale)/2, pctStr, scale);

    int labelScale = 2;
    int labelW = (int)strlen(label) * (5 + 1) * labelScale;
    displayDrawText(cx - labelW/2, cy + r + 8, label, labelScale);
}

// =============================================================================
// Battery ADC
// =============================================================================
static float battReadVoltage() {
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) sum += analogRead(BATT_ADC_PIN);
    float adcCounts  = sum / 16.0f;
    float adcVoltage = (adcCounts / 4095.0f) * 3.3f;
    return adcVoltage * 2.0f;
}

static float battVoltageToPct(float v) {
    if (v >= 4.20f) return 100.0f;
    if (v <= 3.00f) return   0.0f;
    struct { float v; float pct; } lut[] = {
        {4.20f, 100.0f}, {4.10f, 90.0f}, {4.00f, 80.0f},
        {3.90f,  70.0f}, {3.80f, 60.0f}, {3.70f, 45.0f},
        {3.60f,  30.0f}, {3.50f, 20.0f}, {3.40f, 10.0f},
        {3.20f,   5.0f}, {3.00f,  0.0f},
    };
    int n = sizeof(lut) / sizeof(lut[0]);
    for (int i = 0; i < n - 1; i++) {
        if (v <= lut[i].v && v >= lut[i+1].v) {
            float t = (v - lut[i+1].v) / (lut[i].v - lut[i+1].v);
            return lut[i+1].pct + t * (lut[i].pct - lut[i+1].pct);
        }
    }
    return 0.0f;
}

// =============================================================================
// Screens
// NOTE: layout below is NOT bench-verified against the physical MIP display -
// coordinates are computed from font/glyph pixel math only. Expect to tweak
// on first flash. Header convention: small font (scale 2, ~14px) top-left;
// primary numerals use displayDrawBigNumber (native-size, not upscaled).
// =============================================================================

static void displayDrawScreenHeader(const char *title, TrackState trackState) {
    displayDrawText(10, 8, title, 2);
    displayDrawText(330, 8, trackState == TRACK_ARMED ? "ARM" : "OFF", 2);
    displayFillRect(0, 28, 400, 2, false);
}

// Default screen: speed (left, placeholder - no GPS parsing yet), lean
// (top-right corner, real data). No route source yet, so no center nav
// readout beyond a placeholder heading dash. No bottom status bar - that
// info (pitch/USB/battery) was cluttering this screen; battery still shows
// on STATUS.
void displayDrawNavScreen(float lean, float pitch, bool usbPresent, float battPct, TrackState trackState) {
    (void)pitch; (void)usbPresent; (void)battPct; // not shown on this screen
    displayClear(true);
    displayDrawScreenHeader("NAV", trackState);

    // SPD (left) - placeholder until GPS NMEA parsing exists
    displayDrawText(20, 40, "SPD", 2);
    displayDrawBigNumber(20, 62, "--", 2);       // 24x34 bitmap @2x = 48x68px, ends y=130
    displayDrawText(20, 135, "KPH", 2);

    // LEAN (top-right corner) - real data, magnitude only; side badge is
    // fixed next to the "LEAN" label (NOT trailing the number - that
    // overflowed off the 400px-wide panel once lean hit double digits and
    // the number got wider, silently clipping the badge)
    displayDrawText(285, 40, "LEAN", 2);
    const char *side = (lean < -1.0f) ? "L" : (lean > 1.0f) ? "R" : "-";
    displayDrawText(285 + 4*(5+1)*2 + 6, 40, side, 2);

    char leanStr[8];
    snprintf(leanStr, sizeof(leanStr), "%d", (int)fabsf(lean));
    displayDrawBigNumber(285, 62, leanStr, 2);

    // Nav heading placeholder - no route source yet
    displayDrawText(175, 160, "---", 3);

    displayFlush();
}

void displayDrawLapScreen(TrackState trackState) {
    displayClear(true);
    displayDrawScreenHeader("LAP", trackState);

    if (trackState == TRACK_ARMED) {
        displayDrawText(20, 45, "TIME", 2);
        displayDrawBigNumber(20, 68, "--", 2);
        displayDrawText(20, 205, "LAST --.-   BST --.-", 2);
    } else {
        displayDrawText(20, 45, "NOT ARM", 3);
        displayDrawText(20, 90, "LONG PRESS MODE TO START", 2);
    }

    displayFlush();
}

void displayDrawAccScreen() {
    displayClear(true);
    displayDrawScreenHeader("ACC", TRACK_IDLE);

    displayDrawText(20, 40, "SPD", 2);
    displayDrawBigNumber(20, 62, "--", 2);       // ends y=130
    displayDrawText(20, 135, "KPH", 2);

    displayDrawText(20, 155, "0-100 --.-", 3);
    displayDrawText(20, 205, "NO DATA", 2);

    displayFlush();
}

void displayDrawTripScreen() {
    displayClear(true);
    displayDrawScreenHeader("TRIP", TRACK_IDLE);

    displayDrawText(20, 50,  "MAX LEAN  --", 3);
    displayDrawText(20, 90,  "AVG SPD   --", 3);
    displayDrawText(20, 130, "DIST      --", 3);
    displayDrawText(20, 170, "TIME      --", 3);

    displayFlush();
}

void displayDrawStatusScreen(float battPct) {
    displayClear(true);
    displayDrawText(10, 8, "STAT", 2);
    displayFillRect(0, 28, 400, 2, false);

    displayDrawGauge(140, 130, 70, battPct, "DEVICE");
    // Phone battery gauge deferred - no BLE write path / expo-battery on the
    // app side yet (see conversation notes). Slot reserved at (260,130).

    displayDrawText(120, 215, "GPS NO ANT", 2);
    displayDrawText(280, 215, "DISK --", 2);

    displayFlush();
}

void displayUpdateScreen(ScreenId screen, float lean, float pitch, bool usbPresent,
                          float battPct, TrackState trackState) {
    switch (screen) {
        case SCREEN_NAV:
            displayDrawNavScreen(lean, pitch, usbPresent, battPct, trackState);
            break;
        case SCREEN_LAP:
            displayDrawLapScreen(trackState);
            break;
        case SCREEN_ACC:
            displayDrawAccScreen();
            break;
        case SCREEN_TRIP:
            displayDrawTripScreen();
            break;
        case SCREEN_STATUS:
            displayDrawStatusScreen(battPct);
            break;
        default:
            break;
    }
}

// =============================================================================
// IMU
// =============================================================================
static void imuWriteReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(LSM_ADDR);
    Wire.write(reg); Wire.write(val);
    Wire.endTransmission();
}

static uint8_t imuReadReg(uint8_t reg) {
    Wire.beginTransmission(LSM_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)LSM_ADDR, (uint8_t)1);
    return Wire.read();
}

static void imuReadRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(LSM_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)LSM_ADDR, len);
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
}

// =============================================================================
// Gyro bias calibration (run once at startup, board assumed stationary)
// =============================================================================
#define GYRO_CAL_SAMPLES 200

static void calibrateGyroBias() {
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
        uint8_t buf[6];
        imuReadRegs(REG_OUTX_L_G, buf, 6);
        int16_t gx_raw = (int16_t)((buf[1] << 8) | buf[0]);
        int16_t gy_raw = (int16_t)((buf[3] << 8) | buf[2]);
        int16_t gz_raw = (int16_t)((buf[5] << 8) | buf[4]);
        sumX += gx_raw;
        sumY += gy_raw;
        sumZ += gz_raw;
        delay((uint32_t)(DT * 1000.0f));
    }
    g_gyroBiasX = (float)sumX / GYRO_CAL_SAMPLES;
    g_gyroBiasY = (float)sumY / GYRO_CAL_SAMPLES;
    g_gyroBiasZ = (float)sumZ / GYRO_CAL_SAMPLES;
    Serial.printf("GYRO CAL: biasX=%.2f biasY=%.2f biasZ=%.2f (raw)\n",
                  g_gyroBiasX, g_gyroBiasY, g_gyroBiasZ);
}

#ifdef RAW_AXIS_DEBUG
// Temporary diagnostic: prints raw accel/gyro straight from the IMU driver,
// before any remap and before Madgwick. Independent 5Hz read, separate from
// the 100Hz fusion loop.
static void printRawIMU() {
    static uint32_t lastRawDbgMs = 0;
    uint32_t now = millis();
    if (now - lastRawDbgMs < 200) return;
    lastRawDbgMs = now;

    uint8_t buf[12];
    imuReadRegs(REG_OUTX_L_G, buf, 12);
    int16_t gx_raw = (int16_t)((buf[1]  << 8) | buf[0]);
    int16_t gy_raw = (int16_t)((buf[3]  << 8) | buf[2]);
    int16_t gz_raw = (int16_t)((buf[5]  << 8) | buf[4]);
    int16_t ax_raw = (int16_t)((buf[7]  << 8) | buf[6]);
    int16_t ay_raw = (int16_t)((buf[9]  << 8) | buf[8]);
    int16_t az_raw = (int16_t)((buf[11] << 8) | buf[10]);

    float ax = ax_raw * ACCEL_SCALE;
    float ay = ay_raw * ACCEL_SCALE;
    float az = az_raw * ACCEL_SCALE;
    float gx = gx_raw * GYRO_SCALE;
    float gy = gy_raw * GYRO_SCALE;
    float gz = gz_raw * GYRO_SCALE;

    Serial.printf("RAW ax=%.3f ay=%.3f az=%.3f gx=%.2f gy=%.2f gz=%.2f\n",
                  ax, ay, az, gx, gy, gz);
}
#endif

// =============================================================================
// Madgwick
// =============================================================================
static void madgwickUpdate(float ax, float ay, float az,
                           float gx, float gy, float gz) {
    float recipNorm, s0, s1, s2, s3, qDot0, qDot1, qDot2, qDot3;
    qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx);
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = 1.0f / sqrtf(ax*ax + ay*ay + az*az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
        float _2q0=2.0f*q0, _2q1=2.0f*q1, _2q2=2.0f*q2, _2q3=2.0f*q3;
        float q0q0=q0*q0, q1q1=q1*q1, q2q2=q2*q2, q3q3=q3*q3;
        s0 = 4.0f*q0*q2q2 + _2q2*ax + 4.0f*q0*q1q1 - _2q1*ay;
        s1 = 4.0f*q1*q3q3 - _2q3*ax + 4.0f*q0q0*q1 - _2q0*ay
           - 4.0f*q1 + 8.0f*q1*q1q1 + 8.0f*q1*q2q2 + 4.0f*q1*az;
        s2 = 4.0f*q0q0*q2 + _2q0*ax + 4.0f*q2*q3q3 - _2q3*ay
           - 4.0f*q2 + 8.0f*q2*q1q1 + 8.0f*q2*q2q2 + 4.0f*q2*az;
        s3 = 4.0f*q1q1*q3 - _2q1*ax + 4.0f*q2q2*q3 - _2q2*ay;
        recipNorm = 1.0f / sqrtf(s0*s0+s1*s1+s2*s2+s3*s3);
        s0*=recipNorm; s1*=recipNorm; s2*=recipNorm; s3*=recipNorm;
        qDot0 -= BETA*s0; qDot1 -= BETA*s1; qDot2 -= BETA*s2; qDot3 -= BETA*s3;
    }
    q0+=qDot0*DT; q1+=qDot1*DT; q2+=qDot2*DT; q3+=qDot3*DT;
    recipNorm = 1.0f/sqrtf(q0*q0+q1*q1+q2*q2+q3*q3);
    q0*=recipNorm; q1*=recipNorm; q2*=recipNorm; q3*=recipNorm;
}

// =============================================================================
// BLE callbacks
// =============================================================================
class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer*) override {
        bleClientConnected = true;
        Serial.println("BLE: client connected");
    }
    void onDisconnect(BLEServer*) override {
        bleClientConnected = false;
        Serial.println("BLE: client disconnected");
    }
};

// =============================================================================
// Buttons
// =============================================================================
static bool buttonPressed(uint8_t pin, uint32_t &lastMs) {
    if (digitalRead(pin) == LOW) {
        uint32_t now = millis();
        if (now - lastMs > BTN_DEBOUNCE_MS) { lastMs = now; return true; }
    }
    return false;
}

enum ButtonEvent { BTN_EVENT_NONE, BTN_EVENT_SHORT, BTN_EVENT_LONG };

// Non-blocking press/hold/release tracker: fires SHORT on release (unless a
// LONG already fired for this press), fires LONG once as soon as the hold
// threshold is crossed while still held.
static ButtonEvent pollButtonEvent(uint8_t pin, ButtonHoldState &st) {
    bool isDown = (digitalRead(pin) == LOW);
    uint32_t now = millis();
    if (isDown && !st.held) {
        st.held         = true;
        st.pressStartMs = now;
        st.longFired    = false;
    } else if (isDown && st.held && !st.longFired) {
        if (now - st.pressStartMs >= BTN_LONGPRESS_MS) {
            st.longFired = true;
            return BTN_EVENT_LONG;
        }
    } else if (!isDown && st.held) {
        st.held = false;
        if (!st.longFired && (now - st.pressStartMs) >= BTN_DEBOUNCE_MS) {
            return BTN_EVENT_SHORT;
        }
    }
    return BTN_EVENT_NONE;
}

static void handleButtons() {
    if (buttonPressed(BTN_DOWN_PIN, btnDownLastMs)) {
        Serial.println("BTN_DOWN pressed");
    }

    ButtonEvent modeEvt = pollButtonEvent(BTN_UP_PIN, btnModeState);
    if (modeEvt == BTN_EVENT_SHORT) {
        if (g_currentScreen == SCREEN_STATUS) {
            g_currentScreen = SCREEN_NAV;
        } else {
            g_currentScreen = (ScreenId)((g_currentScreen + 1) % MAIN_SCREEN_COUNT);
        }
        Serial.printf("MODE: screen -> %d\n", (int)g_currentScreen);
    } else if (modeEvt == BTN_EVENT_LONG) {
        if (g_trackState == TRACK_IDLE) {
            g_trackState = TRACK_ARMED;
            Serial.println("MODE: track ARMED");
        } else {
            g_trackState = TRACK_IDLE;
            Serial.println("MODE: track IDLE (session save not yet implemented)");
        }
    }

    ButtonEvent selEvt = pollButtonEvent(BTN_SELECT_PIN, btnSelectState);
    if (selEvt == BTN_EVENT_SHORT) {
        if (g_currentScreen == SCREEN_STATUS) {
            g_currentScreen = SCREEN_NAV;
        } else {
            g_currentScreen = (ScreenId)((g_currentScreen + MAIN_SCREEN_COUNT - 1) % MAIN_SCREEN_COUNT);
        }
        Serial.printf("SELECT: screen -> %d\n", (int)g_currentScreen);
    } else if (selEvt == BTN_EVENT_LONG) {
        g_currentScreen = SCREEN_STATUS;
        Serial.println("SELECT: screen -> STATUS");
    }
}

// =============================================================================
// LEDs
// =============================================================================
static void updateLEDs(uint32_t now) {
    if (bleClientConnected) {
        digitalWrite(LED_STATUS, HIGH);
    } else {
        digitalWrite(LED_STATUS, (now % 1000) < 500 ? HIGH : LOW);
    }
}

// =============================================================================
// Setup
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("MotoTrack - BOOT");

    // LEDs
    pinMode(LED_CHARGE, OUTPUT); digitalWrite(LED_CHARGE, LOW);
    pinMode(LED_STATUS, OUTPUT); digitalWrite(LED_STATUS, LOW);

    // Buttons and PGOOD (all active LOW)
    pinMode(BTN_UP_PIN,     INPUT_PULLUP);
    pinMode(BTN_DOWN_PIN,   INPUT_PULLUP);
    pinMode(BTN_SELECT_PIN, INPUT_PULLUP);
    pinMode(PGOOD_PIN,      INPUT);

    // Battery ADC
    analogReadResolution(12);
    pinMode(BATT_ADC_PIN, INPUT);

    // Flash CS - deselect before SPI init
    pinMode(FLASH_CS_PIN, OUTPUT);
    flashDeselect();

    // I2C for IMU
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    // GPS UART
    Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.println("GPS: UART init 9600 8N1");

    // IMU
    uint8_t who = imuReadReg(REG_WHO_AM_I);
    if (who != WHO_AM_I_VAL) {
        Serial.printf("FATAL: IMU not found (WHO_AM_I=0x%02X)\n", who);
        while (true) { delay(1000); }
    }
    Serial.println("IMU: detected");
    imuWriteReg(REG_CTRL1_XL, 0x48);
    imuWriteReg(REG_CTRL2_G,  0x4C);

    // Gyro bias calibration - board must be stationary during boot
    calibrateGyroBias();

    // Display - inits the shared SPI bus
    displayInit();
    displayClear(true);
    displayFlush();

    // Flash test - runs once at boot on the same SPI bus
    // Display CS is LOW (idle), Flash CS is HIGH (idle) before test
    flashTest();

    // BLE
    BLEDevice::init(BLE_DEVICE_NAME);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    BLEService *pTelemetryService = pServer->createService(SERVICE_UUID);
    pLeanCharacteristic = pTelemetryService->createCharacteristic(
        LEAN_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    pLeanCharacteristic->addDescriptor(new BLE2902());
    pTelemetryService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->start();
    Serial.println("BLE: advertising as MotoTrack");
}

// =============================================================================
// Loop
// =============================================================================
void loop() {
    uint32_t now = millis();

    handleButtons();

#ifdef RAW_AXIS_DEBUG
    printRawIMU();
#endif

    // -- IMU at 100Hz --------------------------------------------------------
    static uint32_t lastImuMs = 0;
    if (now - lastImuMs >= (uint32_t)(DT * 1000.0f)) {
        lastImuMs = now;
        uint8_t buf[12];
        imuReadRegs(REG_OUTX_L_G, buf, 12);
        int16_t gx_raw = (int16_t)((buf[1]  << 8) | buf[0]);
        int16_t gy_raw = (int16_t)((buf[3]  << 8) | buf[2]);
        int16_t gz_raw = (int16_t)((buf[5]  << 8) | buf[4]);
        int16_t ax_raw = (int16_t)((buf[7]  << 8) | buf[6]);
        int16_t ay_raw = (int16_t)((buf[9]  << 8) | buf[8]);
        int16_t az_raw = (int16_t)((buf[11] << 8) | buf[10]);
        gx_raw -= (int16_t)lroundf(g_gyroBiasX);
        gy_raw -= (int16_t)lroundf(g_gyroBiasY);
        gz_raw -= (int16_t)lroundf(g_gyroBiasZ);
        float ax, ay, az, gx, gy, gz;
        imuTransformAxes(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw,
                          ax, ay, az, gx, gy, gz);
        madgwickUpdate(ax, ay, az, gx, gy, gz);
        float roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD_TO_DEG;
        float pitch = asinf (2.0f*(q0*q2 - q3*q1))                               * RAD_TO_DEG;
        float lean = roll;
        if (lean >  90.0f) lean =  180.0f - lean;
        if (lean < -90.0f) lean = -180.0f - lean;
        g_leanAngle  = lean;
        g_pitchAngle = pitch;
        g_ax = ax;
        g_ay = ay;
        g_az = az;

#if IMU_DEBUG
        static uint32_t lastImuDbgMs = 0;
        if (now - lastImuDbgMs >= 500) {
            lastImuDbgMs = now;
            Serial.printf("RAW  ax=%.3f ay=%.3f az=%.3f  gx=%.2f gy=%.2f gz=%.2f\n",
                          ax_raw * ACCEL_SCALE, ay_raw * ACCEL_SCALE, az_raw * ACCEL_SCALE,
                          gx_raw * GYRO_SCALE,  gy_raw * GYRO_SCALE,  gz_raw * GYRO_SCALE);
            Serial.printf("BODY ax=%.3f ay=%.3f az=%.3f\n", ax, ay, az);
            Serial.printf("FUSED lean=%.1f pitch=%.1f\n", lean, pitch);
        }
#endif
    }

#if GPS_DEBUG
    // Raw GPS passthrough — confirms module is talking before NMEA parsing is added
    while (Serial1.available()) Serial.write(Serial1.read());
#endif

    // -- Battery ADC at 1Hz --------------------------------------------------
    static uint32_t lastBattMs = 0;
    if (now - lastBattMs >= 1000) {
        lastBattMs = now;
        float v = battReadVoltage();
        g_battPct = battVoltageToPct(v);
        if (Serial) Serial.printf("BATT: %.2fV = %.0f%%\n", v, g_battPct);
    }

    // -- BLE notify at 10Hz --------------------------------------------------
    static uint32_t lastBleMs = 0;
    if (now - lastBleMs >= BLE_NOTIFY_INTERVAL) {
        lastBleMs = now;
        if (!bleClientConnected && bleClientWasPreviouslyConnected) {
            delay(500);
            BLEDevice::startAdvertising();
            if (Serial) Serial.println("BLE: restarting advertising");
            bleClientWasPreviouslyConnected = false;
        } else if (bleClientConnected) {
            bleClientWasPreviouslyConnected = true;
        }
        bool usbPresent = (digitalRead(PGOOD_PIN) == LOW);
        updateLEDs(now);
        if (bleClientConnected) {
            float lean  = g_leanAngle;
            float pitch = g_pitchAngle;
            float batt  = g_battPct;
            uint32_t powerFlags = 0;
            if (digitalRead(PGOOD_PIN) == LOW) powerFlags |= PWRFLAG_USB_PRESENT;
            uint8_t payload[16];
            memcpy(payload + 0,  &lean,       4);
            memcpy(payload + 4,  &pitch,      4);
            memcpy(payload + 8,  &batt,       4);
            memcpy(payload + 12, &powerFlags, 4);
            pLeanCharacteristic->setValue(payload, sizeof(payload));
            pLeanCharacteristic->notify();
        }
        static uint32_t lastDbgMs = 0;
        if (now - lastDbgMs >= 500) {
            lastDbgMs = now;
            bool usbPresent = (digitalRead(PGOOD_PIN) == LOW);
            if (Serial) Serial.printf("Lean=%.1f Pitch=%.1f Batt=%.0f%% USB=%s BLE=%s\n",
                g_leanAngle, g_pitchAngle, g_battPct,
                usbPresent ? "yes" : "no",
                bleClientConnected ? "connected" : "advertising");
        }
    }

    // -- Display at 2Hz ------------------------------------------------------
    static uint32_t lastDispMs = 0;
    if (now - lastDispMs >= 500) {
        lastDispMs = now;
        bool usbPresent = (digitalRead(PGOOD_PIN) == LOW);
        displayUpdateScreen(g_currentScreen, g_leanAngle, g_pitchAngle, usbPresent, g_battPct, g_trackState);
    }

    // -- VCOM toggle at 1Hz --------------------------------------------------
    static uint32_t lastVcomMs = 0;
    if (now - lastVcomMs >= 1000) {
        lastVcomMs = now;
        displayToggleVCOM();
    }
}
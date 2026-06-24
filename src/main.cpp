#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// =============================================================================
// Pin definitions
// =============================================================================
#define SDA_PIN       4
#define SCL_PIN       5
#define BTN_UP        8
#define BTN_DOWN      3
#define BTN_SELECT    38
#define PGOOD_PIN     14

// LEDs - active high
#define LED_POWER     21
#define LED_STATUS    17
#define LED_CHARGE    18

// Battery ADC
#define BATT_ADC_PIN  1

// Flash (W25Q512) - rev1 pins
#define FLASH_CS_PIN  9
#define FLASH_SCLK    6
#define FLASH_MOSI    7
#define FLASH_MISO    13

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
#define DISP_SCLK     6
#define DISP_MOSI     7
#define DISP_CS       10
#define DISP_EXTCOMIN 11
#define DISP_DISP     12
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

static BLECharacteristic *pLeanCharacteristic = nullptr;
static bool bleClientConnected              = false;
static bool bleClientWasPreviouslyConnected = false;
static BLEServer *pServer                   = nullptr;

// =============================================================================
// Shared state
// =============================================================================
static volatile float g_leanAngle  = 0.0f;
static volatile float g_pitchAngle = 0.0f;
static volatile float g_battPct    = 0.0f;

// =============================================================================
// Buttons
// =============================================================================
#define BTN_DEBOUNCE_MS  50
static uint32_t btnUpLastMs     = 0;
static uint32_t btnDownLastMs   = 0;
static uint32_t btnSelectLastMs = 0;

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

    dispSPI.begin(DISP_SCLK, FLASH_MISO, DISP_MOSI, DISP_CS);

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
    {0x7F,0x49,0x49,0x49,0x41}, // F
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
    {0x7F,0x10,0x08,0x04,0x7F}, // N
    {0x3E,0x41,0x49,0x49,0x7A}, // G
    {0x7F,0x09,0x09,0x01,0x01}, // F dup
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
    {0x7F,0x48,0x44,0x42,0x41}, // K
    {0x3E,0x41,0x41,0x41,0x41}, // C dup
    {0x7F,0x44,0x44,0x44,0x38}, // R dup
};

static int charToFontIndex(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    switch (c) {
        case ' ': return 10; case '.': return 11; case '-': return 12;
        case 'P': return 13; case 'I': return 14; case 'M': return 15;
        case 'C': return 16; case 'R': return 17; case 'A': return 18;
        case 'F': return 19; case 'L': return 20; case 'O': return 21;
        case 'W': return 22; case 'U': return 23; case 'S': return 24;
        case 'B': return 25; case 'D': return 27; case 'T': return 28;
        case 'V': return 29; case 'N': return 30; case 'G': return 31;
        case 'H': return 33; case 'X': return 34; case 'Y': return 37;
        case 'Z': return 38; case 'K': return 43;
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
// Telemetry screen
// =============================================================================
void displayUpdateTelemetry(float lean, float pitch, bool usbPresent, float battPct) {
    displayClear(true);
    char leanStr[16];
    snprintf(leanStr, sizeof(leanStr), "%d", (int)lean);
    displayDrawText(10, 20, leanStr, 7);
    const char *side = (lean < -1.0f) ? "L" : (lean > 1.0f) ? "R" : " ";
    displayDrawText(330, 30, side, 6);
    displayFillRect(0, 150, 400, 3, false);
    char pitchStr[16];
    snprintf(pitchStr, sizeof(pitchStr), "P %d", (int)pitch);
    displayDrawText(10, 170, pitchStr, 3);
    displayDrawText(220, 170, usbPresent ? "USB" : "BAT", 3);
    char battStr[16];
    snprintf(battStr, sizeof(battStr), "%d", (int)battPct);
    displayDrawText(310, 170, battStr, 3);
    displayDrawText(150, 215, "LEAN", 2);
    displayFlush();
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

static void handleButtons() {
    if (buttonPressed(BTN_UP, btnUpLastMs)) {
        Serial.println("BTN_UP pressed");
    }
    if (buttonPressed(BTN_DOWN, btnDownLastMs)) {
        Serial.println("BTN_DOWN pressed");
    }
    if (buttonPressed(BTN_SELECT, btnSelectLastMs)) {
        Serial.println("BTN_SELECT pressed");
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
    pinMode(LED_POWER,  OUTPUT); digitalWrite(LED_POWER,  HIGH);
    pinMode(LED_CHARGE, OUTPUT); digitalWrite(LED_CHARGE, LOW);
    pinMode(LED_STATUS, OUTPUT); digitalWrite(LED_STATUS, LOW);

    // Buttons and PGOOD
    pinMode(BTN_UP,     INPUT_PULLUP);
    pinMode(BTN_DOWN,   INPUT_PULLUP);
    pinMode(BTN_SELECT, INPUT_PULLUP);
    pinMode(PGOOD_PIN,  INPUT);

    // Battery ADC
    analogReadResolution(12);
    pinMode(BATT_ADC_PIN, INPUT);

    // Flash CS - deselect before SPI init
    pinMode(FLASH_CS_PIN, OUTPUT);
    flashDeselect();

    // I2C for IMU
    Wire.begin(SDA_PIN, SCL_PIN);

    // IMU
    uint8_t who = imuReadReg(REG_WHO_AM_I);
    if (who != WHO_AM_I_VAL) {
        Serial.printf("FATAL: IMU not found (WHO_AM_I=0x%02X)\n", who);
        while (true) { delay(1000); }
    }
    Serial.println("IMU: detected");
    imuWriteReg(REG_CTRL1_XL, 0x48);
    imuWriteReg(REG_CTRL2_G,  0x4C);

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
        float ax =  ay_raw * ACCEL_SCALE;
        float ay =  ax_raw * ACCEL_SCALE;
        float az = -az_raw * ACCEL_SCALE;
        float gx =  gy_raw * GYRO_SCALE * DEG_TO_RAD;
        float gy =  gx_raw * GYRO_SCALE * DEG_TO_RAD;
        float gz = -gz_raw * GYRO_SCALE * DEG_TO_RAD;
        madgwickUpdate(ax, ay, az, gx, gy, gz);
        float roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD_TO_DEG;
        float pitch = asinf (2.0f*(q0*q2 - q3*q1))                               * RAD_TO_DEG;
        float lean = roll;
        if (lean >  90.0f) lean =  180.0f - lean;
        if (lean < -90.0f) lean = -180.0f - lean;
        g_leanAngle  = lean;
        g_pitchAngle = pitch;
    }

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
            uint8_t payload[16];
            memcpy(payload + 0,  &lean,  4);
            memcpy(payload + 4,  &pitch, 4);
            memcpy(payload + 8,  &batt,  4);
            uint32_t powerFlags = 0;
            if (usbPresent) powerFlags |= (1u << 0);
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
        displayUpdateTelemetry(g_leanAngle, g_pitchAngle, usbPresent, g_battPct);
    }

    // -- VCOM toggle at 1Hz --------------------------------------------------
    static uint32_t lastVcomMs = 0;
    if (now - lastVcomMs >= 1000) {
        lastVcomMs = now;
        displayToggleVCOM();
    }
}
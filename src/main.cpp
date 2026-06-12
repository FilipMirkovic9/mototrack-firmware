#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ─── Pin definitions ──────────────────────────────────────────────────────────
#define SDA_PIN       4
#define SCL_PIN       5
#define BTN_UP        8
#define BTN_DOWN      3
#define BTN_SELECT    38
#define PIN_PGOOD     14   // BQ24072 PGOOD: active-low, 100k pullup R7 to 3V3. LOW = USB OK.

// ─── IMU (LSM6DSO) ───────────────────────────────────────────────────────────
#define LSM_ADDR      0x6A
#define REG_WHO_AM_I  0x0F
#define REG_CTRL1_XL  0x10
#define REG_CTRL2_G   0x11
#define REG_OUTX_L_G  0x22
#define WHO_AM_I_VAL  0x6C

#define ACCEL_SCALE   (4.0f    / 32768.0f)   // g/LSB       (±4g range)
#define GYRO_SCALE    (2000.0f / 32768.0f)   // dps/LSB     (±2000dps range)
#define DEG_TO_RAD    (M_PI / 180.0f)
#define RAD_TO_DEG    (180.0f / M_PI)

// ─── Madgwick filter ─────────────────────────────────────────────────────────
#define SAMPLE_HZ     100.0f
#define DT            (1.0f / SAMPLE_HZ)
#define BETA          0.1f                   // lower = trust gyro more, less vibration noise

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// ─── BLE - telemetry service (do not change UUIDs) ───────────────────────────
#define BLE_DEVICE_NAME       "MotoTrack"
#define SERVICE_UUID          "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define LEAN_CHAR_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_NOTIFY_HZ         10             // notify phone at 10Hz
#define BLE_NOTIFY_INTERVAL   (1000 / BLE_NOTIFY_HZ)

// ─── BLE - Nordic UART Service (NUS) ─────────────────────────────────────────
#define NUS_SERVICE_UUID      "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_CHAR_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // device->phone
#define NUS_RX_CHAR_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // phone->device

static BLECharacteristic *pLeanCharacteristic = nullptr;
static BLECharacteristic *pNusTxCharacteristic = nullptr;
static bool bleClientConnected = false;
static bool bleClientWasPreviouslyConnected = false;
static BLEServer *pServer = nullptr;

// ─── Power / charging ────────────────────────────────────────────────────────
struct PowerStatus {
    bool    usbPresent;   // PGOOD low = USB present
    bool    isCharging;   // CHG pin drives LED only (R8 470R) - no GPIO on rev1
    String  stateLabel;
};

static PowerStatus readPowerStatus() {
    PowerStatus ps;
    ps.usbPresent = (digitalRead(PIN_PGOOD) == LOW);
    // CHG is not routed to any ESP32 GPIO on rev1 - always unknown/false until rev2.
    ps.isCharging = false;
    ps.stateLabel = ps.usbPresent ? "USB Connected" : "On Battery";
    return ps;
}

// ─── Shared state (written by IMU loop, read by BLE loop) ────────────────────
// Both run on same core for now - no mutex needed until FreeRTOS migration
static volatile float g_leanAngle = 0.0f;
static volatile float g_pitchAngle = 0.0f;
static PowerStatus    g_powerStatus;

// ─── NUS helpers ─────────────────────────────────────────────────────────────

// Send a string over NUS TX; silently drops if no client or characteristic not ready.
static void bleLog(const String &msg) {
    if (!bleClientConnected || pNusTxCharacteristic == nullptr) return;
    pNusTxCharacteristic->setValue((uint8_t*)msg.c_str(), msg.length());
    pNusTxCharacteristic->notify();
}

class NusRxCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) override {
        String val = pChar->getValue().c_str();
        val.trim();
        if (val.equalsIgnoreCase("status")) {
            PowerStatus ps = readPowerStatus();
            String reply = "Lean=" + String(g_leanAngle, 2)
                         + " Pitch=" + String(g_pitchAngle, 2)
                         + " Power=" + ps.stateLabel
                         + " USB=" + (ps.usbPresent ? "yes" : "no")
                         + " CHG=unknown\n";
            bleLog(reply);
        }
    }
};

// ─── Button state (inactive until display work - wired and debounced, not used) 
#define BTN_DEBOUNCE_MS  50
static uint32_t btnUpLastMs     = 0;
static uint32_t btnDownLastMs   = 0;
static uint32_t btnSelectLastMs = 0;

// =============================================================================
// IMU helpers
// =============================================================================

static void imuWriteReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(LSM_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static uint8_t imuReadReg(uint8_t reg) {
    Wire.beginTransmission(LSM_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(LSM_ADDR, (uint8_t)1);
    return Wire.read();
}

static void imuReadRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(LSM_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(LSM_ADDR, len);
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
}

// =============================================================================
// Madgwick filter
// =============================================================================

static void madgwickUpdate(float ax, float ay, float az,
                           float gx, float gy, float gz) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot0, qDot1, qDot2, qDot3;

    // Quaternion rate of change from gyroscope (rad/s)
    qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = 1.0f / sqrtf(ax*ax + ay*ay + az*az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        float _2q0 = 2.0f*q0, _2q1 = 2.0f*q1;
        float _2q2 = 2.0f*q2, _2q3 = 2.0f*q3;
        float q0q0 = q0*q0, q1q1 = q1*q1;
        float q2q2 = q2*q2, q3q3 = q3*q3;

        s0 = 4.0f*q0*q2q2 + _2q2*ax + 4.0f*q0*q1q1 - _2q1*ay;
        s1 = 4.0f*q1*q3q3 - _2q3*ax + 4.0f*q0q0*q1 - _2q0*ay
           - 4.0f*q1 + 8.0f*q1*q1q1 + 8.0f*q1*q2q2 + 4.0f*q1*az;
        s2 = 4.0f*q0q0*q2 + _2q0*ax + 4.0f*q2*q3q3 - _2q3*ay
           - 4.0f*q2 + 8.0f*q2*q1q1 + 8.0f*q2*q2q2 + 4.0f*q2*az;
        s3 = 4.0f*q1q1*q3 - _2q1*ax + 4.0f*q2q2*q3 - _2q2*ay;

        recipNorm = 1.0f / sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= recipNorm; s1 *= recipNorm;
        s2 *= recipNorm; s3 *= recipNorm;

        qDot0 -= BETA * s0;
        qDot1 -= BETA * s1;
        qDot2 -= BETA * s2;
        qDot3 -= BETA * s3;
    }

    q0 += qDot0 * DT;
    q1 += qDot1 * DT;
    q2 += qDot2 * DT;
    q3 += qDot3 * DT;

    recipNorm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm;
    q2 *= recipNorm; q3 *= recipNorm;
}

// =============================================================================
// BLE callbacks
// =============================================================================

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pSvr) override {
        bleClientConnected = true;
        Serial.println("BLE: client connected");
    }
    void onDisconnect(BLEServer *pSvr) override {
        bleClientConnected = false;
        Serial.println("BLE: client disconnected");
    }
};

// =============================================================================
// Button helpers (debounced, stubs until display work)
// =============================================================================

// Returns true once per press (falling edge + debounce)
static bool buttonPressed(uint8_t pin, uint32_t &lastMs) {
    if (digitalRead(pin) == LOW) {
        uint32_t now = millis();
        if (now - lastMs > BTN_DEBOUNCE_MS) {
            lastMs = now;
            return true;
        }
    }
    return false;
}

static void handleButtons() {
    // Wired and debounced - actions added here when display is ready Friday
    if (buttonPressed(BTN_UP,     btnUpLastMs))     { /* TODO: menu up    */ }
    if (buttonPressed(BTN_DOWN,   btnDownLastMs))   { /* TODO: menu down  */ }
    if (buttonPressed(BTN_SELECT, btnSelectLastMs)) { /* TODO: select     */ }
}

// =============================================================================
// Setup
// =============================================================================

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("MotoTrack - BOOT");

    // Buttons
    pinMode(BTN_UP,     INPUT_PULLUP);
    pinMode(BTN_DOWN,   INPUT_PULLUP);
    pinMode(BTN_SELECT, INPUT_PULLUP);

    // Power status - external 100k pullup R7 already present, no internal pullup needed
    pinMode(PIN_PGOOD, INPUT);

    // IMU
    Wire.begin(SDA_PIN, SCL_PIN);

    uint8_t who = imuReadReg(REG_WHO_AM_I);
    Serial.printf("WHO_AM_I = 0x%02X (expected 0x%02X)\n", who, WHO_AM_I_VAL);
    if (who != WHO_AM_I_VAL) {
        Serial.println("FATAL: IMU not found - check wiring");
        while (true) delay(1000);
    }

    // CTRL1_XL: ODR=104Hz, FS=±4g  -> 0x48
    imuWriteReg(REG_CTRL1_XL, 0x48);
    // CTRL2_G:  ODR=104Hz, FS=±2000dps -> 0x4C
    imuWriteReg(REG_CTRL2_G,  0x4C);
    Serial.println("IMU: LSM6DSO initialised");

    // BLE
    BLEDevice::init(BLE_DEVICE_NAME);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    // Telemetry service (lean + pitch + power flags)
    BLEService *pTelemetryService = pServer->createService(SERVICE_UUID);
    pLeanCharacteristic = pTelemetryService->createCharacteristic(
        LEAN_CHAR_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pLeanCharacteristic->addDescriptor(new BLE2902());
    pTelemetryService->start();

    // Nordic UART Service
    BLEService *pNusService = pServer->createService(NUS_SERVICE_UUID);

    pNusTxCharacteristic = pNusService->createCharacteristic(
        NUS_TX_CHAR_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pNusTxCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic *pNusRxCharacteristic = pNusService->createCharacteristic(
        NUS_RX_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    pNusRxCharacteristic->setCallbacks(new NusRxCallbacks());

    pNusService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->addServiceUUID(NUS_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();
    Serial.println("BLE: advertising as MotoTrack (telemetry + NUS)");
}

// =============================================================================
// Loop
// =============================================================================

void loop() {
    uint32_t now = millis();

    // ── IMU + Madgwick at 100Hz ───────────────────────────────────────────────
    static uint32_t imuLastMs = 0;
    if (now - imuLastMs >= (uint32_t)(DT * 1000.0f)) {
        imuLastMs = now;

        uint8_t buf[12];
        imuReadRegs(REG_OUTX_L_G, buf, 12);

        int16_t gx_raw = (int16_t)((buf[1]  << 8) | buf[0]);
        int16_t gy_raw = (int16_t)((buf[3]  << 8) | buf[2]);
        int16_t gz_raw = (int16_t)((buf[5]  << 8) | buf[4]);
        int16_t ax_raw = (int16_t)((buf[7]  << 8) | buf[6]);
        int16_t ay_raw = (int16_t)((buf[9]  << 8) | buf[8]);
        int16_t az_raw = (int16_t)((buf[11] << 8) | buf[10]);

        // Remap axes for inverted board mount:
        // chip Y -> bike forward (X), chip X -> bike lean (Y), negate Z
        float ax =  ay_raw * ACCEL_SCALE;
        float ay =  ax_raw * ACCEL_SCALE;
        float az = -az_raw * ACCEL_SCALE;

        float gx =  gy_raw * GYRO_SCALE * DEG_TO_RAD;
        float gy =  gx_raw * GYRO_SCALE * DEG_TO_RAD;
        float gz = -gz_raw * GYRO_SCALE * DEG_TO_RAD;

        madgwickUpdate(ax, ay, az, gx, gy, gz);

        // Extract angles from quaternion
        float roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD_TO_DEG;
        float pitch = asinf (2.0f*(q0*q2 - q3*q1))                                * RAD_TO_DEG;

        // Correct for inverted mount (-176 -> ~0 at rest)
        float lean = roll;
        if (lean >  90.0f) lean =  180.0f - lean;
        if (lean < -90.0f) lean = -180.0f - lean;

        g_leanAngle  = lean;
        g_pitchAngle = pitch;
    }

    // ── BLE notify at 10Hz ───────────────────────────────────────────────────
    static uint32_t bleLastMs = 0;
    if (now - bleLastMs >= BLE_NOTIFY_INTERVAL) {
        bleLastMs = now;

        // Restart advertising after disconnect so phone can reconnect
        if (!bleClientConnected && bleClientWasPreviouslyConnected) {
            delay(500);
            pServer->startAdvertising();
            Serial.println("BLE: restarting advertising");
            bleClientWasPreviouslyConnected = false;
        }
        if (bleClientConnected) {
            bleClientWasPreviouslyConnected = true;
        }

        // Read power status (10Hz is plenty; atomic copy of volatiles before use)
        float lean  = g_leanAngle;
        float pitch = g_pitchAngle;
        g_powerStatus = readPowerStatus();

        if (bleClientConnected && pLeanCharacteristic != nullptr) {
            // 12-byte payload, all little-endian:
            //   [0-3]  lean angle  float32
            //   [4-7]  pitch angle float32
            //   [8-11] power flags uint32 — bit0=usbPresent, bit1=isCharging
            uint8_t payload[12];
            memcpy(payload + 0, &lean,  4);
            memcpy(payload + 4, &pitch, 4);
            uint32_t powerFlags = 0;
            if (g_powerStatus.usbPresent) powerFlags |= (1u << 0);
            if (g_powerStatus.isCharging) powerFlags |= (1u << 1);
            memcpy(payload + 8, &powerFlags, 4);
            pLeanCharacteristic->setValue(payload, sizeof(payload));
            pLeanCharacteristic->notify();
        }
    }

    // ── Serial + NUS debug at 2Hz ────────────────────────────────────────────
    static uint32_t serialLastMs = 0;
    if (now - serialLastMs >= 500) {
        serialLastMs = now;
        Serial.printf("Lean=%7.2f  Pitch=%7.2f  BLE=%s\n",
            g_leanAngle, g_pitchAngle,
            bleClientConnected ? "connected" : "advertising");
        Serial.printf("Power: %s | USB: %s | CHG: unknown\n",
            g_powerStatus.stateLabel.c_str(),
            g_powerStatus.usbPresent ? "yes" : "no");

        String logLine = "Lean=" + String(g_leanAngle, 2)
                       + " Pitch=" + String(g_pitchAngle, 2)
                       + " " + g_powerStatus.stateLabel + "\n";
        bleLog(logLine);
    }

    // ── Buttons ──────────────────────────────────────────────────────────────
    handleButtons();
}
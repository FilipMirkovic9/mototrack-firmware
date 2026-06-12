# MotoTrack Firmware - Hardware Context

Board: custom PCB, ESP32-S3-WROOM-1-N16R8 (16MB flash, 8MB PSRAM)
Framework: Arduino via PlatformIO (env: esp32-s3-devkitc-1)

## Build flags required (without these the board boot-loops)
-DBOARD_HAS_PSRAM
-DARDUINO_USB_MODE=1
-DARDUINO_USB_CDC_ON_BOOT=1
-mfix-esp32-psram-cache-issue

## Pin Map
### Display (Sharp LS027B7DH01 MIP - PARKED, connector reversed in rev1)
SCLK=6, MOSI=7, MISO=13, CS=10, EXTCOMIN=11, DISP=12

### Flash CS
IO9

### IMU (LSM6DSO, I2C) - validated working
SDA=IO4, SCL=IO5, address 0x6A, WHO_AM_I=0x6C

### GPS (MAX-M10S, UART) - validated working
TX=IO16 (ESP32 RX), RX=IO15 (ESP32 TX), 9600 baud
NMEA sentences confirmed. No fix indoors — expected.

### Buttons
BTN_UP=IO8, BTN_DOWN=IO3, BTN_SELECT=IO38, BOOT=IO0 (strapping pin), RESET=EN

## Known hardware issues (rev1)
- J4 display connector pin order reversed after FPC fold. Fix in rev2.
- USB-C data only works one orientation (USBLC6 D+/D- not through ESD path). Fix in rev2.

# MotoTrack

Motorcycle telemetry on a custom ESP32-S3 board: IMU lean-angle fusion, GPS,
on-board flash logging, a sunlight-readable memory-in-pixel display, and a
React Native companion app over BLE.

Hardware, firmware and app are all my own work, from schematic through
bring-up to the phone client.

![Assembled rev2 board](docs/img/board.jpg)

## Status

| Subsystem | State |
|---|---|
| IMU (LSM6DSO) fusion | Working, validated on bench |
| GPS (u-blox MAX-M10S) | UART up, NMEA sentences confirmed |
| BLE telemetry | Working, streaming to phone app |
| External SPI flash | Read / write / erase verified |
| Buttons, screen navigation | Working |
| Sharp MIP display | Wired and driven; blocked in rev1 by a connector fault, fixed in rev2 |
| Ride logging to flash | Not yet implemented |
| NMEA parsing | Not yet implemented |

## Hardware

Custom 4-layer PCB, designed in KiCad, hand-assembled including SMD.

| | |
|---|---|
| MCU | ESP32-S3-WROOM-1-N16R8 (16 MB flash, 8 MB PSRAM) |
| IMU | LSM6DSO, I2C @ 0x6A, SDA IO4 / SCL IO5 |
| GPS | u-blox MAX-M10S, UART 9600 baud |
| Display | Sharp LS027B7DH01 MIP, SPI |
| Storage | External SPI NOR flash, CS on IO9 |
| Input | Three buttons, active-low with internal pull-ups |

Schematic: [`hardware/schematic.pdf`](hardware/schematic.pdf)
Full pin map and hardware notes: [`HARDWARE.md`](HARDWARE.md)

## Firmware

Arduino framework via PlatformIO.

### Attitude estimation

Accelerometer alone cannot measure lean on a motorcycle. In a steady corner the
resultant of gravity and centripetal acceleration points roughly through the
bike's vertical axis, so an accelerometer reads near-zero lean at exactly the
moment you most want a reading. The gyro gives good short-term rate but
integrates drift.

The firmware runs Madgwick quaternion fusion at 100 Hz over both, and derives
lean from the roll component of the resulting quaternion. Gyro bias is measured
and subtracted at startup by averaging samples while the board is stationary.

IMU axes are remapped in a single function (`imuTransformAxes`) rather than
scattered through the fusion code, so a change of physical mounting orientation
is a one-place edit.

### BLE telemetry

GATT server, one notify characteristic at 10 Hz. 16-byte little-endian payload:

| Offset | Type | Field |
|---|---|---|
| 0..3 | float | lean angle, degrees |
| 4..7 | float | pitch angle, degrees |
| 8..11 | float | battery percentage |
| 12..15 | uint32 | power flags (bit 0: USB present) |

Remaining flag bits are reserved and held at zero, so the app can be extended
without breaking the existing payload layout.

### Build

```bash
pio run -t upload
pio device monitor
```

These build flags are mandatory. Without them the board boot-loops:

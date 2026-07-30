# MotoTrack

Motorcycle telemetry and turn-by-turn navigation on a custom ESP32-S3 platform,
combining IMU sensor fusion, GPS positioning, BLE communication, external flash
storage and a sunlight-readable Sharp Memory LCD with a React Native companion
application.

Hardware, firmware and app are all my own work, from schematic through
bring-up to the phone client.

<img width="960" height="1280" alt="photo_2026-07-30_10-41-48" src="https://github.com/user-attachments/assets/b6f16e93-169c-446d-be60-983e9b2aba43" />

## Status

| Subsystem | State |
|---|---|
| IMU (LSM6DSO) fusion | Working, validated on bench |
| GPS (u-blox MAX-M10S) | UART up, NMEA RMC/GGA parsed, checksum-validated |
| BLE telemetry (device → phone) | Working, 10 Hz |
| BLE navigation (phone → device) | Working: maneuver, distance, speed limit, route ribbon, road name |
| Display rendering (fonts, icons, screens) | Working: NAV, LEAN, STATUS |
| External SPI flash (W25Q512, 64 MB) | Read / write / erase verified |
| Buttons, screen navigation | Working, short/long press, non-blocking |
| LAP / ACC screens | UI skeleton only, placeholder values |
| Ride logging to flash | Not yet implemented |

## Hardware

Custom PCB, designed in KiCad, hand-assembled including SMD.

| | |
|---|---|
| MCU | ESP32-S3-WROOM-1-N16R8 (16 MB flash, 8 MB PSRAM) |
| IMU | LSM6DSO, I2C @ 0x6A, SDA IO4 / SCL IO5 |
| GPS | u-blox MAX-M10S, UART 9600 baud |
| Display | Sharp LS027B7DH01 MIP, 400×240, SPI |
| Storage | W25Q512 SPI NOR flash, 64 MB, 4-byte addressing |
| Input | Three buttons, active-low with internal pull-ups |
| Power monitoring | USB-present (PGOOD) pin, battery via ADC voltage divider |

Display and flash share the same SPI bus with separate chip-select lines.

Schematic: [`hardware/schematic.pdf`](hardware/schematic.pdf)
Full pin map and hardware notes: [`HARDWARE.md`](HARDWARE.md)

## Firmware

Arduino framework via PlatformIO.

### Attitude estimation

Accelerometer alone cannot measure lean on a motorcycle. In a steady corner
the resultant of gravity and centripetal acceleration points roughly through
the bike's vertical axis, so an accelerometer reads near-zero lean at exactly
the moment you most want a reading. The gyro gives good short-term rate but
integrates drift.

The firmware runs Madgwick quaternion fusion over both, and derives lean from
the roll component of the resulting quaternion. Gyro bias is measured and
subtracted at startup by averaging 200 samples while the board is stationary.
Axis remap from raw sensor frame to board frame (forward/left/up) lives in one
function, `imuTransformAxes`, so a change of physical mounting orientation is
a one-place edit rather than a search through the fusion code. Signs in that
function were derived from bench data (a left lean measured on the bench put
a known sign on a specific raw axis), not assumed from the datasheet diagram.

**Fusion runs on measured elapsed time, not an assumed fixed step.** The loop
originally assumed a fixed 10 ms interval between IMU updates. Once the
display rendering got heavier, that stopped holding: any loop iteration that
ran long left the filter integrating with time it didn't actually have, or
discarding elapsed time as a "resync" once the stall crossed 50 ms and
correcting zero times against it. Net effect: after any hitch, the reported
lean angle took visibly longer than it should have to settle back near level.
Fix was to measure actual elapsed time each pass with `micros()` and integrate
against that instead of a constant, with a clamp so a rare long stall can't
destabilise the filter outright. This removed the dependency on loop cadence
entirely.

### BLE

GATT server with one notify characteristic (device → phone) and three write
characteristics (phone → device):

**Telemetry (notify, 10 Hz)** — 16-byte little-endian payload: lean angle,
pitch angle, battery percentage, and a power-flags word (bit 0: USB present).
Flag bits beyond bit 0 are reserved and held at zero so the payload can be
extended without breaking existing consumers.

**Navigation (write, phone → device)** — three characteristics:
- `NAV_CHAR`: fixed 12 bytes — protocol version, maneuver type, speed limit,
  distance to maneuver, phone-side GPS speed, status flags (rerouting /
  arrived).
- `ROUTE_CHAR`: variable length, up to 55 points — the phone projects the
  upcoming route into a heading-up 0–255 square and sends the point list; the
  device only scales and draws, all map math and routing stay on the phone.
- `ROAD_CHAR`: ASCII name of the next road, up to 20 bytes.

Nav packets older than 5 seconds are treated as stale, so the device falls
back to an idle display if the phone app dies or drops out of range rather
than showing a frozen instruction.

Advertising restart on disconnect is non-blocking — an earlier version called
`delay(500)` in the disconnect handler, which stalled the 100 Hz IMU loop for
half a second on every reconnect. It's now scheduled against `millis()` and
handled in the main loop instead.

### Display

Custom rendering pipeline over the Sharp memory-in-pixel panel: a 400×240
1-bit framebuffer, a proportional bitmap font renderer (fonts generated from
Liberation Sans Bold by a Python script), fixed-size maneuver icons, and
primitive drawing (rings, discs, thick lines, arc gauges) built on top of a
byte-aligned fast-fill path for large rectangles, since the per-pixel path
alone was measurably too slow for full-width bars.

The panel holds its image with no refresh needed, so redraw is triggered only
when the values a screen actually displays have changed — a snapshot of those
values is diffed each pass, capped at 5 Hz, rather than redrawing every loop
regardless of content.

Screens: NAV (route ribbon or maneuver icon, distance, speed, speed-limit
roundel), LEAN (large numeral readout with session max-lean tracking), STATUS
(battery gauge, USB/BLE/GPS/satellite state). LAP and ACC exist as UI
skeletons with placeholder values pending lap timing and 0–100 timing logic.

### GPS

NMEA parsing for RMC (fix validity, position, speed, course) and GGA
(satellite count), including checksum validation before any sentence is
trusted — a sentence with a bad checksum is dropped rather than parsed.

### Build

```bash
pio run -t upload
pio device monitor
```

These build flags are mandatory. Without them the board boot-loops:

```
-DBOARD_HAS_PSRAM
-mfix-esp32-psram-cache-issue
-DARDUINO_USB_MODE=1
-DARDUINO_USB_CDC_ON_BOOT=1
```

Optional diagnostics: `-DRAW_AXIS_DEBUG=1`, `-DIMU_DEBUG=1`, `-DGPS_DEBUG=1`,
`-DSERIAL_TELEMETRY=1`. `FLASH_SELFTEST` runs a boot-time read/write/erase
check on the flash chip and can be disabled for release builds.

## Bring-up and debugging

**IMU fusion lag after display changes.** Covered above under attitude
estimation — traced to a fixed-timestep assumption breaking once the render
loop got heavier, fixed by measuring actual elapsed time per update.

**BLE reconnect stalling the IMU loop.** A blocking half-second delay in the
disconnect handler was silently costing five IMU update cycles on every
reconnect. Moved to a non-blocking scheduled restart.

**Display connector reversed (rev1).** The J4 FPC connector ended up with
reversed pin order once the flex was folded into its mounting position. The
footprint was correct in isolation and only wrong in context. Corrected in
rev2.

**USB-C data worked in only one orientation (rev1).** The USBLC6 ESD
protection was not in the D+/D- path for both CC orientations, so the port
enumerated one way up and not the other. Fixed in rev2.

**IMU axis orientation.** Rather than guess the remap after changing the
board mounting to face upwards, an independent 5 Hz diagnostic read
(`RAW_AXIS_DEBUG`) prints raw accelerometer and gyro values straight from the
driver, before remap and before fusion. That separates three failure layers
that look identical from the outside — a wiring/I2C fault, a wrong remap, and
a fusion problem. The remap was confirmed against known physical orientations
before the diagnostic path was disabled.

**Boot-looping on first bring-up** traced to missing PSRAM build flags rather
than a hardware fault.

Full pin map and hardware notes: [`HARDWARE.md`](HARDWARE.md).

## Companion app

`MotoTrackApp/` — React Native via Expo SDK 56, TypeScript.

- BLE transport with `react-native-ble-plx`, wrapped in a `useBLE` hook
- Global connection and telemetry state in `zustand`
- File-based routing with `expo-router`: live, map, rides, settings
- `expo-sqlite` for local ride storage
- Owns routing and map matching; sends the device only a pre-projected route
  ribbon and maneuver data, so the device does no map math

```bash
cd MotoTrackApp
npm install
npx expo start --dev-client
```

## Validation

The project has been validated through bench testing, functional verification,
and on-road testing rather than formal laboratory measurement.

### Bench testing

- IMU outputs verified against known static board orientations before and after
  axis remapping.
- Gyroscope startup bias calibration confirmed by observing stable zero-rate
  output after calibration.
- GPS parser verified using live NMEA RMC and GGA traffic with checksum
  validation and rejection of corrupted sentences.
- External SPI flash verified using read/write/erase self-tests.
- Display rendering verified on hardware for all implemented screens,
  including proportional fonts, navigation graphics and partial redraw logic.
- BLE communication verified between the ESP32-S3 firmware and the React
  Native companion application.

### On-road verification

The system has been tested on a motorcycle to verify:

- Stable lean-angle behaviour during normal riding and cornering.
- Reliable GPS lock and satellite reporting.
- Turn-by-turn navigation updates from the companion application.
- BLE connection stability while riding.
- Daylight readability of the Sharp Memory LCD.

### Software verification

Several Python utilities were used during development to inspect logged
telemetry, decode BLE packets, generate fonts and icons, and verify packet
formats during debugging. These tools were used to confirm packet integrity,
correct scaling of transmitted values and expected update rates.

Formal quantitative validation against a laboratory-grade reference IMU has
not yet been performed. Future work includes collecting synchronized reference
measurements to calculate metrics such as RMS error, bias and repeatability.

## Repository layout

```
src/            ESP32-S3 firmware
include/        headers (fonts.h, icons.h generated by gen_fonts.py / gen_icons.py)
hardware/       KiCad project, schematic PDF
MotoTrackApp/   React Native companion app
HARDWARE.md     pin map, hardware notes, known issues
```

## Known limitations

- Ride logging to flash and lap/acceleration timing are not implemented; the
  LAP and ACC screens are UI skeletons with placeholder fields.
- No automated unit or integration tests. Verification has primarily been
  performed through bench testing, on-road testing and development
  diagnostics.
- Screen layout coordinates were computed from font metrics, not yet
  bench-verified pixel-for-pixel on the physical panel.

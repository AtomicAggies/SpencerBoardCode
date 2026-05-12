# Spencer Board Code

Spencer is the telemetry producer board. It samples sensors, packs a fixed 98-byte telemetry struct, and emits framed I2C chunks for SD logging and timed radio forwarding.

## What This Firmware Does

- Samples GPS, BMP, magnetometer, and IMU data in `loop()`.
- Tracks per-sensor validity bits in a packed telemetry packet.
- Sends framed I2C data continuously to SD receiver path.
- Sends framed I2C data to radio receiver path in a PPS-aligned window.

## Data Path Overview

```text
Sensors + GPS callbacks
        |
        v
Build 98-byte TelemetryData (validity + payload + I2C status bytes)
        |
        v
Chunk into I2C frames (max 32 bytes each, 2-byte header + up to 30 payload)
        |
        +--> destination flag SD (bit0) every loop
        |
        +--> destination flag RADIO (bit1) after PPS gate
```

## Telemetry Packet Layout (98 bytes)

| Field group | Bytes | Notes |
|---|---:|---|
| `packetCounter` | 2 | Increments each send attempt |
| `validity` | 1 | bit0 GPS, bit1 BMP, bit2 mag, bit3 inertial |
| GPS block | 28 | lat/lon/alt/NED velocities/unix epoch |
| BMP block | 8 | float temperature and pressure |
| Magnetometer block | 6 | int16 x/y/z |
| Inertial block | 28 | accel xyz, gyro zyx, temperature |
| `lastI2CBytesWritten` | 1 | Saturated to 255 |
| `lastI2CStatus` | 1 | `Wire.endTransmission()` status or short write |
| Reserved | 23 | Pad to 98 bytes |

## I2C Framing

Spencer sends telemetry as multiple frames, each up to 32 bytes total.

| Item | Value |
|---|---|
| Max I2C frame size | 32 bytes |
| Header size | 2 bytes |
| Max payload per frame | 30 bytes |
| Start-of-packet flag | bit7 in header byte 0 |
| Destination SD flag | bit0 in header byte 0 |
| Destination RADIO flag | bit1 in header byte 0 |
| Header byte 1 | 8-bit checksum over payload bytes |

Frame header byte 0 bit usage:

```text
bit7  bit1  bit0
 START RADIO SD
```

## Event and Timing Behavior

1. `loop()` reads local sensors and services GPS.
2. `sendTelemetryPacket(I2C_FRAME_DESTINATION_SD)` runs every loop pass.
3. PPS interrupt sets `can_blink` and `time_of_last_pps`.
4. After about `550 + pps_delay` ms from PPS, one RADIO-destination packet is sent and gate is closed until next PPS.

## GPS Handling Notes

- GPS callback (`setAutoPVTcallbackPtr`) updates telemetry when auto-PVT data is available.
- `serviceGPS()` also performs non-blocking fallback polling (`getPVT(0)`).
- GPS validity clears when update age exceeds timeout window.

## Debug guide

### Status LEDs (Teensy)

| Signal | Pin | Meaning |
|--------|-----|---------|
| **I2C / activity LED** | **3** | Driven **HIGH** for the duration of each `sendTelemetryPacket()` burst (framed I2C writes to Abraham and/or Jacob), **LOW** when idle between bursts. |
| **Built-in LED** | **13** (`LED_BUILTIN`) | Short flash on each GPS PVT update in `saveGPSData()`. During boot, pin 3 or pin 13 may toggle while waiting for BMP, magnetometer, IMU, or GNSS to initialize. |
| **PPS** | **15** (input, pull-up) | Not an LED: GPS pulse-per-second. On each rising edge, `handleInterrupt()` pulls **pin 3 LOW**. |

### UART / serial

This sketch does **not** open `Serial` or `Serial1`; there is no live UART log unless you add it.

- **USB**: Use the Teensy USB connection and the Arduino Serial Monitor if you add `Serial.begin(...)` for temporary prints.
- **External USB–UART on Serial1 (Teensy 4.x)**: Use a **3.3 V** logic adapter only. Wiring is **GND**, **RX from the adapter to Teensy TX1 (pin 1)**, and **TX from the adapter to Teensy RX1 (pin 0)**. Typical baud is **115200** if you match Jacob’s console.

Never connect 5 V TTL UART lines directly to Teensy I/O.

## Tooling

- `telemetry_packet_viewer.py` decodes Spencer 98-byte packet logs and supports raw or terminated record formats.
- The viewer maps `lastI2CStatus` values to readable I2C status text for quick diagnostics.

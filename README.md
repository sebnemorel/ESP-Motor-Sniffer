# Motor CAN Sniffer

A simple ESP32-based CAN bus sniffer for motor driver (E32) traffic using
the ESP32's built-in TWAI controller with an SN65HVD230 transceiver.

## Overview

This project reads CAN frames from the motor driver network and prints
packet details over serial. It supports both standard and extended CAN IDs,
scans common baud rates, and highlights which data bytes change over time —
the first step for mapping bytes to physical values (RPM, temperature,
current, voltage, error flags).

## Features

- ESP32 built-in TWAI (no MCP2515 / no SPI module needed)
- LISTEN-ONLY passive mode: never transmits a single bit on the bus
- Auto-detects CAN baud rate: 500 / 250 / 125 / 1000 kbps
- Supports standard and extended CAN IDs
- Highlights changed bytes per frame: `[3A]` = changed since last frame
- Periodic summary: per-ID frame count, average period, change mask
- CSV output mode for offline analysis (pandas-friendly)
- Scenario markers for labeled test recordings

## Hardware

- ESP32 development board (dedicated sniffer board, NOT the vehicle VCU)
- SN65HVD230 (or similar 3.3V) CAN transceiver — the ESP32's internal CAN
  controller pins cannot drive the bus directly; MCP2515 modules with 5V
  transceivers are NOT needed here
- CAN bus from the motor driver

### SN65HVD230 wiring

| ESP32 pin | SN65HVD230 pin | Notes |
|-----------|----------------|-------|
| GPIO5     | D (CTX)        | CAN TX — straight, not crossed |
| GPIO4     | R (CRX)        | CAN RX — straight, not crossed |
| 3V3       | VCC            | 3.3V ONLY (not 5V) |
| GND       | GND            | Common ground with motor driver LV side |
| —         | CANH / CANL    | To motor driver CAN bus |

Bus requirements (from team notes):

- 120 ohm termination at BOTH ends of the bus (CAN-H to CAN-L). Measure
  ~60 ohm across CAN-H/CAN-L with power off; if you read ~120 ohm, add the
  second terminator, otherwise reflections corrupt the data.
- Twisted-pair wiring, as short as practical — motor drivers emit heavy EMI.

## Software

### Build and upload

1. Open the project in PlatformIO.
2. Select the `esp32dev` environment.
3. Build and upload to the ESP32.

No external CAN library is required — the TWAI driver ships with the ESP32
Arduino core.

### Serial monitor

Open the serial monitor at `115200` baud.

Serial commands (single key):

| Key | Action |
|-----|--------|
| `c` | Toggle CSV mode (for logged captures) |
| `m` | Print a scenario marker (e.g. right before opening throttle) |
| `s` | Print the summary table immediately |

The output includes connection status, detected frames (type STD/EXT, CAN
ID, DLC, data bytes with changed bytes bracketed), and a 5-second summary
listing every ID with its frame count, average broadcast period and which
byte positions have ever changed (`[..XX..X.]`).

## Test scenarios (data collection plan)

Record each scenario to its own log file
(`pio device monitor | tee S1_kontak.txt`), press `m` at key moments:

1. `S1_kontak`  — ignition on, motor idle, 2 min (baseline / static bytes)
2. `S2_gaz`     — wheel off ground, throttle slowly 0 -> max -> 0
3. `S3_sabit`   — 3-4 fixed RPM levels, 30 s each (scale factor)
4. `S4_isinma`  — 10+ min under load (temperature byte drifts slowly)
5. `S5_ariza`   — controlled fault if possible (error flags)

The recordings feed the byte-mapping analysis; the resulting signal map is
then implemented in the vehicle repo's `CanParse` module (separate PR).

## Notes

- LISTEN-ONLY means the sniffer never ACKs frames. At least one other
  ACK-capable node must be on the bus (e.g. the driver's own display). If
  the bench bus is only driver + sniffer, the driver may stop transmitting;
  in that case, after the baud rate is confirmed, switch the mode in
  `initTWAI` to normal.
- If no frames appear on any baud rate: check ignition, CAN-H/CAN-L
  polarity, termination, and transceiver power.

## License

This project is licensed under the MIT License.

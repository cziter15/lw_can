# Lightweight `ESP32` CAN Library

Library to support the ESP32 built-in CAN controller.

> This project is at an experimental stage!

## Introduction

When I developed the code for the heating boiler monitoring module (`PelletMon`) I evaluated several existing CAN libraries.

Most of them are linked below, but each has trade-offs that motivated creating a lightweight alternative:

- The most extensive is `esp32_can`, but it runs a few redundant background tasks. I took a lot of inspiration from it — for example, the idea for an automatic watchdog.
- `ESP32-Arduino-CAN` is very simple and influenced the overall code architecture.
- `ESP32-CAN-Driver` is the basis for many implementations on the web; it contains the commands required for the `SJA1000` module.
- I also used the original `TWAI` driver from `ESP-IDF`, but found some instability (for example, a fatal error in `twai_initiate_recovery()`).

These issues led me to create `lw_can` — a lightweight CAN bus library for `ESP32`.

### Features

- `TWAI`-like interface familiar from `ESP-IDF` (functions such as `lw_can_install`, `lw_can_transmit`, etc.)
- Error counters (arbitration lost, bus error, overrun, error passive, etc.)
- Watchdog that resets the CAN peripheral on error — a "smart reset" which avoids reset loops (note: this is not strictly CAN-standard behavior)
- Support for various bus speeds (configurable `BRP` settings)

## Based on

- `esp32_can` (Collin Kidder) — https://github.com/collin80/esp32_can  
- `ESP32-Arduino-CAN` (Michael Wagner) — https://github.com/miwagner/ESP32-Arduino-CAN  
- `ESP32-CAN-Driver` (Thomas Barth) — https://github.com/ThomasBarth/ESP32-CAN-Driver

## Quick notes

- This project aims to be small and stable. If you need the full feature set of `esp32_can` or the Arduino wrapper, those projects may be better fits.
- If you encounter issues with `TWAI` recovery in `ESP-IDF`, `lw_can` attempts to mitigate those problems via the watchdog logic, but please report any unexpected behavior.

## Contributing

Contributions, bug reports, and improvements are welcome. Please open issues or pull requests describing the change and the reasoning.

## License

See the repository license file for details.

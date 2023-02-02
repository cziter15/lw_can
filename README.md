# Lightweight ESP32 CAN Library

## Brief
When I developed the code for the heating boiler monitoring module (PelletMon) I used several different CAN libraries.

Most of them are linked below, but each of them has disadvantages. 

- The most extensive is esp32_can, but it has a few redundant tasks running in the background. Nevertheless, I took a lot of inspiration from it - for example, the idea for an automatic watchdog.
- As for ESP32-Arduino-CAN, this library is very simple. It inspired me in the context of the code architecture itself.
- The last linked library is the basis of most implementations available on the web. In short, it contains all the commands we need to give the SJA1000 module.
- ... of course, I also used the original TWAI driver from ESP-IDF, but here the most disturbing was its instability - for example, a fatal error in twai_initiate_recovery().

All these issues led me to create my own solution - lw_can - which is lightweight CAN bus library for ESP32.

### Features
- TWAI-like interface known from ESP-IDF (methods like lw_can_install, lw_can_transmit etc)
- Error counters (arbitration lost, bus error, overrun, passive etc)
- Watchdog (will reset peripheral in case of error)
- Various bus speed support.

## Based on
- esp32_can [from Collin Kidder](https://github.com/collin80/esp32_can)
- ESP32-Arduino-CAN [from Michael Wagner](https://github.com/miwagner/ESP32-Arduino-CAN)
- ESP32-CAN-Driver [from Thomas Barth](https://github.com/ThomasBarth/ESP32-CAN-Driver)

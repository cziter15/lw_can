# ✨ lw_can - Lightweight ESP32 CAN Library

> ⚠️ **Experimental!**  
> This project is in an early stage. Use with caution and report issues.

<div align="center">

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg?style=for-the-badge)](https://github.com/cziter15/lw_can/blob/main/LICENSE)

</div>

## 📝 Introduction

When I developed the code for the heating boiler monitoring module (`PelletMon`), I evaluated several existing CAN libraries.

Most of them are linked below, but each has trade-offs that motivated me to create a lightweight alternative:

- The most extensive is [`esp32_can`](https://github.com/collin80/esp32_can), but it runs a few redundant background tasks. I took a lot of inspiration from it - for example, the idea for an automatic watchdog.
- [`ESP32-Arduino-CAN`](https://github.com/miwagner/ESP32-Arduino-CAN) is very simple and influenced the overall code architecture.
- [`ESP32-CAN-Driver`](https://github.com/ThomasBarth/ESP32-CAN-Driver) is the basis for many implementations on the web; it contains the commands required for the `SJA1000` module.
- I also used the original `TWAI` driver from `ESP-IDF`, but found some instability (for example, a fatal error in `twai_initiate_recovery()`).

These issues led me to create **`lw_can`** - a lightweight CAN bus library for `ESP32`. 🦾

## 🌟 Features

- 🛠️ **TWAI-like interface** familiar from `ESP-IDF` (functions such as `lw_can_install`, `lw_can_transmit`, etc.)
- 📊 **Error counters** (arbitration lost, bus error, overrun, error passive, etc.) — available when compiled with LWCAN_DEBUG_COUNTERS
- 🕒 **Watchdog** that resets the CAN peripheral on error — a "smart reset" which avoids reset loops  
  _(note: this is not strictly CAN-standard behavior)_
- ⚡ **Support for various bus speeds** (configure via the provided `lw_can_bus_timing_t`)

## 🛠️ Example usage — split into functions

Below is a compact example that shows a single sketch with two clear functions: `sendCanFrame(...)` for transmitting, and `pollCan()` for receiving/processing incoming frames. These functions use the real public API and structures from `src/lw_can.h` / `src/lw_can_defs.h`.

```cpp
// Example: split send / receive into functions
// Uses API from src/lw_can.h / src/lw_can_defs.h

#include <Arduino.h>
#include <string.h>
#include "lw_can.h" // includes lw_can_defs.h

// Send a CAN frame.
// id     - standard (11-bit) or extended (29-bit) identifier
// data   - pointer to payload bytes
// len    - number of payload bytes (0..8)
// ext    - true if extended ID (LWCAN_FRAME_EXT), false for standard (LWCAN_FRAME_STD)
// rtr    - true for remote frame (LWCAN_RTR), false otherwise
// returns true if frame was queued/started
bool sendCanFrame(uint32_t id, const uint8_t* data, uint8_t len, bool ext = false, bool rtr = false)
{
    if (len > 8) return false;

    lw_can_frame_t frame;
    memset(&frame, 0, sizeof(frame));

    frame.FIR.U = 0;
    frame.FIR.B.DLC = len;
    frame.FIR.B.FF  = ext ? LWCAN_FRAME_EXT : LWCAN_FRAME_STD;
    frame.FIR.B.RTR = rtr ? LWCAN_RTR : LWCAN_NO_RTR;

    frame.MsgID = id;
    if (len)
        memcpy(frame.data.u8, data, len);

    return lw_can_transmit(frame); // returns true if queued/started
}

// Poll for a received CAN frame (non-blocking).
// If a frame is available, process it (here we print it) and return true.
bool pollCan()
{
    lw_can_frame_t rx;
    if (!lw_can_read_next_frame(rx))
        return false;

    // Simple processing: print ID, DLC and data bytes
    Serial.print("RX ID: 0x");
    Serial.println(rx.MsgID, HEX);

    Serial.print("DLC: ");
    Serial.println(rx.FIR.B.DLC);

    Serial.print("Data:");
    for (uint8_t i = 0; i < rx.FIR.B.DLC; ++i)
    {
        Serial.print(' ');
        if (rx.data.u8[i] < 0x10) Serial.print('0');
        Serial.print(rx.data.u8[i], HEX);
    }
    Serial.println();

    return true;
}

void setup()
{
    Serial.begin(115200);

    // Select pins (rxPin first, txPin second).
    gpio_num_t rxPin = GPIO_NUM_22;
    gpio_num_t txPin = GPIO_NUM_21;

    // Bus timing: choose values that match your desired bitrate.
    // Note: implementation maps prescaler into hardware BRP as (prescaler / 2) - 1.
    lw_can_bus_timing_t busTiming;
    busTiming.prescaler = 4; // example: adjust for target bitrate
    busTiming.tseg1 = 15;
    busTiming.tseg2 = 8;
    busTiming.sjw = 1;

    // Install and start the driver
    if (!lw_can_install(rxPin, txPin, busTiming, 30, 30, LWCAN_CFG_AUTO_RETRANSMIT)) {
        Serial.println("lw_can_install failed");
        while (1) delay(1000);
    }
    if (!lw_can_start()) {
        Serial.println("lw_can_start failed");
        while (1) delay(1000);
    }

    // Optionally set a filter: call after install and before start if you want to limit RX IDs
    // Example: accept all IDs by default (mask = 0xFFFFFFFF)
    // lw_can_set_filter(0x00000000, 0xFFFFFFFF);

    Serial.println("CAN ready");
}

void loop()
{
    // Example: send a test frame every second
    static uint32_t lastMillis = 0;
    if (millis() - lastMillis >= 1000) {
        lastMillis = millis();
        const uint8_t payload[8] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01, 0x02, 0x03 };
        if (sendCanFrame(0x123, payload, sizeof(payload), false, false)) {
            Serial.println("TX queued");
        } else {
            Serial.println("TX failed");
        }
    }

    // Poll for received frames frequently (non-blocking)
    pollCan();

    // Small delay to avoid a tight busy loop
    delay(10);
}
```

Notes & hints:
- `sendCanFrame(...)` builds lw_can_frame_t using the FIR union (set DLC/FF/RTR) and copies payload to `data.u8`.
- `pollCan()` calls `lw_can_read_next_frame(...)` which is non-blocking in the public API.
- Choose `busTiming.prescaler`, `tseg1`, `tseg2`, `sjw` to match your CAN bitrate. The driver maps `prescaler` to the hardware BRP as `(prescaler / 2) - 1`.
- If you want to filter incoming messages, call `lw_can_set_filter(id, mask)` after `lw_can_install()` and before `lw_can_start()`.

## 🤝 Contributing

Contributions, bug reports, and improvements are welcome. Please open issues or pull requests describing the change and the reasoning.

## 📄 License

See the repository [LICENSE](https://github.com/cziter15/lw_can/blob/main/LICENSE) file for details.

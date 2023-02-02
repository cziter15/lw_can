/**
 * @section License
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017, Thomas Barth, barth-dev.de
 * Copyright (c) 2023, Krzysztof Strehlau, github.com/cziter15
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#pragma once

#include "driver/gpio.h"
#include "lw_can_defs.h"

extern bool lw_can_install(gpio_num_t rxPin, gpio_num_t txPin, uint16_t speedKbps, uint8_t rxQueueSize, uint8_t txQueueSize);
extern bool lw_can_uninstall();
extern bool lw_can_set_filter(uint32_t msgFilter);
extern bool lw_can_start();
extern bool lw_can_stop();
extern bool lw_can_transmit(const CAN_frame_t& frame);
extern bool lw_can_read_next_frame(CAN_frame_t& outFrame);

extern uint32_t lw_can_get_wd_hits();
extern uint32_t lw_can_get_arb_lost_cnt();
extern uint32_t lw_can_get_data_overrun_cnt();
extern uint32_t lw_can_get_wake_up_cnt();
extern uint32_t lw_can_get_err_passive_cnt();
extern uint32_t lw_can_get_bus_error_cnt();
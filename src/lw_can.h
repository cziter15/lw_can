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
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

/*
	This function is used to install the CAN bus driver.

	@param rxPin CAN RX pin.
	@param txPin CAN TX pin.
	@param speedKbps CAN speed in kbps.
	@param rxQueueSize CAN RX queue size.
	@param txQueueSize CAN TX queue size.

	@return true on success, false on failure.
*/
extern bool lw_can_install(gpio_num_t rxPin, gpio_num_t txPin, uint16_t speedKbps, uint8_t rxQueueSize = 30, uint8_t txQueueSize = 30, uint8_t ocMode = LWCAN_OC_NOM);

/*
	This function is used to set CAN bus filter for incomming message.
	Must be called after install and before start.

	ISR will check it like this by
		(recvID & mask) == id

	@param id Frame ID.
	@param mask Frame ID mask.

	@return true on success, false on failure.
*/
extern bool lw_can_set_filter(uint32_t id, uint32_t mask = 0xFFFFFFFF);

/*
	This function is used to uninstall the CAN bus driver.

	@return true on success, false on failure.
*/
extern bool lw_can_uninstall(); 


/*
	This function is used to start the CAN bus driver.

	@return true on success, false on failure.
*/
extern bool lw_can_start();

/*
	This function is used to stop the CAN bus driver.

	@return true on success, false on failure.
*/
extern bool lw_can_stop();

/*
	This function transmits CAN frame.

	@param frame Frame reference

	@return true if frame has been queued for TX, false otherwise.
*/
extern bool lw_can_transmit(const lw_can_frame_t& frame);

/*
	This function reads CAN frame from queue.

	@param frame Frame reference

	@return true if frame has been read, false otherwise.
*/
extern bool lw_can_read_next_frame(lw_can_frame_t& outFrame);

/*
	This function return the count of watchdog hits (automatic bus reset).

	@return Count of watchdog hits.
*/
extern uint32_t lw_can_get_wd_hits();

/*
	This function return the count of arbitration loss.

	@return Count of arbitration loss.
*/
extern uint32_t lw_can_get_arb_lost_cnt();

/*
	This function return the count of data overrun.

	@return Count of data overrun.
*/
extern uint32_t lw_can_get_data_overrun_cnt();

/*
	This function return the count of wake ups.

	@return Count of wake ups.
*/
extern uint32_t lw_can_get_wake_up_cnt();

/*
	This function return the count of error passive ocurrences.

	@return Count of error passive ocurrences.
*/
extern uint32_t lw_can_get_err_passive_cnt();

/*
	This function return the count of bus error ocurrences.

	@return Count of bus error ocurrences.
*/
extern uint32_t lw_can_get_bus_error_cnt();

/*
	This function return the counter of errata frame resend counter.

	@return Count of errata resend frame.
*/
extern uint32_t lw_can_get_errata_resend_frame_cnt();

/*
	This function return the number of messages waiting to be transmitted.
up
	@return Count of messages to be received.
*/
extern uint32_t lw_can_get_msgs_to_rx();

/*
	This function return the number of messages to be transmitted.

	@return Count of messages to be transmitted..
*/
extern uint32_t lw_can_get_msgs_to_tx();
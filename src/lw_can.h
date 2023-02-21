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
	@param busTiming CAN BUS timing reference structure.
	@param rxQueueSize CAN RX queue size.
	@param txQueueSize CAN TX queue size.
	@param configFlags CAN config flags.

	@return true on success, false on failure.
*/
extern bool lw_can_install(gpio_num_t rxPin, gpio_num_t txPin, const lw_can_bus_timing_t& busTiming, uint8_t rxQueueSize = 30, uint8_t txQueueSize = 30, uint8_t configFlags = LWCAN_CFG_AUTO_RETRANSMIT);

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
	This function will return all bus counter information.

	@param outCounters Reference to counters structure to fill.
	@param msgsToTx Reference to fill with number of messages to transmit.
	@param msgsToRx Reference to fill with number of messages to receive.

	@return True on success, false on fail.
*/
extern bool lw_can_get_bus_counters(lw_can_bus_counters& outCounters, uint32_t& msgsToTx, uint32_t& msgsToRx);
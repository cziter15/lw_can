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
#include "lw_can.h"
#include <stdint.h>
#include <math.h>
#include <byteswap.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "soc/dport_reg.h"

//===================================================================================================================
// Critical sections.
//===================================================================================================================
static portMUX_TYPE globalCanSpinLock 	=	portMUX_INITIALIZER_UNLOCKED;
#define LWCAN_ENTER_CRITICAL()				portENTER_CRITICAL(&globalCanSpinLock)
#define LWCAN_EXIT_CRITICAL()				portEXIT_CRITICAL(&globalCanSpinLock)
#define LWCAN_ENTER_CRITICAL_ISR()			portENTER_CRITICAL_ISR(&globalCanSpinLock)
#define LWCAN_EXIT_CRITICAL_ISR()			portEXIT_CRITICAL_ISR(&globalCanSpinLock)

//===================================================================================================================
// Driver object
//===================================================================================================================
typedef struct 
{
	uint32_t id;
	uint32_t mask;
} lw_can_filter_t;

typedef union
{
	uint8_t U;												// Unsigned access 
	struct 
	{
		bool isDriverStarted : 1;							// Flag to indicate if CAN driver is started.
		bool hasAnyFrameInTxBuffer : 1;						// Flag to indicate if CAN driver is transmitting.
	} B;
} lw_can_driver_state;

typedef struct
{
	lw_can_filter_t filter;									// Filter settings.

	gpio_num_t txPin;										// CAN TX pin.
	gpio_num_t rxPin;										// CAN RX pin.
	uint16_t speedKbps;										// CAN speed in kbps.
	uint8_t ocMode;											// CAN output control mode.

	uint8_t rxQueueSize;									// CAN RX queue size.
	uint8_t txQueueSize;									// CAN TX queue size.
	QueueHandle_t rxQueue;									// CAN RX queue handle.
	QueueHandle_t txQueue;									// CAN TX queue handle.

	lw_can_frame_t savedFrame;								// Temporary frame to workaround chip bugs.
	lw_can_bus_counters counters;							// Statistics counters.

	intr_handle_t intrHandle;								// CAN interrupt handle.
	lw_can_driver_state state;								// Driver state flags.

	void resetBusCounters()
	{
		counters.arbLostCnt = 0;
		counters.busErrorCnt = 0;
		counters.dataOverrunCnt = 0;
		counters.errPassiveCnt = 0;
		counters.busErrorCnt = 0;
		counters.errataResendFrameCnt = 0;
	}

	void resetFilter()
	{
		filter.mask = 0;
		filter.id = 0;
	}
} lw_can_driver_obj_t;

lw_can_driver_obj_t* pCanDriverObj{nullptr}; 			// Driver object pointer.

//===================================================================================================================
// Required forward declarations.
//===================================================================================================================
void IRAM_ATTR lw_can_interrupt(void *arg);
//===================================================================================================================
// Spinlock free / low level functions.
//===================================================================================================================
void IRAM_ATTR ll_lw_can_reset_filter_reg()
{
	MODULE_CAN->MOD.B.AFM = 0;
	MODULE_CAN->MBX_CTRL.ACC.CODE[0] = 0xff;
	MODULE_CAN->MBX_CTRL.ACC.CODE[1] = 0xff;
	MODULE_CAN->MBX_CTRL.ACC.CODE[2] = 0xff;
	MODULE_CAN->MBX_CTRL.ACC.CODE[3] = 0xff;
	MODULE_CAN->MBX_CTRL.ACC.MASK[0] = 0xff;
	MODULE_CAN->MBX_CTRL.ACC.MASK[1] = 0xff;
	MODULE_CAN->MBX_CTRL.ACC.MASK[2] = 0xff;
	MODULE_CAN->MBX_CTRL.ACC.MASK[3] = 0xff;
}

void IRAM_ATTR ll_lw_can_enable_peripheral()
{
	DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_CAN_CLK_EN);
	DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_CAN_RST);
	MODULE_CAN->MOD.B.RM = 1;
	MODULE_CAN->IER.U = 0;
}

void IRAM_ATTR ll_lw_can_disable_peripheral()
{
	MODULE_CAN->MOD.B.RM = 1;
	DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_CAN_CLK_EN);
	DPORT_SET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_CAN_RST);
	MODULE_CAN->IER.U = 0;
}

void IRAM_ATTR ll_lw_can_clear_ir_and_ecc()
{
	MODULE_CAN->TXERR.U = 0;
	MODULE_CAN->RXERR.U = 0;
	(void)MODULE_CAN->ECC;
	(void)MODULE_CAN->IR.U;
}

void IRAM_ATTR ll_lw_can_read_frame_phy()
{
	lw_can_frame_t frame;
	BaseType_t xHigherPriorityTaskWoken;

	frame.FIR.U = MODULE_CAN->MBX_CTRL.FCTRL.FIR.U;
	frame.MsgID = frame.FIR.B.FF == LWCAN_FRAME_STD ? LWCAN_GET_STD_ID : LWCAN_GET_EXT_ID;

	if ((frame.MsgID & pCanDriverObj->filter.mask) == pCanDriverObj->filter.id)
	{
		if (frame.FIR.B.FF == LWCAN_FRAME_STD)
		{
			for (uint8_t thisByte = 0; thisByte < frame.FIR.B.DLC; ++thisByte)
				frame.data.u8[thisByte] = MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.STD.data[thisByte];
		}
		else
		{
			for (uint8_t thisByte = 0; thisByte < frame.FIR.B.DLC; ++thisByte)
				frame.data.u8[thisByte] = MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.data[thisByte];
		}
		xQueueSendFromISR(pCanDriverObj->rxQueue, &frame, &xHigherPriorityTaskWoken);
	}

	MODULE_CAN->CMR.B.RRB = 1;
}

void IRAM_ATTR ll_lw_can_write_frame_phy(const lw_can_frame_t& frame) 
{
	MODULE_CAN->MBX_CTRL.FCTRL.FIR.U = frame.FIR.U;
	if (frame.FIR.B.FF == LWCAN_FRAME_STD) 
	{
		LWCAN_SET_STD_ID(frame.MsgID);
		for (uint8_t thisByte = 0; thisByte < frame.FIR.B.DLC; ++thisByte)
			MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.STD.data[thisByte] = frame.data.u8[thisByte];
	}
	else 
	{
		LWCAN_SET_EXT_ID(frame.MsgID);
		for (uint8_t thisByte = 0; thisByte < frame.FIR.B.DLC; ++thisByte)
			MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.data[thisByte] = frame.data.u8[thisByte];
	}
	pCanDriverObj->savedFrame = frame; // need to cache to workaround errata bug.
	MODULE_CAN->CMR.B.TR = 1;
}

void IRAM_ATTR ll_lw_can_rst_from_isr()
{
	// Save registers.
	uint32_t BTR0 = MODULE_CAN->BTR0.U;
	uint32_t BTR1 = MODULE_CAN->BTR1.U;
	uint32_t CDR = MODULE_CAN->CDR.U;
	uint32_t IER = MODULE_CAN->IER.U;

	// TODO: 
	// maybe abort transmission via MODULE_CAN->CMR...

	ll_lw_can_disable_peripheral();
	ll_lw_can_enable_peripheral();
	ll_lw_can_reset_filter_reg();
	ll_lw_can_clear_ir_and_ecc();

	// Restore registers.
	MODULE_CAN->BTR0.U = BTR0;
	MODULE_CAN->BTR1.U = BTR1;
	MODULE_CAN->IER.U = IER;
	MODULE_CAN->CDR.U = CDR;

	// Release reset mode.
	MODULE_CAN->MOD.B.RM = 0;
}

void IRAM_ATTR ll_lw_can_interrupt()
{
	// Read and clear interrupts.
	uint32_t interrupt {MODULE_CAN->IR.U};

	// Handle counters.
	if (interrupt & LWCAN_IRQ_ARB_LOST)
		++pCanDriverObj->counters.arbLostCnt;
	
	if (interrupt & LWCAN_IRQ_DATA_OVERRUN)
		++pCanDriverObj->counters.dataOverrunCnt;

	if (interrupt & LWCAN_IRQ_WAKEUP)
		++pCanDriverObj->counters.wakeUpCnt;

	if (interrupt & LWCAN_IRQ_ERR_PASSIVE)
		++pCanDriverObj->counters.errPassiveCnt;

	if (interrupt & LWCAN_IRQ_BUS_ERR)
		++pCanDriverObj->counters.busErrorCnt;

	// Read frames from buffer.
	for (unsigned int rxFrames = 0; rxFrames < MODULE_CAN->RMC.B.RMC; ++rxFrames)
	{
		ll_lw_can_read_frame_phy();
	}

	// Handle error interrupts.
	if (interrupt & ( LWCAN_IRQ_ERR				//0x4
					| LWCAN_IRQ_DATA_OVERRUN	//0x8
					| LWCAN_IRQ_WAKEUP			//0x10
					| LWCAN_IRQ_ERR_PASSIVE		//0x20
					| LWCAN_IRQ_BUS_ERR			//0x80
	))
	{
		// Reset module but preserve registers.
		ll_lw_can_rst_from_isr();

		// Requeue the frame if we broke sending.
		if (pCanDriverObj->state.B.hasAnyFrameInTxBuffer)
		{
			ll_lw_can_write_frame_phy(pCanDriverObj->savedFrame);
			++pCanDriverObj->counters.errataResendFrameCnt;
		}
		return;
	}

	// Handle sending in case there is no error.
	if (pCanDriverObj->state.B.hasAnyFrameInTxBuffer && MODULE_CAN->SR.B.TBS) 
	{
		lw_can_frame_t frame;
		if (xQueueIsQueueEmptyFromISR(pCanDriverObj->txQueue) == pdFALSE)
		{
			xQueueReceiveFromISR(pCanDriverObj->txQueue, &frame, nullptr);
			ll_lw_can_write_frame_phy(frame);
		}
		else 
		{
			pCanDriverObj->state.B.hasAnyFrameInTxBuffer = false;
		}
	}
}

bool ll_lw_can_start()
{
	// If not installed or started, return false.
	if (!pCanDriverObj || pCanDriverObj->state.B.isDriverStarted)
		return false;

	// Time quanta
	double quanta;

	// Enable module
	ll_lw_can_enable_peripheral();

	// Configure TX pin
	gpio_set_level(pCanDriverObj->txPin, 1);
	gpio_set_direction(pCanDriverObj->txPin,GPIO_MODE_OUTPUT);
	gpio_matrix_out(pCanDriverObj->txPin,CAN_TX_IDX,0,0);
	gpio_pad_select_gpio(pCanDriverObj->txPin);

	// Configure RX pin
	gpio_set_direction(pCanDriverObj->rxPin,GPIO_MODE_INPUT);
	gpio_matrix_in(pCanDriverObj->rxPin,CAN_RX_IDX,0);
	gpio_pad_select_gpio(pCanDriverObj->rxPin);

	// Set to PELICAN mode
	MODULE_CAN->CDR.B.CAN_M = 1;
	
	// Synchronization jump width is the same for all baud rates
	MODULE_CAN->BTR0.B.SJW = 3;

	// TSEG2 is the same for all baud rates
	MODULE_CAN->BTR1.B.TSEG2 = 1;

	// Select time quantum and set TSEG1
	switch(pCanDriverObj->speedKbps)
	{
		case 1000:
			MODULE_CAN->BTR1.B.TSEG1 = 4;
			quanta = 0.125;
		break;
		case 800:
			MODULE_CAN->BTR1.B.TSEG1 = 6;
			quanta = 0.125;
		break;
		case 33:
			//changes everything...
			MODULE_CAN->BTR1.B.TSEG2 = 6;
			MODULE_CAN->BTR1.B.TSEG1 = 15; //16 + 1 + 7 = 24
			quanta = ((float)1000.0f / 33.3f) / 24.0f;
		break;
		default:
			MODULE_CAN->BTR1.B.TSEG1 = 12;
			quanta = ((float)1000.0f / (float)pCanDriverObj->speedKbps) / 16.0f;
	}

	// Set baud rate prescaler - APB_CLK_FREQ should be 80M.
	MODULE_CAN->BTR0.B.BRP = (uint8_t)round((((APB_CLK_FREQ * quanta) / 2) - 1)/1000000)-1;

	/* 
		Set sampling

		1 -> triple; the bus is sampled three times; recommended for low/medium speed buses     
		(class A and B) where filtering spikes on the bus line is beneficial 
		
		0 -> single; the bus is sampled once; recommended for high speed buses (SAE class C).
	*/
	MODULE_CAN->BTR1.B.SAM = 1;

	// Enable all interrupts (BUT NOT BIT 4 which has turned into a baud rate scalar!).
	MODULE_CAN->IER.U = 0xEF; //1110 1111
	
	// TODO: GTS REGISTER?

	// Set to normal mode.
	MODULE_CAN->OCR.B.OCMODE = pCanDriverObj->ocMode;

	// Allocate queues.
	pCanDriverObj->rxQueue = xQueueCreate(pCanDriverObj->rxQueueSize, sizeof(lw_can_frame_t));
	pCanDriverObj->txQueue = xQueueCreate(pCanDriverObj->txQueueSize, sizeof(lw_can_frame_t));

	// Reset counters.
	pCanDriverObj->resetBusCounters();
	
	// Install CAN interrupt service.
	esp_intr_alloc(ETS_CAN_INTR_SOURCE, 0, lw_can_interrupt, nullptr, &pCanDriverObj->intrHandle);

	// Clear error counters and current interrupt flag.
	ll_lw_can_reset_filter_reg();
	ll_lw_can_clear_ir_and_ecc();

	// Set state.
	pCanDriverObj->state.B.isDriverStarted = true;

	// Showtime. Release Reset Mode.
	MODULE_CAN->MOD.B.RM = 0;

	return true;
}

bool ll_lw_can_stop()
{
	// If not installed or not started, return false.
	if (!pCanDriverObj || !pCanDriverObj->state.B.isDriverStarted)
		return false;

	// Turn off modkule.
	ll_lw_can_stop();

	// Remove interrupt and semaphore.
	esp_intr_free(pCanDriverObj->intrHandle);

	// Delete queues.
	vQueueDelete(pCanDriverObj->txQueue);
	vQueueDelete(pCanDriverObj->rxQueue);

	// Clear state flags.
	pCanDriverObj->state.B.isDriverStarted = false;

	return true;
}

bool ll_lw_can_set_filter(uint32_t id, uint32_t mask)
{
	// If not installed or already started, return false.
	if (!pCanDriverObj || pCanDriverObj->state.B.isDriverStarted)
		return false;

	pCanDriverObj->filter.mask = mask;
	pCanDriverObj->filter.id = id;

	return true;
}

bool ll_lw_can_install(gpio_num_t rxPin, gpio_num_t txPin, uint16_t speedKbps, uint8_t rxQueueSize, uint8_t txQueueSize, uint8_t ocMode)
{
	// If already installed, return false.
	if (pCanDriverObj)
		return false;

	pCanDriverObj = new lw_can_driver_obj_t();

	// Setup pins.
	pCanDriverObj->txPin = txPin;
	pCanDriverObj->rxPin = rxPin;

	// Setup speed.
	pCanDriverObj->speedKbps = speedKbps;

	// Setup OC mode.
	pCanDriverObj->ocMode = ocMode;

	// Copy queue sizes.
	pCanDriverObj->rxQueueSize = rxQueueSize;
	pCanDriverObj->txQueueSize = txQueueSize;

	// Reset states.
	pCanDriverObj->state.U = 0;

	// Reset filter.
	pCanDriverObj->resetBusCounters();

	return true;
}

bool ll_lw_can_uninstall()
{
	// If not installed, return false.
	if (!pCanDriverObj)
		return false;

	// Stop driver if working.
	if (pCanDriverObj->state.B.isDriverStarted)
		ll_lw_can_stop();

	// Reset pins.
	gpio_reset_pin(pCanDriverObj->rxPin);
	gpio_reset_pin(pCanDriverObj->txPin);

	// Delete driver object.
	delete pCanDriverObj;
	pCanDriverObj = nullptr;

	return true;
}
//===================================================================================================================
// Wrapper routines.
//===================================================================================================================
void IRAM_ATTR lw_can_interrupt(void* arg)
{
	LWCAN_ENTER_CRITICAL_ISR();
	ll_lw_can_interrupt();
	LWCAN_EXIT_CRITICAL_ISR();
}
//===================================================================================================================
// PUBLIC API
// Remember to use spinlock properly.
//===================================================================================================================
bool lw_can_install(gpio_num_t rxPin, gpio_num_t txPin, uint16_t speedKbps, uint8_t rxQueueSize, uint8_t txQueueSize, uint8_t ocMode)
{
	bool driverInstalled;
	LWCAN_ENTER_CRITICAL();
	driverInstalled = ll_lw_can_install(rxPin, txPin, speedKbps, rxQueueSize, txQueueSize, ocMode);
	LWCAN_EXIT_CRITICAL();
	return driverInstalled;
}

bool lw_can_uninstall()
{
	bool driverUninstalled;
	LWCAN_ENTER_CRITICAL();
	driverUninstalled = ll_lw_can_uninstall();
	LWCAN_EXIT_CRITICAL();
	return driverUninstalled;
}

bool lw_can_start()
{
	bool driverStarted;
	LWCAN_ENTER_CRITICAL();
	driverStarted = ll_lw_can_start();
	LWCAN_EXIT_CRITICAL();
	return driverStarted;
}

bool lw_can_stop()
{
	bool driverStopped;
	LWCAN_ENTER_CRITICAL();
	driverStopped = ll_lw_can_stop();
	LWCAN_EXIT_CRITICAL();
	return driverStopped;
}

bool lw_can_transmit(const lw_can_frame_t& frame)
{
	bool frameQueued{false};
	LWCAN_ENTER_CRITICAL();
	if (pCanDriverObj && pCanDriverObj->state.B.isDriverStarted)
	{
		if (pCanDriverObj->state.B.hasAnyFrameInTxBuffer)
		{
			frameQueued = xQueueSend(pCanDriverObj->txQueue, &frame, 0) == pdTRUE;
		}
		else
		{
			pCanDriverObj->state.B.hasAnyFrameInTxBuffer = true;
			ll_lw_can_write_frame_phy(frame);
			frameQueued = true;
		}
	}
	LWCAN_EXIT_CRITICAL();
	return frameQueued;
}

bool lw_can_read_next_frame(lw_can_frame_t& outFrame)
{
	QueueHandle_t rxQueue;
	LWCAN_ENTER_CRITICAL();
	rxQueue = (pCanDriverObj && pCanDriverObj->state.B.isDriverStarted) ? pCanDriverObj->rxQueue : nullptr;
	LWCAN_EXIT_CRITICAL();
	return rxQueue != NULL && xQueueReceive(rxQueue, &outFrame, 0) == pdTRUE;
}

bool lw_can_set_filter(uint32_t id, uint32_t mask)
{
	bool filterSetStatus;
	LWCAN_ENTER_CRITICAL();
	filterSetStatus = ll_lw_can_set_filter(id, mask);
	LWCAN_EXIT_CRITICAL();
	return filterSetStatus;
}

bool lw_can_get_bus_counters(lw_can_bus_counters& outCounters, uint32_t& msgsToTx, uint32_t& msgsToRx)
{
	bool readDone{false};
	LWCAN_ENTER_CRITICAL();
	if (pCanDriverObj)
	{
		outCounters = pCanDriverObj->counters;
		msgsToRx = uxQueueMessagesWaiting(pCanDriverObj->rxQueue);
		msgsToTx = uxQueueMessagesWaiting(pCanDriverObj->txQueue);
		readDone = true;
	}
	LWCAN_EXIT_CRITICAL();
	return readDone;
}
//===================================================================================================================
// END PUBLIC API
//===================================================================================================================
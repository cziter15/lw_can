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
static portMUX_TYPE globalCanSpinLock 	=	 portMUX_INITIALIZER_UNLOCKED;
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
	uint8_t U;											// Unsigned access 
	struct 
	{
		bool isDriverStarted : 1;						// Flag to indicate if CAN driver is started.
		bool needResetPeripheral : 1;					// Flag to indicate if need to reset CAN peripheral.
		bool hasAnyFrameInTxBuffer : 1;					// Flag to indicate if CAN driver is transmitting.
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
	lw_can_bus_counters counters;								// Statistics counters.

	intr_handle_t intrHandle;								// CAN interrupt handle.
	TaskHandle_t wdtHandle;									// CAN watchdog task handle.

	lw_can_driver_state state;								// Driver state flags.
} lw_can_driver_obj_t;

inline void pdo_reset_bus_counters(lw_can_driver_obj_t* pdo)
{
	pdo->counters.arbLostCnt = 0;
	pdo->counters.busErrorCnt = 0;
	pdo->counters.dataOverrunCnt = 0;
	pdo->counters.errPassiveCnt = 0;
	pdo->counters.busErrorCnt = 0;
	pdo->counters.errataResendFrameCnt = 0;
}

inline void pdo_reset_filter(lw_can_driver_obj_t * pdo)
{
	pdo->filter.mask = 0;
	pdo->filter.id = 0;
}

static lw_can_driver_obj_t* pCanDriverObj = NULL; 			// Driver object pointer.

//===================================================================================================================
// Required forward declarations.
//===================================================================================================================
void IRAM_ATTR lw_can_interrupt(void *arg);
void lw_can_watchdog(void *param);

//===================================================================================================================
// Spinlock free functions.
//===================================================================================================================
void IRAM_ATTR impl_lw_read_frame_phy()
{
	uint8_t thisByte;
	lw_can_frame_t frame;
	BaseType_t xHigherPriorityTaskWoken;

	// Copy FIR.
	frame.FIR.U = MODULE_CAN->MBX_CTRL.FCTRL.FIR.U;

	// Set frame ID depedning on framer type.
	frame.MsgID = frame.FIR.B.FF == LWCAN_FRAME_STD ? LWCAN_GET_STD_ID : LWCAN_GET_EXT_ID;

	// Check frame filtering and copy bytes if match.
	if ((frame.MsgID & pCanDriverObj->filter.mask) == pCanDriverObj->filter.id)
	{
		if(frame.FIR.B.FF == LWCAN_FRAME_STD)
		{
			for(thisByte = 0; thisByte < frame.FIR.B.DLC; ++thisByte)
			{
				frame.data.u8[thisByte] = MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.STD.data[thisByte];
			}
		}
		else
		{
			for(thisByte = 0; thisByte < frame.FIR.B.DLC; ++thisByte)
			{
				frame.data.u8[thisByte] = MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.data[thisByte];
			}
		}
		
		// Send to RX queue.
		xQueueSendFromISR(pCanDriverObj->rxQueue, &frame, &xHigherPriorityTaskWoken);
	}

	MODULE_CAN->CMR.B.RRB = 0x1;
}

void IRAM_ATTR impl_write_frame_phy(const lw_can_frame_t& frame) 
{
	uint8_t thisByte;
	MODULE_CAN->MBX_CTRL.FCTRL.FIR.U = frame.FIR.U;
	if (frame.FIR.B.FF == LWCAN_FRAME_STD) 
	{
		LWCAN_SET_STD_ID(frame.MsgID);
		for (thisByte = 0; thisByte < frame.FIR.B.DLC; ++thisByte)
			MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.STD.data[thisByte] = frame.data.u8[thisByte];
	}
	else 
	{
		LWCAN_SET_EXT_ID(frame.MsgID);
		for (thisByte = 0; thisByte < frame.FIR.B.DLC; ++thisByte)
			MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.data[thisByte] = frame.data.u8[thisByte];
	}

	pCanDriverObj->savedFrame = frame;

	MODULE_CAN->CMR.B.TR = 0x1;
}

bool impl_lw_can_start(bool notInReset = true)
{
	if (pCanDriverObj && !pCanDriverObj->state.B.isDriverStarted)
	{
		// Time quanta
		double quanta;

		// Enable module
		DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_CAN_CLK_EN);
		DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_CAN_RST);

		// First thing once module is enabled at hardware level is to make sure it is in reset
		MODULE_CAN->MOD.B.RM = 0x1; 

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
		MODULE_CAN->CDR.B.CAN_M = 0x1;

 		// Disable all interrupt sources until we're ready
		MODULE_CAN->IER.U = 0;

		// Clear interrupt flags
		(void)MODULE_CAN->IR.U;
		
		// Synchronization jump width is the same for all baud rates
		MODULE_CAN->BTR0.B.SJW = 0x1;

		// TSEG2 is the same for all baud rates
		MODULE_CAN->BTR1.B.TSEG2 = 0x1;

		// Select time quantum and set TSEG1
		switch(pCanDriverObj->speedKbps)
		{
			case 1000:
				MODULE_CAN->BTR1.B.TSEG1 = 0x4;
				quanta = 0.125;
			break;
			case 800:
				MODULE_CAN->BTR1.B.TSEG1 = 0x6;
				quanta = 0.125;
			break;
			case 33:
				//changes everything...
				MODULE_CAN->BTR1.B.TSEG2 = 0x6;
				MODULE_CAN->BTR1.B.TSEG1 = 0xf; //16 + 1 + 7 = 24
				quanta = ((float)1000.0f / 33.3f) / 24.0f;
			break;
			default:
				MODULE_CAN->BTR1.B.TSEG1 = 0xc;
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
		MODULE_CAN->BTR1.B.SAM = 0x1;

		// Enable all interrupts (BUT NOT BIT 4 which has turned into a baud rate scalar!).
		MODULE_CAN->IER.U = 0xEF; //1110 1111

		// Set acceptance filter.
		MODULE_CAN->MOD.B.AFM = 0;
		MODULE_CAN->MBX_CTRL.ACC.CODE[0] = 0xff;
		MODULE_CAN->MBX_CTRL.ACC.CODE[1] = 0xff;
		MODULE_CAN->MBX_CTRL.ACC.CODE[2] = 0xff;
		MODULE_CAN->MBX_CTRL.ACC.CODE[3] = 0xff;
		MODULE_CAN->MBX_CTRL.ACC.MASK[0] = 0xff;
		MODULE_CAN->MBX_CTRL.ACC.MASK[1] = 0xff;
		MODULE_CAN->MBX_CTRL.ACC.MASK[2] = 0xff;
		MODULE_CAN->MBX_CTRL.ACC.MASK[3] = 0xff;

		// Set to normal mode.
		MODULE_CAN->OCR.B.OCMODE = pCanDriverObj->ocMode;

		// Clear error counters.
		MODULE_CAN->TXERR.U = 0;
		MODULE_CAN->RXERR.U = 0;
		(void)MODULE_CAN->ECC;

		// Clear interrupt flags.
		(void)MODULE_CAN->IR.U;

		if (notInReset)
		{
			// Allocate queues.
			pCanDriverObj->rxQueue = xQueueCreate(pCanDriverObj->rxQueueSize, sizeof(lw_can_frame_t));
			pCanDriverObj->txQueue = xQueueCreate(pCanDriverObj->txQueueSize, sizeof(lw_can_frame_t));

			// Reset counters.
			pdo_reset_bus_counters(pCanDriverObj);
			
			// Install CAN interrupt service.
			esp_intr_alloc(ETS_CAN_INTR_SOURCE, 0, lw_can_interrupt, NULL, &pCanDriverObj->intrHandle);
		}

		// Set state.
		pCanDriverObj->state.B.isDriverStarted = true;

		// Showtime. Release Reset Mode.
		MODULE_CAN->MOD.B.RM = 0;

		return true;
	}
	
	return false;
}

bool impl_lw_can_stop(bool notInReset = true)
{
	if (pCanDriverObj && pCanDriverObj->state.B.isDriverStarted)
	{
		// Reset the module.
		MODULE_CAN->MOD.B.RM = 0x1;
		DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_CAN_CLK_EN);
		DPORT_SET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_CAN_RST);
		MODULE_CAN->IER.U = 0;

		if (notInReset)
		{
			// Remove interrupt and semaphore.
			esp_intr_free(pCanDriverObj->intrHandle);

			// Delete queues.
			vQueueDelete(pCanDriverObj->txQueue);
			vQueueDelete(pCanDriverObj->rxQueue);
		}

		// Clear state flags.
		pCanDriverObj->state.B.needResetPeripheral = false;
		pCanDriverObj->state.B.isDriverStarted = false;

		return true;
	}

	return false;
}

bool impl_lw_can_set_filter(uint32_t id, uint32_t mask)
{
	if (pCanDriverObj != NULL && !pCanDriverObj->state.B.isDriverStarted)
	{
		pCanDriverObj->filter.mask = mask;
		pCanDriverObj->filter.id = id;
		return true;
	}
	return false;
}

bool impl_lw_can_install(gpio_num_t rxPin, gpio_num_t txPin, uint16_t speedKbps, uint8_t rxQueueSize, uint8_t txQueueSize, uint8_t ocMode)
{
	if (pCanDriverObj == NULL)
	{
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

		// Reset wdt counter.
		pCanDriverObj->counters.wdHitCnt = 0;

		// Reset states.
		pCanDriverObj->state.U = 0;

		// Reset filter.
		pdo_reset_filter(pCanDriverObj);

		// Create watchdog task.
		xTaskCreatePinnedToCore(&lw_can_watchdog, "lw_can_wdt", 2048, NULL, 10, &pCanDriverObj->wdtHandle, 1);

		return true;
	}

	return false;
}

bool impl_lw_can_uninstall()
{
	if (pCanDriverObj != NULL)
	{
		// Stop driver if working.
		if (pCanDriverObj->state.B.isDriverStarted)
			impl_lw_can_stop();
		
		// Delete watchdog task.
		vTaskDelete(pCanDriverObj->wdtHandle);

		// Reset pins.
		gpio_reset_pin(pCanDriverObj->rxPin);
		gpio_reset_pin(pCanDriverObj->txPin);

		// Delete driver object.
		delete pCanDriverObj;
		pCanDriverObj = NULL;

		return true;
	}

	return false;
}

//===================================================================================================================
// Remember to use spinlock properly.
//===================================================================================================================
void IRAM_ATTR lw_can_interrupt(void* arg)
{
	lw_can_frame_t frame;
	uint32_t interrupt;

	LWCAN_ENTER_CRITICAL_ISR();

	// Read and clear interrupts.
	interrupt = MODULE_CAN->IR.U;

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

	// We should always read buffered frames, even when error occurred.
	for (unsigned int rxFrames = 0; rxFrames < MODULE_CAN->RMC.B.RMC; ++rxFrames)
	{
		impl_lw_read_frame_phy();
	}

	// Handle error interrupts.
	if (interrupt & (LWCAN_IRQ_ERR				//0x4
					| LWCAN_IRQ_DATA_OVERRUN	//0x8
					| LWCAN_IRQ_WAKEUP			//0x10
					| LWCAN_IRQ_ERR_PASSIVE		//0x20
					| LWCAN_IRQ_BUS_ERR			//0x80
	))
	{
		pCanDriverObj->state.B.needResetPeripheral = true;
		esp_intr_disable(pCanDriverObj->intrHandle);
	}
	// Handle sending in case there is no error.
	else if (pCanDriverObj->state.B.hasAnyFrameInTxBuffer && MODULE_CAN->SR.B.TBS) 
	{
		if (xQueueIsQueueEmptyFromISR(pCanDriverObj->txQueue) == pdFALSE)
		{
			xQueueReceiveFromISR(pCanDriverObj->txQueue, &frame, NULL);
			impl_write_frame_phy(frame);
		}
		else 
		{
			pCanDriverObj->state.B.hasAnyFrameInTxBuffer = false;
		}
	}

	LWCAN_EXIT_CRITICAL_ISR();
}

void lw_can_watchdog(void* param)
{
	const TickType_t watchdogSmallDelay = pdMS_TO_TICKS(50);
	
	while (true)
	{
		// Delay for a while.
		vTaskDelay(watchdogSmallDelay);

		LWCAN_ENTER_CRITICAL();
		if (pCanDriverObj && pCanDriverObj->state.B.isDriverStarted && pCanDriverObj->state.B.needResetPeripheral)
		{	
			// Do CAN peripheral reset.
			impl_lw_can_stop(false);
			impl_lw_can_start(false);

			// Enable INTR handler back (it is disabled on reset request).
			esp_intr_enable(pCanDriverObj->intrHandle);

			// Increment watchdog counter.
			++pCanDriverObj->counters.wdHitCnt;

			// If we reset due to errata workaround, then send pedning frame.
			if (pCanDriverObj->state.B.hasAnyFrameInTxBuffer)
			{
				impl_write_frame_phy(pCanDriverObj->savedFrame);
				++pCanDriverObj->counters.errataResendFrameCnt;
			}
		}
		LWCAN_EXIT_CRITICAL();
	}
}

//===================================================================================================================
// PUBLIC API
// Remember to use spinlock properly.
//===================================================================================================================
bool lw_can_install(gpio_num_t rxPin, gpio_num_t txPin, uint16_t speedKbps, uint8_t rxQueueSize, uint8_t txQueueSize, uint8_t ocMode)
{
	bool driverInstalled;
	LWCAN_ENTER_CRITICAL();
	driverInstalled = impl_lw_can_install(rxPin, txPin, speedKbps, rxQueueSize, txQueueSize, ocMode);
	LWCAN_EXIT_CRITICAL();
	return driverInstalled;
}

bool lw_can_uninstall()
{
	bool driverUninstalled;
	LWCAN_ENTER_CRITICAL();
	driverUninstalled = impl_lw_can_uninstall();
	LWCAN_EXIT_CRITICAL();
	return driverUninstalled;
}

bool lw_can_start()
{
	bool driverStarted;
	LWCAN_ENTER_CRITICAL();
	driverStarted = impl_lw_can_start();
	LWCAN_EXIT_CRITICAL();
	return driverStarted;
}

bool lw_can_stop()
{
	bool driverStopped;
	LWCAN_ENTER_CRITICAL();
	driverStopped = impl_lw_can_stop();
	LWCAN_EXIT_CRITICAL();
	return driverStopped;
}

bool lw_can_transmit(const lw_can_frame_t& frame)
{
	bool frameQueued = false;
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
			impl_write_frame_phy(frame);
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
	rxQueue = (pCanDriverObj != NULL && pCanDriverObj->state.B.isDriverStarted) ? pCanDriverObj->rxQueue : NULL;
	LWCAN_EXIT_CRITICAL();
	return rxQueue != NULL && xQueueReceive(rxQueue, &outFrame, 0) == pdTRUE;
}

bool lw_can_set_filter(uint32_t id, uint32_t mask)
{
	bool filterSetStatus;
	LWCAN_ENTER_CRITICAL();
	filterSetStatus = impl_lw_can_set_filter(id, mask);
	LWCAN_EXIT_CRITICAL();
	return filterSetStatus;
}

bool lw_can_get_bus_counters(lw_can_bus_counters& outCounters, uint32_t& msgsToTx, uint32_t& msgsToRx)
{
	bool readDone = false;
	LWCAN_ENTER_CRITICAL();
	if (pCanDriverObj != NULL)
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
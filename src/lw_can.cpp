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
	uint8_t	FM;												// Filter mode.
	union
	{
		uint32_t U;
		struct 
		{
			uint8_t R0;										// ACR0 register.
			uint8_t R1;										// ACR1 register.
			uint8_t R2;										// ACR2 register.
			uint8_t R3;										// ACR3 register.
		} B;
	} AC ;
	union
	{
		uint32_t U;
		struct 
		{
			uint8_t R0;										// ACR0 register.
			uint8_t R1;										// ACR1 register.
			uint8_t R2;										// ACR2 register.
			uint8_t R3;										// ACR3 register.
		} B;
	} AM;
} lw_can_filter_t;

typedef struct 
{
	bool isStarted : 1;										// Flag to indicate if CAN driver is started.
	bool needReset : 1;										// Flag to indicate if need to reset CAN peripheral.
	bool txOccupied : 1;								// Flag to indicate if CAN driver is transmitting.
} lw_can_driver_state_t;

typedef struct
{
	lw_can_filter_t filter;									// Filter settings.

	gpio_num_t txPin;										// CAN TX pin.
	gpio_num_t rxPin;										// CAN RX pin.
	uint16_t speedKbps;										// CAN speed in kbps.

	uint8_t rxQueueSize;									// CAN RX queue size.
	uint8_t txQueueSize;									// CAN TX queue size.

	QueueHandle_t rxQueue;									// CAN RX queue handle.
	QueueHandle_t txQueue;									// CAN TX queue handle.

	intr_handle_t intrHandle;								// CAN interrupt handle.
	TaskHandle_t wdtHandle;									// CAN watchdog task handle.

	uint8_t ocMode;											// CAN output control mode.

	uint32_t wd_hit_cnt;									// Watchdog hit counter.

	uint32_t arb_lost_cnt;									// Arbitration lost counter.
	uint32_t data_overrun_cnt;								// Data overrun counter.
	uint32_t wake_up_cnt;									// Wake up counter.
	uint32_t err_passive_cnt;								// Error passive counter.
	uint32_t bus_error_cnt;									// Bus error counter.
	uint32_t total_int_cnt;									// Total interrupt counter.

	lw_can_driver_state_t state;							// Driver state.

} lw_can_driver_obj_t;


inline void pdo_reset_bus_counters(lw_can_driver_obj_t* pdo)
{
	pdo->arb_lost_cnt = 0;
	pdo->bus_error_cnt = 0;
	pdo->data_overrun_cnt = 0;
	pdo->err_passive_cnt = 0;
	pdo->wake_up_cnt = 0;
	pdo->total_int_cnt = 0;
}

inline void pdo_reset_filter(lw_can_driver_obj_t * pdo)
{
	pdo->filter.FM = LWCAN_FILTER_DUAL;
	pdo->filter.AM.U = 0;
	pdo->filter.AC.U = 0xFFFFFFFF;
}

static lw_can_driver_obj_t* pCanDriverObj = NULL; 			// Driver object pointer

//===================================================================================================================
// Required forward declarations
//===================================================================================================================
void IRAM_ATTR lw_can_interrupt(void *arg);
void lw_can_watchdog(void *param);

//===================================================================================================================
// Spinlock free functions
//===================================================================================================================
void IRAM_ATTR impl_lw_read_frame_phy()
{
	uint8_t thisByte;
	lw_can_frame_t frame;
	BaseType_t xHigherPriorityTaskWoken;

	frame.FIR.U = MODULE_CAN->MBX_CTRL.FCTRL.FIR.U;

	if(frame.FIR.B.FF == LWCAN_FRAME_STD)
	{
		frame.MsgID = LWCAN_GET_STD_ID;
		for(thisByte = 0; thisByte < frame.FIR.B.DLC; thisByte++)
		{
			frame.data.u8[thisByte] = MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.STD.data[thisByte];
		}
	}
	else
	{
		frame.MsgID = LWCAN_GET_EXT_ID;
		for(thisByte = 0; thisByte < frame.FIR.B.DLC; thisByte++)
		{
			frame.data.u8[thisByte] = MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.data[thisByte];
		}
	}

	xQueueSendFromISR(pCanDriverObj->rxQueue, &frame, &xHigherPriorityTaskWoken);
	MODULE_CAN->CMR.B.RRB = 0x1;
}

void IRAM_ATTR impl_write_frame_phy(const lw_can_frame_t *p_frame) 
{
	uint8_t thisByte;
	MODULE_CAN->MBX_CTRL.FCTRL.FIR.U = p_frame->FIR.U;
	if (p_frame->FIR.B.FF == LWCAN_FRAME_STD) 
	{
		LWCAN_SET_STD_ID(p_frame->MsgID);
		for (thisByte = 0; thisByte < p_frame->FIR.B.DLC; thisByte++)
			MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.STD.data[thisByte] = p_frame->data.u8[thisByte];
	}
	else 
	{
		LWCAN_SET_EXT_ID(p_frame->MsgID);
		for (thisByte = 0; thisByte < p_frame->FIR.B.DLC; thisByte++)
			MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.data[thisByte] = p_frame->data.u8[thisByte];
	}

	MODULE_CAN->CMR.B.TR = 0x1;
}

bool impl_lw_can_start(bool resetCounters)
{
	if (pCanDriverObj && !pCanDriverObj->state.isStarted)
	{
		// Time quantum
		double quanta;

		//enable module
		DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_CAN_CLK_EN);
		DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_CAN_RST);

		//first thing once module is enabled at hardware level is to make sure it is in reset
		MODULE_CAN->MOD.B.RM = 0x1; 

		//configure TX pin
		gpio_set_level(pCanDriverObj->txPin, 1);
		gpio_set_direction(pCanDriverObj->txPin,GPIO_MODE_OUTPUT);
		gpio_matrix_out(pCanDriverObj->txPin,CAN_TX_IDX,0,0);
		gpio_pad_select_gpio(pCanDriverObj->txPin);

		//configure RX pin
		gpio_set_direction(pCanDriverObj->rxPin,GPIO_MODE_INPUT);
		gpio_matrix_in(pCanDriverObj->rxPin,CAN_RX_IDX,0);
		gpio_pad_select_gpio(pCanDriverObj->rxPin);

		//set to PELICAN mode
		MODULE_CAN->CDR.B.CAN_M = 0x1;

		MODULE_CAN->IER.U = 0; //disable all interrupt sources until we're ready
		//clear interrupt flags
		(void)MODULE_CAN->IR.U;
		
		//synchronization jump width is the same for all baud rates
		MODULE_CAN->BTR0.B.SJW = 0x1;

		//TSEG2 is the same for all baud rates
		MODULE_CAN->BTR1.B.TSEG2 = 0x1;

		//select time quantum and set TSEG1
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

		//set baud rate prescaler
		//APB_CLK_FREQ should be 80M
		MODULE_CAN->BTR0.B.BRP = (uint8_t)round((((APB_CLK_FREQ * quanta) / 2) - 1)/1000000)-1;

		/* Set sampling
		* 1 -> triple; the bus is sampled three times; recommended for low/medium speed buses     (class A and B) where
		* filtering spikes on the bus line is beneficial 0 -> single; the bus is sampled once; recommended for high speed
		* buses (SAE class C)*/
		MODULE_CAN->BTR1.B.SAM = 0x1;

		//enable all interrupts (BUT NOT BIT 4 which has turned into a baud rate scalar!)
		MODULE_CAN->IER.U = 0xEF; //1110 1111

		// Set acceptance filter	
		MODULE_CAN->MOD.B.AFM = pCanDriverObj->filter.FM;	
		MODULE_CAN->MBX_CTRL.ACC.CODE[0] = pCanDriverObj->filter.AC.B.R0;
		MODULE_CAN->MBX_CTRL.ACC.CODE[1] = pCanDriverObj->filter.AC.B.R1;
		MODULE_CAN->MBX_CTRL.ACC.CODE[2] = pCanDriverObj->filter.AC.B.R2;
		MODULE_CAN->MBX_CTRL.ACC.CODE[3] = pCanDriverObj->filter.AC.B.R3;
		MODULE_CAN->MBX_CTRL.ACC.MASK[0] = pCanDriverObj->filter.AM.B.R0;
		MODULE_CAN->MBX_CTRL.ACC.MASK[1] = pCanDriverObj->filter.AM.B.R1;
		MODULE_CAN->MBX_CTRL.ACC.MASK[2] = pCanDriverObj->filter.AM.B.R2;
		MODULE_CAN->MBX_CTRL.ACC.MASK[3] = pCanDriverObj->filter.AM.B.R3;

		// Set to normal mode
		MODULE_CAN->OCR.B.OCMODE = pCanDriverObj->ocMode;

		// Clear error counters
		MODULE_CAN->TXERR.U = 0;
		MODULE_CAN->RXERR.U = 0;
		(void) MODULE_CAN->ECC;

		// Clear interrupt flags
		(void) MODULE_CAN->IR.U;

		// Allocate queues.
		pCanDriverObj->rxQueue = xQueueCreate(pCanDriverObj->rxQueueSize, sizeof(lw_can_frame_t));
		pCanDriverObj->txQueue = xQueueCreate(pCanDriverObj->txQueueSize, sizeof(lw_can_frame_t));

		// Install CAN interrupt service.
		esp_intr_alloc(ETS_CAN_INTR_SOURCE, 0, lw_can_interrupt, NULL, &pCanDriverObj->intrHandle);

		// Reset counters.
		if (resetCounters)
			pdo_reset_bus_counters(pCanDriverObj);

		// Set state.
		pCanDriverObj->state.isStarted = true;

		// Showtime. Release Reset Mode.
		MODULE_CAN->MOD.B.RM = 0;

		return true;
	}
	
	return false;
}

bool impl_lw_can_stop()
{
	if (pCanDriverObj && pCanDriverObj->state.isStarted)
	{
		// Reset the module.
		MODULE_CAN->MOD.B.RM = 0x1;
		DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_CAN_CLK_EN);
		DPORT_SET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_CAN_RST);
		MODULE_CAN->IER.U = 0;

		// Remove interrupt and semaphore.
		esp_intr_free(pCanDriverObj->intrHandle);

		// Delete queues.
		vQueueDelete(pCanDriverObj->txQueue);
		vQueueDelete(pCanDriverObj->rxQueue);

		// Clear state flags.
		pCanDriverObj->state.needReset = false;
		pCanDriverObj->state.isStarted = false;

		return true;
	}

	return false;
}

bool impl_lw_can_set_filter(uint32_t messageId)
{
	if (pCanDriverObj != NULL && !pCanDriverObj->state.isStarted)
	{
		pCanDriverObj->filter.FM = LWCAN_FILTER_SINGLE;
		pCanDriverObj->filter.AC.U = __bswap32(messageId);
		pCanDriverObj->filter.AM.U = 0xFFFFFFFF;
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
		pCanDriverObj->wd_hit_cnt = 0;

		// Setup states.
		pCanDriverObj->state.isStarted = false;
		pCanDriverObj->state.needReset = false;
		pCanDriverObj->state.txOccupied = false;

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
		if (pCanDriverObj->state.isStarted)
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
	LWCAN_ENTER_CRITICAL_ISR();
	lw_can_frame_t frame;
	uint32_t interrupt;
	BaseType_t higherPriorityTaskWoken = pdFALSE;
	interrupt = MODULE_CAN->IR.U;

	// Increase interrupt counter.
	++pCanDriverObj->total_int_cnt;

	// Handle counters.
	if (interrupt & LWCAN_IRQ_ARB_LOST)
		++pCanDriverObj->arb_lost_cnt;
	
	if (interrupt & LWCAN_IRQ_DATA_OVERRUN)
		++pCanDriverObj->data_overrun_cnt;

	if (interrupt & LWCAN_IRQ_WAKEUP)
		++pCanDriverObj->wake_up_cnt;

	if (interrupt & LWCAN_IRQ_ERR_PASSIVE)
		++pCanDriverObj->err_passive_cnt;

	if (interrupt & LWCAN_IRQ_BUS_ERR)
		++pCanDriverObj->bus_error_cnt;

	// Handle TX complete interrupt (incl. errata fix)
	if (((interrupt & LWCAN_IRQ_TX) || pCanDriverObj->state.txOccupied) && MODULE_CAN->SR.B.TBS) 
	{
		if (uxQueueMessagesWaitingFromISR(pCanDriverObj->txQueue) > 0)
		{
			xQueueReceiveFromISR(pCanDriverObj->txQueue, &frame, NULL);
			impl_write_frame_phy(&frame);
		}
		else
		{
			pCanDriverObj->state.txOccupied = false;
		}
	}

	// Handle RX frame available interrupt
	if (interrupt & LWCAN_IRQ_RX)
	{
		for (int rxFrames = 0; rxFrames < MODULE_CAN->RMC.B.RMC; rxFrames++)
		{
			impl_lw_read_frame_phy();
		}
	}

	// Handle error interrupts.
	if (interrupt & (LWCAN_IRQ_ERR				//0x4
					| LWCAN_IRQ_DATA_OVERRUN	//0x8
					| LWCAN_IRQ_WAKEUP			//0x10
					| LWCAN_IRQ_ERR_PASSIVE		//0x20
					| LWCAN_IRQ_BUS_ERR			//0x80
	))
	{
		pCanDriverObj->state.needReset = true;
	}
	LWCAN_EXIT_CRITICAL_ISR();

	if (higherPriorityTaskWoken)
		portYIELD_FROM_ISR();
}

void lw_can_watchdog(void* param)
{
	const TickType_t xDelay = 200 / portTICK_PERIOD_MS;
	bool doReset = false;
	while (true)
	{
		vTaskDelay( xDelay );

		LWCAN_ENTER_CRITICAL();
		if (pCanDriverObj && pCanDriverObj->state.isStarted && pCanDriverObj->state.needReset)
		{
			impl_lw_can_stop();
			impl_lw_can_start(false);
			pCanDriverObj->wd_hit_cnt++;
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
	driverStarted = impl_lw_can_start(true);
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
	if (pCanDriverObj && pCanDriverObj->state.isStarted)
	{
		if (pCanDriverObj->state.txOccupied)
		{
			frameQueued = xQueueSend(pCanDriverObj->txQueue, &frame, 0) == pdTRUE;
		}
		else
		{
			pCanDriverObj->state.txOccupied = true;
			impl_write_frame_phy(&frame);
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
	rxQueue = (pCanDriverObj != NULL && pCanDriverObj->state.isStarted) ? pCanDriverObj->rxQueue : NULL;
	LWCAN_EXIT_CRITICAL();
	return rxQueue != NULL && xQueueReceive(rxQueue, &outFrame, 0) == pdTRUE;
}

bool lw_can_set_filter(uint32_t messageId)
{
	bool filterSetStatus;
	LWCAN_ENTER_CRITICAL();
	filterSetStatus = impl_lw_can_set_filter(messageId);
	LWCAN_EXIT_CRITICAL();
	return filterSetStatus;
}

uint32_t lw_can_get_wd_hits()
{
	uint32_t wd_hit_cnt;
	LWCAN_ENTER_CRITICAL();
	wd_hit_cnt = pCanDriverObj != NULL ? pCanDriverObj->wd_hit_cnt : 0;
	LWCAN_EXIT_CRITICAL();
	return wd_hit_cnt;
}

uint32_t lw_can_get_arb_lost_cnt()
{
	uint32_t arb_lost_cnt;
	LWCAN_ENTER_CRITICAL();
	arb_lost_cnt = pCanDriverObj != NULL ? pCanDriverObj->arb_lost_cnt : 0;
	LWCAN_EXIT_CRITICAL();
	return arb_lost_cnt;
}

uint32_t lw_can_get_data_overrun_cnt()
{
	uint32_t data_overrun_cnt;
	LWCAN_ENTER_CRITICAL();
	data_overrun_cnt = pCanDriverObj != NULL ? pCanDriverObj->data_overrun_cnt : 0;
	LWCAN_EXIT_CRITICAL();
	return data_overrun_cnt;
}

uint32_t lw_can_get_wake_up_cnt()
{
	uint32_t wake_up_cnt;
	LWCAN_ENTER_CRITICAL();
	wake_up_cnt = pCanDriverObj != NULL ? pCanDriverObj->wake_up_cnt : 0;
	LWCAN_EXIT_CRITICAL();
	return wake_up_cnt;
}

uint32_t lw_can_get_err_passive_cnt()
{
	uint32_t err_passive_cnt;
	LWCAN_ENTER_CRITICAL();
	err_passive_cnt = pCanDriverObj != NULL ? pCanDriverObj->err_passive_cnt : 0;
	LWCAN_EXIT_CRITICAL();
	return err_passive_cnt;
}

uint32_t lw_can_get_bus_error_cnt()
{
	uint32_t bus_error_cnt;
	LWCAN_ENTER_CRITICAL();
	bus_error_cnt = pCanDriverObj != NULL ? pCanDriverObj->bus_error_cnt : 0;
	LWCAN_EXIT_CRITICAL();
	return bus_error_cnt;
}

uint32_t lw_can_get_total_int_cnt()
{
	uint32_t total_int_cnt;
	LWCAN_ENTER_CRITICAL();
	total_int_cnt = pCanDriverObj != NULL ? pCanDriverObj->total_int_cnt : 0;
	LWCAN_EXIT_CRITICAL();
	return total_int_cnt;
}

uint32_t lw_can_get_msgs_to_rx()
{
	uint32_t msgs_to_rx;
	LWCAN_ENTER_CRITICAL();
	msgs_to_rx = pCanDriverObj != NULL ? uxQueueMessagesWaiting(pCanDriverObj->rxQueue) : 0;
	LWCAN_EXIT_CRITICAL();
	return msgs_to_rx;
}

uint32_t lw_can_get_msgs_to_tx()
{
	uint32_t msgs_to_tx;
	LWCAN_ENTER_CRITICAL();
	msgs_to_tx = pCanDriverObj != NULL ? uxQueueMessagesWaiting(pCanDriverObj->txQueue) : 0;
	LWCAN_EXIT_CRITICAL();
	return msgs_to_tx;
}
//===================================================================================================================
// END PUBLIC API
//===================================================================================================================
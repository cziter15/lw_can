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

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "soc/dport_reg.h"
#include "esp_pm.h"

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
struct lw_can_filter_t
{
	uint32_t id{0};
	uint32_t mask{0};
};

struct lw_can_driver_obj_t
{
	lw_can_filter_t filter;									// Filter settings.

	gpio_num_t txPin;										// CAN TX pin.
	gpio_num_t rxPin;										// CAN RX pin.

	lw_can_bus_timing_t	busTiming;							// CAN bus timing

	uint8_t rxQueueSize;									// CAN RX queue size.
	uint8_t txQueueSize;									// CAN TX queue size.
	QueueHandle_t rxQueue;									// CAN RX queue handle.
	QueueHandle_t txQueue;									// CAN TX queue handle.

	xTaskHandle wdtTask;									// CAN TX task.

	lw_can_frame_t cachedFrame;								// Cached frame to retry.
	lw_can_bus_counters counters;							// Statistics counters.

	intr_handle_t intrHandle;								// CAN interrupt handle.

	uint8_t configFlags;									// CAN config flags.
	uint8_t driverFlags;									// Driver state flags.

	esp_pm_lock_handle_t pmLock;							// PM lock for APB.
};

lw_can_driver_obj_t* pCanDriverObj{nullptr}; 				// Driver object pointer.

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

void IRAM_ATTR ll_lw_can_enable_peripheral_in_reset_mode()
{
	DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_CAN_CLK_EN);
	DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_CAN_RST);
	MODULE_CAN->MOD.U = 1;
	MODULE_CAN->IER.U = 0;
}

void IRAM_ATTR ll_lw_can_disable_peripheral()
{
	MODULE_CAN->MOD.U = 1;
	DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_CAN_CLK_EN);
	DPORT_SET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_CAN_RST);
	MODULE_CAN->IER.U = 0;
}

void IRAM_ATTR ll_lw_can_clear_registers()
{
	MODULE_CAN->BTR0.U = 0;
	MODULE_CAN->BTR1.U = 0;
	MODULE_CAN->CDR.U = 0;
	MODULE_CAN->OCR.U = 0;

	MODULE_CAN->RMC.U = 0;
	MODULE_CAN->SR.U = 0;
	MODULE_CAN->CMR.U = 0;

	MODULE_CAN->TXERR.U = 0;
	MODULE_CAN->RXERR.U = 0;

	(void)MODULE_CAN->ECC;
	(void)MODULE_CAN->IR.U;
}

void ll_lw_can_release_gpio_matrix()
{
	gpio_reset_pin(pCanDriverObj->rxPin);
	gpio_reset_pin(pCanDriverObj->txPin);
}

void ll_lw_can_assign_gpio_matrix()
{
	// Reset pins.
	ll_lw_can_release_gpio_matrix();

	// Configure TX pin
	gpio_set_direction(pCanDriverObj->txPin,GPIO_MODE_OUTPUT);
	gpio_set_pull_mode(pCanDriverObj->txPin, GPIO_FLOATING);
	gpio_matrix_out(pCanDriverObj->txPin, CAN_TX_IDX, 0, 0);
	gpio_pad_select_gpio(pCanDriverObj->txPin);

	// Configure RX pin
	gpio_set_direction(pCanDriverObj->rxPin, GPIO_MODE_INPUT);
	gpio_set_pull_mode(pCanDriverObj->rxPin, GPIO_FLOATING);
	gpio_matrix_in(pCanDriverObj->rxPin, CAN_RX_IDX, 0);
	gpio_pad_select_gpio(pCanDriverObj->rxPin);
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

void IRAM_ATTR ll_lw_can_write_frame_phy(const lw_can_frame_t& frame, bool cacheFrame = true) 
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

	if (cacheFrame && (pCanDriverObj->configFlags & LWCAN_CFG_AUTO_RETRANSMIT))
		pCanDriverObj->cachedFrame = frame;

	MODULE_CAN->CMR.B.TR = 1;
}

void IRAM_ATTR ll_lw_can_rst_from_isr()
{
	// @TODO: Check CMR for abort transmission later. Maybe worth to call before.
	// Save registers.
	uint32_t BTR0 = MODULE_CAN->BTR0.U;
	uint32_t BTR1 = MODULE_CAN->BTR1.U;
	uint32_t CDR = MODULE_CAN->CDR.U;
	uint32_t OCR = MODULE_CAN->OCR.U;
	uint32_t IER = MODULE_CAN->IER.U;

	// Restart with register cleanup. 
	ll_lw_can_disable_peripheral();
	ll_lw_can_enable_peripheral_in_reset_mode();
	ll_lw_can_clear_registers();

	// Restore registers.
	MODULE_CAN->BTR0.U = BTR0;
	MODULE_CAN->BTR1.U = BTR1;
	MODULE_CAN->CDR.U = CDR;
	MODULE_CAN->OCR.U = OCR;
	MODULE_CAN->IER.U = IER;

	// Reset filter reg.
	ll_lw_can_reset_filter_reg();

	// Release reset mode.
	MODULE_CAN->MOD.B.RM = 0;
}

void lw_can_wdt_task(void* arg)
{
	uint32_t shortDelay = pdMS_TO_TICKS(LWCAN_SHORT_RESET_DELAY_MS);
	uint32_t resetDelay = shortDelay;
	uint8_t maxResets = 10;

	while (true)
	{	
		LWCAN_ENTER_CRITICAL();
		if (pCanDriverObj->driverFlags & LWCAN_DS_TX_COOLDOWN)
		{
			// Exit reset mode.
			MODULE_CAN->MOD.B.RM = 0;

			// Requeue the frame if we broke sending.
			if ((pCanDriverObj->configFlags & LWCAN_CFG_AUTO_RETRANSMIT) && (pCanDriverObj->driverFlags & LWCAN_DS_HAS_FRAME_TO_SEND))
			{
				ll_lw_can_write_frame_phy(pCanDriverObj->cachedFrame, false);
#ifdef LWCAN_DEBUG_COUNTERS
				++pCanDriverObj->counters.retransmitCount;
#endif
			}

			// If this is n-th reset, then enlarge delay for reset to long delay.
			if (pCanDriverObj->counters.resetsInARow < maxResets && ++pCanDriverObj->counters.resetsInARow == maxResets)
			{
				xQueueReset(pCanDriverObj->txQueue);
				xQueueReset(pCanDriverObj->rxQueue);
				resetDelay = pdMS_TO_TICKS(LWCAN_LONG_RESET_DELAY_MS);
				pCanDriverObj->driverFlags &= ~LWCAN_DS_HAS_FRAME_TO_SEND;
			}
			
			pCanDriverObj->driverFlags &= ~LWCAN_DS_TX_COOLDOWN;
		}
		else
		{
			// Restore short delay and clear reset counter.
			pCanDriverObj->counters.resetsInARow = 0;
			resetDelay = shortDelay;
		}
		LWCAN_EXIT_CRITICAL();
		
		// Some cooldown.
		vTaskDelay(resetDelay);
	}
}

void IRAM_ATTR ll_lw_can_interrupt()
{
	// Read and clear interrupts.
	uint32_t interrupt {MODULE_CAN->IR.U};

	// Handle counters.
#ifdef LWCAN_DEBUG_COUNTERS
	if (interrupt & LWCAN_IRQ_ARB_LOST)
		++pCanDriverObj->counters.arbLostCnt;

	if (interrupt & LWCAN_IRQ_ERR_PASSIVE)
		++pCanDriverObj->counters.errPassiveCnt;

	if (interrupt & LWCAN_IRQ_BUS_ERR)
		++pCanDriverObj->counters.busErrorCnt;
#endif

	// Read frames from buffer.
	for (unsigned int rxFrames = 0; rxFrames < MODULE_CAN->RMC.B.RMC; ++rxFrames)
	{
		ll_lw_can_read_frame_phy();
	}

	// Handle error interrupts.
	if (interrupt & (LWCAN_IRQ_ERR_PASSIVE | LWCAN_IRQ_BUS_ERR))
	{
		ll_lw_can_rst_from_isr();
		pCanDriverObj->driverFlags |= LWCAN_DS_TX_COOLDOWN;
		return;
	}

	// Skip handling transmission if we are in TX cooldown state.
	if (pCanDriverObj->driverFlags & LWCAN_DS_TX_COOLDOWN)
		return;

	// Handle sending in case there is no error.
	if ((pCanDriverObj->driverFlags & LWCAN_DS_HAS_FRAME_TO_SEND) && MODULE_CAN->SR.B.TBS)
	{
		lw_can_frame_t frame;
		if (xQueueIsQueueEmptyFromISR(pCanDriverObj->txQueue) == pdFALSE)
		{
			xQueueReceiveFromISR(pCanDriverObj->txQueue, &frame, nullptr);
			ll_lw_can_write_frame_phy(frame);
		}
		else 
		{
			pCanDriverObj->driverFlags &= ~LWCAN_DS_HAS_FRAME_TO_SEND;
		}
	}
}

bool ll_lw_can_start()
{
	// If not installed or started, return false.
	if (!pCanDriverObj || (pCanDriverObj->driverFlags & LWCAN_DS_DRIVER_STARTED))
		return false;
	
	if (pCanDriverObj->pmLock)
		esp_pm_lock_acquire(pCanDriverObj->pmLock);

	// Time quanta.
	double quanta;

	// Enable peripheral.
	ll_lw_can_enable_peripheral_in_reset_mode();
	// Assign GPIOs.
	ll_lw_can_assign_gpio_matrix();
	// Reset interrupt and error counters.
	ll_lw_can_clear_registers();

	// Set CAN Mode.
	MODULE_CAN->CDR.B.CAN_M = 1;

	// Set timing.
	MODULE_CAN->BTR0.B.SJW = pCanDriverObj->busTiming.sjw - 1;
	MODULE_CAN->BTR0.B.BRP = (pCanDriverObj->busTiming.prescaler / 2) - 1;
	MODULE_CAN->BTR1.B.TSEG1 = pCanDriverObj->busTiming.tseg1 - 1;
	MODULE_CAN->BTR1.B.TSEG2 = pCanDriverObj->busTiming.tseg2 - 1;
	MODULE_CAN->BTR1.B.SAM = (pCanDriverObj->driverFlags & LWCAN_CFG_TRIPLE_SAMPLING) ? 1 : 0;

	// Set OC mode.
	MODULE_CAN->MOD.B.LOM = (pCanDriverObj->configFlags & LWCAN_CFG_LISTEN_ONLY) ? 1 : 0;
	MODULE_CAN->OCR.B.OCTP0 = 1;
	MODULE_CAN->OCR.B.OCTN0 = 1;

	// Reset filter.
	ll_lw_can_reset_filter_reg();

	// Enable interrupts.
	MODULE_CAN->IER.U = 0xFF;
	MODULE_CAN->IER.B.BRP_DIV = 0;

	// Allocate queues.
	pCanDriverObj->rxQueue = xQueueCreate(pCanDriverObj->rxQueueSize, sizeof(lw_can_frame_t));
	pCanDriverObj->txQueue = xQueueCreate(pCanDriverObj->txQueueSize, sizeof(lw_can_frame_t));

	// Create control task.
	xTaskCreate(lw_can_wdt_task, "canwdt", 2408, nullptr, 2, &pCanDriverObj->wdtTask);

	// Reset counters.
	pCanDriverObj->counters = {};
	
	// Install CAN interrupt service.
	esp_intr_alloc(ETS_CAN_INTR_SOURCE, ESP_INTR_FLAG_IRAM, lw_can_interrupt, nullptr, &pCanDriverObj->intrHandle);

	// Showtime. Release Reset Mode.
	MODULE_CAN->MOD.B.RM = 0;

	// Set driver state.
	pCanDriverObj->driverFlags = LWCAN_DS_DRIVER_STARTED;
	return true;
}

bool ll_lw_can_stop()
{
	// If not installed or not started, return false.
	if (!pCanDriverObj || !(pCanDriverObj->driverFlags & LWCAN_DS_DRIVER_STARTED))
		return false;

	// Turn off modkule.
	ll_lw_can_disable_peripheral();

	// Remove interrupt and semaphore.
	esp_intr_free(pCanDriverObj->intrHandle);

	// Delete queues.
	vQueueDelete(pCanDriverObj->txQueue);
	vQueueDelete(pCanDriverObj->rxQueue);
	vTaskDelete(pCanDriverObj->wdtTask);

	// Clear state flags.
	pCanDriverObj->driverFlags = 0;

	if (pCanDriverObj->pmLock)
		esp_pm_lock_release(pCanDriverObj->pmLock);

	return true;
}

bool ll_lw_can_set_filter(uint32_t id, uint32_t mask)
{
	// If not installed or already started, return false.
	if (!pCanDriverObj || (pCanDriverObj->driverFlags & LWCAN_DS_DRIVER_STARTED))
		return false;

	pCanDriverObj->filter.mask = mask;
	pCanDriverObj->filter.id = id;

	return true;
}

bool ll_lw_can_install(gpio_num_t rxPin, gpio_num_t txPin, const lw_can_bus_timing_t& busTiming, uint8_t rxQueueSize, uint8_t txQueueSize, uint8_t configFlags)
{
	// If already installed, return false.
	if (pCanDriverObj)
		return false;

	pCanDriverObj = new lw_can_driver_obj_t();

	// Setup pins.
	pCanDriverObj->txPin = txPin;
	pCanDriverObj->rxPin = rxPin;
	
	// Set timing.
	pCanDriverObj->busTiming = busTiming;

	// Copy queue sizes.
	pCanDriverObj->rxQueueSize = rxQueueSize;
	pCanDriverObj->txQueueSize = txQueueSize;

	// Setup flags.
	pCanDriverObj->driverFlags = 0;
	pCanDriverObj->configFlags = configFlags;

	// Initialize PM lock for APB frequency.
	pCanDriverObj->pmLock = nullptr;
	esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "candrv", &pCanDriverObj->pmLock);

	return true;
}

bool ll_lw_can_uninstall()
{
	// If not installed, return false.
	if (!pCanDriverObj)
		return false;

	// Stop driver if working.
	if (pCanDriverObj->driverFlags & LWCAN_DS_DRIVER_STARTED)
		ll_lw_can_stop();

	// Release gpio.
	ll_lw_can_release_gpio_matrix();

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
bool lw_can_install(gpio_num_t rxPin, gpio_num_t txPin, const lw_can_bus_timing_t& busTiming, uint8_t rxQueueSize, uint8_t txQueueSize, uint8_t configFlags)
{
	bool driverInstalled;
	LWCAN_ENTER_CRITICAL();
	driverInstalled = ll_lw_can_install(rxPin, txPin, busTiming, rxQueueSize, txQueueSize, configFlags);
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
	if (pCanDriverObj && (pCanDriverObj->driverFlags & LWCAN_DS_DRIVER_STARTED))
	{
		if ((pCanDriverObj->driverFlags & LWCAN_DS_HAS_FRAME_TO_SEND) || (pCanDriverObj->driverFlags & LWCAN_DS_TX_COOLDOWN))
		{
			frameQueued = xQueueSend(pCanDriverObj->txQueue, &frame, 0) == pdTRUE;
		}
		else
		{
			pCanDriverObj->driverFlags |= LWCAN_DS_HAS_FRAME_TO_SEND;
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
	rxQueue = (pCanDriverObj && (pCanDriverObj->driverFlags & LWCAN_DS_DRIVER_STARTED)) ? pCanDriverObj->rxQueue : nullptr;
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

#ifdef LWCAN_DEBUG_COUNTERS
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
#endif
//===================================================================================================================
// END PUBLIC API
//===================================================================================================================

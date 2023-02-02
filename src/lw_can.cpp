#include "lw_can.h"
#include "math.h"
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "soc/dport_reg.h"

// Spinlock.
static portMUX_TYPE builtincan_spinlock = portMUX_INITIALIZER_UNLOCKED;
#define LWCAN_ENTER_CRITICAL()  	portENTER_CRITICAL(&builtincan_spinlock)
#define LWCAN_EXIT_CRITICAL()   	portEXIT_CRITICAL(&builtincan_spinlock)
#define LWCAN_ENTER_CRITICAL_ISR()  portENTER_CRITICAL_ISR(&builtincan_spinlock)
#define LWCAN_EXIT_CRITICAL_ISR()   portEXIT_CRITICAL_ISR(&builtincan_spinlock)

// Can filter struct.
struct lw_can_filter
{
	uint8_t	FM;
	uint8_t ACR0;
	uint8_t ACR1;
	uint8_t ACR2;
	uint8_t ACR3;
	uint8_t AMR0;
	uint8_t AMR1;
	uint8_t AMR2;
	uint8_t AMR3;
};

// Driver object struct.
struct lw_can_driver_obj
{
	lw_can_filter filter;

	gpio_num_t txPin;
	gpio_num_t rxPin;
	uint16_t speedKbps;

	uint8_t rxQueueSize;
	uint8_t txQueueSize;
	uint32_t wdHits;

	QueueHandle_t rxQueue;
	QueueHandle_t txQueue;

	SemaphoreHandle_t txComplete;
	intr_handle_t intrHandle;
	TaskHandle_t wdtHandle;

	bool isStarted;
	bool needReset;
};

// Driver object pointer.
static lw_can_driver_obj* pDriverObj = NULL;

//===================================================================================================================
// Required forward declarations
//===================================================================================================================
void IRAM_ATTR lw_can_interrupt(void *arg_p);
void lw_can_watchdog(void *pvParameters);

//===================================================================================================================
// Spinlock free functions
//===================================================================================================================
void IRAM_ATTR impl_lw_read_frame_phy()
{
	uint8_t thisByte;
	CAN_frame_t frame;
	BaseType_t xHigherPriorityTaskWoken;

	frame.FIR.U = MODULE_CAN->MBX_CTRL.FCTRL.FIR.U;

	if(frame.FIR.B.FF == CAN_frame_std)
	{
		frame.MsgID = _CAN_GET_STD_ID;
		for(thisByte = 0; thisByte < frame.FIR.B.DLC; thisByte++)
		{
			frame.data.u8[thisByte] = MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.STD.data[thisByte];
		}
	}
	else
	{
		frame.MsgID = _CAN_GET_EXT_ID;
		for(thisByte = 0; thisByte < frame.FIR.B.DLC; thisByte++)
		{
			frame.data.u8[thisByte] = MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.data[thisByte];
		}
	}

	xQueueSendFromISR(pDriverObj->rxQueue, &frame, &xHigherPriorityTaskWoken);
	MODULE_CAN->CMR.B.RRB = 1;
}

bool impl_write_frame_phy(const CAN_frame_t *p_frame) 
{
	if (MODULE_CAN->SR.B.TBS)
	{
		uint8_t thisByte;
		MODULE_CAN->MBX_CTRL.FCTRL.FIR.U = p_frame->FIR.U;
		if (p_frame->FIR.B.FF == CAN_frame_std) 
		{
			_CAN_SET_STD_ID(p_frame->MsgID);
			for (thisByte = 0; thisByte < p_frame->FIR.B.DLC; thisByte++)
				MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.STD.data[thisByte] = p_frame->data.u8[thisByte];
		}
		else 
		{
			_CAN_SET_EXT_ID(p_frame->MsgID);
			for (thisByte = 0; thisByte < p_frame->FIR.B.DLC; thisByte++)
				MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.data[thisByte] = p_frame->data.u8[thisByte];
		}

		MODULE_CAN->CMR.B.TR = 1;
		return true;
	}
	return false;
}

bool impl_lw_can_start()
{
	if (pDriverObj && !pDriverObj->isStarted)
	{
		// Time quantum
		double quanta;

		// enable module
		DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_CAN_CLK_EN);
		DPORT_SET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_CAN_RST);
		DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_CAN_RST);

		// configure TX pin
		gpio_set_level(pDriverObj->txPin, 1);
		gpio_set_direction(pDriverObj->txPin, GPIO_MODE_OUTPUT);
		gpio_matrix_out(pDriverObj->txPin, CAN_TX_IDX, 0, 0);
		gpio_pad_select_gpio(pDriverObj->txPin);

		// configure RX pin
		gpio_set_direction(pDriverObj->rxPin, GPIO_MODE_INPUT);
		gpio_matrix_in(pDriverObj->rxPin, CAN_RX_IDX, 0);
		gpio_pad_select_gpio(pDriverObj->rxPin);

		// set to PELICAN mode
		MODULE_CAN->CDR.B.CAN_M = 0x1;

		// synchronization jump width is the same for all baud rates
		MODULE_CAN->BTR0.B.SJW = 0x1;

		// TSEG2 is the same for all baud rates
		MODULE_CAN->BTR1.B.TSEG2 = 0x1;

		// select time quantum and set TSEG1
		switch (pDriverObj->speedKbps) 
		{
			case 1000:
				MODULE_CAN->BTR1.B.TSEG1 = 0x4;
				quanta = 0.125;
			break;

			case 800:
				MODULE_CAN->BTR1.B.TSEG1 = 0x6;
				quanta = 0.125;
			break;

			case 200:
				MODULE_CAN->BTR1.B.TSEG1 = 0xc;
				MODULE_CAN->BTR1.B.TSEG2 = 0x5;
				quanta = 0.25;
			break;

			default:
				MODULE_CAN->BTR1.B.TSEG1 = 0xc;
				quanta = ((float) 1000 / pDriverObj->speedKbps) / 16;
		}

		// set baud rate prescaler
		MODULE_CAN->BTR0.B.BRP = (uint8_t) round((((APB_CLK_FREQ * quanta) / 2) - 1) / 1000000) - 1;

		/* Set sampling
		* 1 -> triple; the bus is sampled three times; recommended for low/medium speed buses     (class A and B) where
		* filtering spikes on the bus line is beneficial 0 -> single; the bus is sampled once; recommended for high speed
		* buses (SAE class C)*/
		MODULE_CAN->BTR1.B.SAM = 0x1;

		// enable all interrupts
		MODULE_CAN->IER.U = 0xff;

		// Set acceptance filter	
		MODULE_CAN->MOD.B.AFM = pDriverObj->filter.FM;	
		MODULE_CAN->MBX_CTRL.ACC.CODE[0] = pDriverObj->filter.ACR0;
		MODULE_CAN->MBX_CTRL.ACC.CODE[1] = pDriverObj->filter.ACR1;
		MODULE_CAN->MBX_CTRL.ACC.CODE[2] = pDriverObj->filter.ACR2;
		MODULE_CAN->MBX_CTRL.ACC.CODE[3] = pDriverObj->filter.ACR3;
		MODULE_CAN->MBX_CTRL.ACC.MASK[0] = pDriverObj->filter.AMR0;
		MODULE_CAN->MBX_CTRL.ACC.MASK[1] = pDriverObj->filter.AMR1;
		MODULE_CAN->MBX_CTRL.ACC.MASK[2] = pDriverObj->filter.AMR2;
		MODULE_CAN->MBX_CTRL.ACC.MASK[3] = pDriverObj->filter.AMR3;

		// set to normal mode
		MODULE_CAN->OCR.B.OCMODE = __CAN_OC_NOM;

		// clear error counters
		MODULE_CAN->TXERR.U = 0;
		MODULE_CAN->RXERR.U = 0;
		(void) MODULE_CAN->ECC;

		// clear interrupt flags
		(void) MODULE_CAN->IR.U;

		// allocate the tx complete semaphore
		pDriverObj->txComplete = xSemaphoreCreateBinary();

		// allocate queues
		pDriverObj->rxQueue = xQueueCreate(pDriverObj->rxQueueSize, sizeof(CAN_frame_t));
		pDriverObj->txQueue = xQueueCreate(pDriverObj->txQueueSize, sizeof(CAN_frame_t));

		// install CAN ISR
		esp_intr_alloc(ETS_CAN_INTR_SOURCE, 0, lw_can_interrupt, NULL, &pDriverObj->intrHandle);

		// Showtime. Release Reset Mode.
		MODULE_CAN->MOD.B.RM = 0;

		pDriverObj->isStarted = true;

		return true;
	}
	
	return false;
}

bool impl_lw_can_stop()
{
	if (pDriverObj && pDriverObj->isStarted)
	{
		esp_intr_free(pDriverObj->intrHandle);
		vSemaphoreDelete(pDriverObj->txComplete);
		pDriverObj->intrHandle = NULL;
		pDriverObj->needReset = false;
		pDriverObj->isStarted = false;
		MODULE_CAN->MOD.B.RM = 0;
		return true;
	}

	return false;
}

bool impl_lw_can_set_filter(uint32_t messageId)
{
	union FilterData
	{
		uint32_t u32;
		uint8_t u8[4];
	};
	
	FilterData acceptance_code;
	acceptance_code.u32 = messageId << 21;

	FilterData acceptance_mask;
	acceptance_mask.u32 = ~(0x7FFU << 21);

	if (pDriverObj != NULL)
	{
		pDriverObj->filter.FM = 0;
		pDriverObj->filter.ACR0 = acceptance_code.u8[3];
		pDriverObj->filter.ACR1 = acceptance_code.u8[2];
		pDriverObj->filter.ACR2 = acceptance_code.u8[1];
		pDriverObj->filter.ACR3 = acceptance_code.u8[0];
		pDriverObj->filter.AMR0 = acceptance_mask.u8[3];
		pDriverObj->filter.AMR1 = acceptance_mask.u8[2];
		pDriverObj->filter.AMR2 = acceptance_mask.u8[1];
		pDriverObj->filter.AMR3 = acceptance_mask.u8[0];
		return true;
	}
	return false;
}

bool impl_lw_can_install(gpio_num_t rxPin, gpio_num_t txPin, uint16_t speedKbps, uint8_t rxQueueSize, uint8_t txQueueSize)
{
	if (pDriverObj == NULL)
	{
		pDriverObj = new lw_can_driver_obj();

		// Setup pins.
		pDriverObj->txPin = txPin;
		pDriverObj->rxPin = rxPin;

		// Setup speed.
		pDriverObj->speedKbps = speedKbps;

		// Copy queue sizes.
		pDriverObj->rxQueueSize = rxQueueSize;
		pDriverObj->txQueueSize = txQueueSize;
	
		pDriverObj->rxQueue = NULL;
		pDriverObj->txQueue = NULL;
		pDriverObj->txComplete = NULL;
		pDriverObj->intrHandle = NULL;
		pDriverObj->wdtHandle = NULL;
		pDriverObj->wdHits = 0;

		// Setup states.
		pDriverObj->isStarted = false;
		pDriverObj->needReset = false;

		// Define the CAN filter.
		pDriverObj->filter.FM = 0;
		pDriverObj->filter.ACR0 = 0;
		pDriverObj->filter.ACR1 = 0;
		pDriverObj->filter.ACR2 = 0;
		pDriverObj->filter.ACR3 = 0;
		pDriverObj->filter.AMR0 = 0xFF;
		pDriverObj->filter.AMR1 = 0xFF;
		pDriverObj->filter.AMR2 = 0xFF;
		pDriverObj->filter.AMR3 = 0xFF;

		xTaskCreatePinnedToCore(&lw_can_watchdog, "lw_can_wdt", 2048, NULL, 10, &pDriverObj->wdtHandle, 1);
		return true;
	}

	return false;
}

bool impl_lw_can_uninstall()
{
	if (pDriverObj != NULL)
	{
		if (pDriverObj->isStarted)
			impl_lw_can_stop();

		vTaskDelete(pDriverObj->wdtHandle);
		delete pDriverObj;
		pDriverObj = NULL;
		return true;
	}
	return false;
}
//===================================================================================================================
// Remember to use spinlock properly.
//===================================================================================================================
void IRAM_ATTR lw_can_interrupt(void *arg_p)
{
	LWCAN_ENTER_CRITICAL_ISR();

	uint32_t interrupt;
	BaseType_t higherPriorityTaskWoken = pdFALSE;
	interrupt = MODULE_CAN->IR.U;

	// Handle RX frame available interrupt
	if ((interrupt & __CAN_IRQ_RX) != 0)
	{
		for (int rxFrames = 0; rxFrames < MODULE_CAN->RMC.B.RMC; rxFrames++)
		{
			impl_lw_read_frame_phy();
		}
	}

	// Handle TX complete interrupt
	if ((interrupt & __CAN_IRQ_TX) != 0)
		xSemaphoreGiveFromISR(pDriverObj->txComplete, &higherPriorityTaskWoken);

	// Handle error interrupts.
	if ((interrupt & (__CAN_IRQ_ERR						//0x4
					| __CAN_IRQ_DATA_OVERRUN			//0x8
					| __CAN_IRQ_WAKEUP				//0x10
					| __CAN_IRQ_ERR_PASSIVE			//0x20
					| __CAN_IRQ_BUS_ERR				//0x80
					)) != 0) 
	{
		pDriverObj->needReset = true;
	}

	LWCAN_EXIT_CRITICAL_ISR();

	if (higherPriorityTaskWoken)
		portYIELD_FROM_ISR();
}

void lw_can_watchdog(void *pvParameters)
{
	const TickType_t xDelay = 200 / portTICK_PERIOD_MS;
	bool doReset = false;
	while (true)
	{
		vTaskDelay( xDelay );

		LWCAN_ENTER_CRITICAL();
		if (pDriverObj && pDriverObj->isStarted && pDriverObj->needReset)
		{
			impl_lw_can_stop();
			impl_lw_can_start();
			pDriverObj->wdHits++;
		}
		LWCAN_EXIT_CRITICAL();
	}
}
 
//===================================================================================================================
// PUBLIC API
// Remember to use spinlock properly.
//===================================================================================================================
bool lw_can_install(gpio_num_t rxPin, gpio_num_t txPin, uint16_t speedKbps, uint8_t rxQueueSize, uint8_t txQueueSize)
{
	bool driverInstalled = false;
	LWCAN_ENTER_CRITICAL();
	driverInstalled = impl_lw_can_install(rxPin, txPin, speedKbps, rxQueueSize, txQueueSize);
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
	bool driverStopped = false;
	LWCAN_ENTER_CRITICAL();
	driverStopped = impl_lw_can_stop();
	LWCAN_EXIT_CRITICAL();
	return driverStopped;
}

bool lw_can_transmit(const CAN_frame_t& frame)
{
	bool frameSent;
	SemaphoreHandle_t txSemaphore;
	LWCAN_ENTER_CRITICAL();
	txSemaphore = pDriverObj->txComplete;
	frameSent = impl_write_frame_phy(&frame);
	LWCAN_EXIT_CRITICAL();

	if (frameSent)
		xSemaphoreTake(txSemaphore, 100);

	return frameSent;
}

bool lw_can_read_next_frame(CAN_frame_t& outFrame)
{
	bool frameReadStatus;
	QueueHandle_t rxQueue = NULL;
	LWCAN_ENTER_CRITICAL();
	if (pDriverObj != NULL)
		rxQueue = pDriverObj->rxQueue;
	LWCAN_EXIT_CRITICAL();
	if (rxQueue == NULL)
		return false;
	return xQueueReceive(rxQueue, &outFrame, 0) == pdTRUE;
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
	uint32_t wdHits = 0;
	LWCAN_ENTER_CRITICAL();
	if (pDriverObj)
		wdHits = pDriverObj->wdHits;
	LWCAN_EXIT_CRITICAL();
	return wdHits;
}
//===================================================================================================================
// END PUBLIC API
//===================================================================================================================
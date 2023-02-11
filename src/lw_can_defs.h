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

#include <stdint.h>
#include "driver/gpio.h"

typedef enum 
{
	LWCAN_FRAME_STD = 0, 							// Standard frame, using 11 bit identifer. 
	LWCAN_FRAME_EXT= 1 								// Extended frame, using 29 bit identifer. 
} lw_can_frame_format_t;

typedef enum 
{
	LWCAN_NO_RTR = 0, 								// No RTR frame. 
	LWCAN_RTR 	 = 1 								// RTR frame. 
} lw_can_rtr_t;

typedef union
{
	uint32_t U;										// Unsigned access 
	 struct 
	 {
		uint8_t 				DLC:4;				// [3:0] DLC, Data length container 
		unsigned int 			unknown_2:2;		// unknown 
		lw_can_rtr_t 			RTR:1;				// [6:6] RTR, Remote Transmission Request 
		lw_can_frame_format_t 	FF:1;				// [7:7] Frame Format, see# lw_can_frame_format_t
		unsigned int 			reserved_24:24;		// Reserved 
	} B;
} CAN_FIR_t;


typedef struct 
{
	CAN_FIR_t	FIR;								// Frame information record
	uint32_t 	MsgID;								// Message ID 
	union 
	{
		uint8_t u8[8];								// Payload byte access
		uint32_t u32[2];							// Payload u32 access
	} data;
} lw_can_frame_t;

typedef enum 
{
	LWCAN_FILTER_DUAL = 0, 										// The dual acceptance filter option is enabled (two filters, each with the length of 16 bit are active) 
	LWCAN_FILTER_SINGLE = 1 										// The single acceptance filter option is enabled (one filter with the length of 32 bit is active) 
} lw_can_filter_mode_t;

typedef struct 
{
	lw_can_filter_mode_t 	FM:1;					// [0:0] Filter Mode 
	uint8_t 				ACR0;					// Acceptance Code Register ACR0 
	uint8_t 				ACR1;					// Acceptance Code Register ACR1 
	uint8_t 				ACR2;					// Acceptance Code Register ACR2 
	uint8_t 				ACR3;					// Acceptance Code Register ACR3 
	uint8_t 				AMR0;					// Acceptance Mask Register AMR0 
	uint8_t 				AMR1;					// Acceptance Mask Register AMR1 
	uint8_t 				AMR2;					// Acceptance Mask Register AMR2 
	uint8_t 				AMR3;					// Acceptance Mask Register AMR3 
} CAN_filter_t;


#define MODULE_CAN									((volatile CAN_Module_t    *)0x3ff6b000)

#define LWCAN_GET_STD_ID							(((uint32_t)MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.STD.ID[0] << 3) | \
													(MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.STD.ID[1] >> 5))

#define LWCAN_GET_EXT_ID							(((uint32_t)MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.ID[0] << 21) | \
													(MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.ID[1] << 13) | \
													(MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.ID[2] << 5) | \
													(MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.ID[3] >> 3 ))

#define LWCAN_SET_STD_ID(x)							MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.STD.ID[0] = ((x) >> 3);	\
													MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.STD.ID[1] = ((x) << 5);

#define LWCAN_SET_EXT_ID(x)							MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.ID[0] = ((x) >> 21);	\
													MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.ID[1] = ((x) >> 13);	\
													MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.ID[2] = ((x) >> 5);	\
													MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.ID[3] = ((x) << 3);	\

typedef enum  
{
	LWCAN_IRQ_RX =				BIT(0),				// RX Interrupt 
	LWCAN_IRQ_TX =				BIT(1),				// TX Interrupt 
	LWCAN_IRQ_ERR =				BIT(2),				// Error Interrupt 
	LWCAN_IRQ_DATA_OVERRUN =	BIT(3),				// Date Overrun Interrupt 
	LWCAN_IRQ_WAKEUP =			BIT(4),				// Wakeup Interrupt 
	LWCAN_IRQ_ERR_PASSIVE =		BIT(5),				// Passive Error Interrupt 
	LWCAN_IRQ_ARB_LOST =		BIT(6),				// Arbitration lost interrupt 
	LWCAN_IRQ_BUS_ERR =			BIT(7),				// Bus error Interrupt 
} lw_can_irq_t;

typedef enum  
{
	LWCAN_OC_BOM =				0b00,				// bi-phase output mode 
	LWCAN_OC_TOM =				0b01,				// test output mode 
	LWCAN_OC_NOM =				0b10,				// normal output mode 
	LWCAN_OC_COM =				0b11,				// clock output mode 
} lw_can_ocmode_t;

typedef struct 
{
	union
	{
		uint32_t U;									// Unsigned access 
		struct 
		{
			unsigned int RM:1;						// MOD.0 Reset Mode 
			unsigned int LOM:1;						// MOD.1 Listen Only Mode 
			unsigned int STM:1;						// MOD.2 Self Test Mode 
			unsigned int AFM:1;						// MOD.3 Acceptance Filter Mode 
			unsigned int SM:1;						// MOD.4 Sleep Mode 
			unsigned int reserved_27:27;			// Reserved 
		} B;
	} MOD;
	union
	{
		uint32_t U;									// Unsigned access 
		struct 
		{
			unsigned int TR:1;						// CMR.0 Transmission Request 
			unsigned int AT:1;						// CMR.1 Abort Transmission 
			unsigned int RRB:1;						// CMR.2 Release Receive Buffer 
			unsigned int CDO:1;						// CMR.3 Clear Data Overrun 
			unsigned int GTS:1;						// CMR.4 Go To Sleep 
			unsigned int reserved_27:27;			// Reserved 
		} B;
	} CMR;
	union
	{
		uint32_t U;									// Unsigned access 
		struct 
		{
			unsigned int RBS:1;						// SR.0 Receive Buffer Status 
			unsigned int DOS:1;						// SR.1 Data Overrun Status 
			unsigned int TBS:1;						// SR.2 Transmit Buffer Status 
			unsigned int TCS:1;						// SR.3 Transmission Complete Status 
			unsigned int RS:1;						// SR.4 Receive Status 
			unsigned int TS:1;						// SR.5 Transmit Status 
			unsigned int ES:1;						// SR.6 Error Status 
			unsigned int BS:1;						// SR.7 Bus Status 
			unsigned int reserved_24:24;			// Reserved 
		} B;
	} SR;
	union
	{
		uint32_t U;									// Unsigned access 
		struct 
		{
			unsigned int RI:1;						// IR.0 Receive Interrupt 
			unsigned int TI:1;						// IR.1 Transmit Interrupt 
			unsigned int EI:1;						// IR.2 Error Interrupt 
			unsigned int DOI:1;						// IR.3 Data Overrun Interrupt 
			unsigned int RES_2:1;					//  Reserved 
			unsigned int EPI:1;						// IR.5 Error Passive Interrupt 
			unsigned int ALI:1;						// IR.6 Arbitration Lost Interrupt 
			unsigned int BEI:1;						// IR.7 Bus Error Interrupt 
			unsigned int reserved_24:24;			// Reserved 
		} B;
	} IR;
	union
	{
		uint32_t U;									// Unsigned access 
		struct 
		{
			unsigned int RIE:1;						// IER.0 Receive Interrupt Enable 
			unsigned int TIE:1;						// IER.1 Transmit Interrupt Enable 
			unsigned int EIE:1;						// IER.2 Error Interrupt Enable 
			unsigned int DOIE:1;					// IER.3 Data Overrun Interrupt Enable 
			unsigned int BRP_DIV:1;					// 1 = Divide baud rate by 2 but only on rev2 and higher ESP32 
			unsigned int EPIE:1;					// IER.5 Error Passive Interrupt Enable 
			unsigned int ALIE:1;					// IER.6 Arbitration Lost Interrupt Enable 
			unsigned int BEIE:1;					// IER.7 Bus Error Interrupt Enable 
			unsigned int reserved_24:24;			// Reserved  
		} B;
	} IER;
	uint32_t RESERVED0;
	union
	{
		uint32_t U;									// Unsigned access 
		struct 
		{
			unsigned int BRP:6;						// BTR0[5:0] Baud Rate Prescaler 
			unsigned int SJW:2;						// BTR0[7:6] Synchronization Jump Width
			unsigned int reserved_24:24;			// Reserved  
		} B;
	} BTR0;
	union
	{
		uint32_t U;									// Unsigned access 
		struct 
		{
			unsigned int TSEG1:4;					// BTR1[3:0] Timing Segment 1 
			unsigned int TSEG2:3;					// BTR1[6:4] Timing Segment 2
			unsigned int SAM:1;						// BTR1.7 Sampling
			unsigned int reserved_24:24;			// Reserved  
		} B;
	} BTR1;
	union{
		uint32_t U;									// Unsigned access 
		struct 
		{
			unsigned int OCMODE:2;					// OCR[1:0] Output Control Mode, see # 
			unsigned int OCPOL0:1;					// OCR.2 Output Control Polarity 0 
			unsigned int OCTN0:1;					// OCR.3 Output Control Transistor N0 
			unsigned int OCTP0:1;					// OCR.4 Output Control Transistor P0 
			unsigned int OCPOL1:1;					// OCR.5 Output Control Polarity 1 
			unsigned int OCTN1:1;					// OCR.6 Output Control Transistor N1 
			unsigned int OCTP1:1;					// OCR.7 Output Control Transistor P1 
			unsigned int reserved_24:24;			// Reserved  
		} B;
	} OCR;
	uint32_t RESERVED1[2];
	union{
		uint32_t U;									// Unsigned access 
		struct 
		{
			unsigned int ALC:8; 					// ALC[7:0] Arbitration Lost Capture 
			unsigned int reserved_24:24;			// Reserved  
		} B;
	} ALC;
	union
	{
		uint32_t U;									// Unsigned access 
		struct
		{
			unsigned int ECC:8; 					// ECC[7:0] Error Code Capture 
			unsigned int reserved_24:24;			// Reserved  
		} B;
	} ECC;
	union
	{
		uint32_t U;									// Unsigned access 
		struct 
		{
			unsigned int EWLR:8; 					// EWLR[7:0] Error Warning Limit 
			unsigned int reserved_24:24;			// Reserved  
		} B;
	} EWLR;
	union
	{
		uint32_t U;									// Unsigned access 
		struct 
		{
			unsigned int RXERR:8; 					// RXERR[7:0] Receive Error Counter 
			unsigned int reserved_24:24;			// Reserved  
		} B;
	} RXERR;
	union
	{
		uint32_t U;									// Unsigned access 
		struct
		{
			unsigned int TXERR:8;					// TXERR[7:0] Transmit Error Counter 
			unsigned int reserved_24:24;			// Reserved  
		} B;
	} TXERR;

	union 
	{
		struct 
		{
			uint32_t CODE[4];						// Acceptance Message ID 
			uint32_t MASK[4];						// Acceptance Mask 
			uint32_t RESERVED2[5];
		} ACC;										// Acceptance filtering 
		struct 
		{
			CAN_FIR_t	FIR;						// Frame information record 
			union
			{
				struct 
				{
					uint32_t ID[2];					// Standard frame message-ID
					uint32_t data[8];				// Standard frame payload 
					uint32_t reserved[2];
				} STD;								// Standard frame format 
				struct 
				{
					uint32_t ID[4];					// Extended frame message-ID
					uint32_t data[8];				// Extended frame payload 
				} EXT;								// Extended frame format 
			} TX_RX;								// RX/TX interface 
		} FCTRL;									// Function control regs 
	} MBX_CTRL;										// Mailbox control 
	union
	{
		uint32_t U;									// Unsigned access 
		struct 
		{
			unsigned int RMC:8; 					// RMC[7:0] RX Message Counter 
			unsigned int reserved_24:24;			// Reserved Enable 
		} B;
	} RMC;
	union
	{
		uint32_t U;									// Unsigned access 
		struct 
		{
			unsigned int RBSA:8;					// RBSA[7:0] RX Buffer Start Address  
			unsigned int reserved_24:24;			// Reserved Enable 
		} B;
	} RBSA;
	union
	{
		uint32_t U;									// Unsigned access 
		struct 
		{
			unsigned int COD:3;						// CDR[2:0] CLKOUT frequency selector based of fOSC
			unsigned int COFF:1;					// CDR.3 CLKOUT off
			unsigned int reserved_1:1;				// Reserved 
			unsigned int RXINTEN:1;					// CDR.5 This bit allows the TX1 output to be used as a dedicated receive interrupt output
			unsigned int CBP:1;						// CDR.6 allows to bypass the CAN input comparator and is only possible in reset mode.
			unsigned int CAN_M:1;					// CDR.7 If CDR.7 is at logic 0 the CAN controller operates in BasicCAN mode. If set to logic 1 the CAN controller operates in PeliCAN mode. Write access is only possible in reset mode
			unsigned int reserved_24:24;			// Reserved  
		} B;
	} CDR;
	uint32_t IRAM[2];
} CAN_Module_t;

typedef struct
{
	uint32_t arbLostCnt;							// Arbitration lost counter.
	uint32_t dataOverrunCnt;						// Data overrun counter.
	uint32_t wakeUpCnt;								// Wake up counter.
	uint32_t errPassiveCnt;							// Error passive counter.
	uint32_t busErrorCnt;							// Bus error counter.
	uint32_t errataResendFrameCnt;					// RXFrame errata error counter.
} lw_can_bus_counters;
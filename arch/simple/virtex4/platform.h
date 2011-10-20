/*
 * Copyright (c) 2008 Xilinx, Inc.  All rights reserved.
 *
 * Xilinx, Inc.
 * XILINX IS PROVIDING THIS DESIGN, CODE, OR INFORMATION "AS IS" AS A
 * COURTESY TO YOU.  BY PROVIDING THIS DESIGN, CODE, OR INFORMATION AS
 * ONE POSSIBLE   IMPLEMENTATION OF THIS FEATURE, APPLICATION OR
 * STANDARD, XILINX IS MAKING NO REPRESENTATION THAT THIS IMPLEMENTATION
 * IS FREE FROM ANY CLAIMS OF INFRINGEMENT, AND YOU ARE RESPONSIBLE
 * FOR OBTAINING ANY RIGHTS YOU MAY REQUIRE FOR YOUR IMPLEMENTATION.
 * XILINX EXPRESSLY DISCLAIMS ANY WARRANTY WHATSOEVER WITH RESPECT TO
 * THE ADEQUACY OF THE IMPLEMENTATION, INCLUDING BUT NOT LIMITED TO
 * ANY WARRANTIES OR REPRESENTATIONS THAT THIS IMPLEMENTATION IS FREE
 * FROM CLAIMS OF INFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#ifndef __PLATFORM_H_
#define __PLATFORM_H_

#include "../../../inc/xparam.h"

extern void _it_handler();

/* Bit definitions for the bits of the MER register */

#define XIN_INT_MASTER_ENABLE_MASK      0x1UL
#define XIN_INT_HARDWARE_ENABLE_MASK    0x2UL	/* once set cannot be cleared */

#define XTC_TCSR_OFFSET		0	/**< Control/Status register */
#define XTC_TLR_OFFSET		4	/**< Load register */
#define XTC_TCR_OFFSET		8	/**< Timer counter register */

/* Each timer counter consumes 16 bytes of address space */

#define XTC_TIMER_COUNTER_OFFSET	16

#define XTC_CSR_ENABLE_ALL_MASK		0x00000400 /**< Enables all timer
							counters */
#define XTC_CSR_ENABLE_PWM_MASK		0x00000200 /**< Enables the Pulse Width
							Modulation */
#define XTC_CSR_INT_OCCURED_MASK	0x00000100 /**< If bit is set, an
							interrupt has occured.
							If set and '1' is
							written to this bit
							position, bit is
							cleared. */
#define XTC_CSR_ENABLE_TMR_MASK		0x00000080 /**< Enables only the
							specific timer */
#define XTC_CSR_ENABLE_INT_MASK		0x00000040 /**< Enables the interrupt
							output. */
#define XTC_CSR_LOAD_MASK		0x00000020 /**< Loads the timer using
							the load value provided
							earlier in the Load
							Register,
							XTC_TLR_OFFSET. */
#define XTC_CSR_AUTO_RELOAD_MASK	0x00000010 /**< In compare mode,
							configures
							the timer counter to
							reload  from the
							Load Register. The
							default  mode
							causes the timer counter
							to hold when the compare
							value is hit. In capture
							mode, configures  the
							timer counter to not
							hold the previous
							capture value if a new
							event occurs. The
							default mode cause the
							timer counter to hold
							the capture value until
							recognized. */
#define XTC_CSR_EXT_CAPTURE_MASK	0x00000008 /**< Enables the
							external input
							to the timer counter. */
#define XTC_CSR_EXT_GENERATE_MASK	0x00000004 /**< Enables the
							external generate output
							for the timer. */
#define XTC_CSR_DOWN_COUNT_MASK		0x00000002 /**< Configures the timer
							counter to count down
							from start value, the
							default is to count
							up.*/
#define XTC_CSR_CAPTURE_MODE_MASK	0x00000001 /**< Enables the timer to
							capture the timer
							counter value when the
							external capture line is
							asserted. The default
							mode is compare mode.*/


typedef void (*mtk_XTmrCtr_Handler) (void *CallBackRef, uint8_t TmrCtrNumber);
typedef void (*mtk_XInterruptHandler) (void *InstancePtr);

typedef struct mtk_XTmrCtrStats mtk_XTmrCtrStats;
typedef struct mtk_XTmrCtr_Config mtk_XTmrCtr_Config;
typedef struct mtk_XTmrCtr mtk_XTmrCtr;
typedef struct mtk_XIntc_VectorTableEntry mtk_XIntc_VectorTableEntry;
typedef struct mtk_XIntc_Config mtk_XIntc_Config;
typedef struct mtk_XIntc mtk_XIntc;


 struct mtk_XTmrCtrStats{
	uint32_t Interrupts;	 /**< The number of interrupts that have occurred */
};


 struct mtk_XTmrCtr_Config{
	uint16_t DeviceId;	/**< Unique ID  of device */
	uint32_t BaseAddress;/**< Register base address */
};

struct  mtk_XTmrCtr{
	mtk_XTmrCtrStats Stats;	 /**< Component Statistics */
	uint32_t BaseAddress;	 /**< Base address of registers */
	uint32_t IsReady;		 /**< Device is initialized and ready */
	mtk_XTmrCtr_Handler Handler; /**< Callback function */
	void *CallBackRef;	 	/**< Callback reference for handler */
};

 struct mtk_XIntc_VectorTableEntry {
	mtk_XInterruptHandler Handler;
	void *CallBackRef;
};

 struct  mtk_XIntc_Config{
	uint16_t DeviceId;		/**< Unique ID  of device */
	uint32_t BaseAddress;	/**< Register base address */
	uint32_t AckBeforeService;	/**< Ack location per interrupt */
	uint32_t Options;		/**< Device options */
	/** Static vector table of interrupt handlers */
	mtk_XIntc_VectorTableEntry HandlerTable[XPAR_INTC_MAX_NUM_INTR_INPUTS];
};

 struct mtk_XIntc{
	uint32_t BaseAddress;	 /**< Base address of registers */
	uint32_t IsReady;		 /**< Device is initialized and ready */
	uint32_t IsStarted;		 /**< Device has been started */
	uint32_t UnhandledInterrupts; /**< Intc Statistics */
	mtk_XIntc_Config *CfgPtr;	 /**< Pointer to instance config entry */
} ;


mtk_XTmrCtr_Config mtk_XTmrCtr_ConfigTable[] =
{
	{
		XPAR_XPS_TIMER_1_DEVICE_ID,
		XPAR_XPS_TIMER_1_BASEADDR
	}
};


#define XPS_TIMER_DEVICE_ID			0
#define TmrCtrNumber				0	// 1 timer per device TODO : Check this value

uint8_t mtk_XTmrCtr_Offsets[] = { 0, XTC_TIMER_COUNTER_OFFSET };

#define W_Out32(Addr, Value) \
	(*(volatile uint32_t *)((Addr)) = (Value))

#define WriteReg(BaseAddress, TmrCtrNumber, RegOffset, ValueToWrite)\
	W_Out32(((BaseAddress) + mtk_XTmrCtr_Offsets[(TmrCtrNumber)] +	\
			   (RegOffset)), (ValueToWrite))

#define ReadReg(BaseAddress, TmrCtrNumber, RegOffset,ValueRead)\
	R_In32(((BaseAddress) + mtk_XTmrCtr_Offsets[(TmrCtrNumber)] +	\
			   (RegOffset)),(ValueRead))

#define R_In32(Addr, Value) \
	 Value=*(uint32_t *)Addr
#endif


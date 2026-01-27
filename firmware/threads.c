#include <stdint.h>
#include <stdio.h>
#include <cmsis_os2.h>
#include <MKL25Z4.h>
#include "misc.h"

#include "LEDs.h"
#include "delay.h"

#include "LCD.h"
#include "touchscreen.h"
#include "LCD_driver.h"
#include "ST7789.h"

#include "font.h"
#include "threads.h"
#include "GPIO_defs.h"
#include "debug.h"
#include "control.h"
#include "UI.h"
#include "config.h"

#include "ADC.h"
#include "MMA8451.h"
#include "fault.h"

#if SCOPE_SYNC_WITH_RTOS
// External event flags for RTOS approach
extern osEventFlagsId_t scope_event_flags;
extern const uint32_t SCOPE_BUFFERS_FULL_FLAG;
#else
// External state variable for state machine approach
extern volatile SCOPE_STATE_E g_scope_state;
#endif

void Thread_Read_Touchscreen(void * arg); 
void Thread_Draw_Waveforms(void * arg);
void Thread_Draw_UI_Controls(void * arg);
osThreadId_t t_DW, t_DUC;
const osThreadAttr_t Draw_Waveforms_attr = {
	.priority = osPriorityAboveNormal, 
	.stack_size = 256
};
const osThreadAttr_t Update_UI_Controls_attr = {
	.priority = osPriorityNormal, 
	.stack_size = 512 
};

void Thread_Update_Setpoint(void * arg);
void Thread_Read_Accelerometer(void * arg); 

osThreadId_t t_Read_TS, t_USP, t_Read_Accelerometer;

const osThreadAttr_t Read_Touchscreen_attr = {
  .priority = osPriorityNormal,
	.stack_size = 384
};

const osThreadAttr_t Update_Setpoint_attr = {
  .priority = osPriorityHigh,
	.stack_size = 128
};
const osThreadAttr_t Read_Accelerometer_attr = {
  .priority = osPriorityAboveNormal,      
	.stack_size = READ_ACCEL_STK_SZ
};

void Create_OS_Objects(void) {
	LCD_Create_OS_Objects();
	
	t_Read_TS = osThreadNew(Thread_Read_Touchscreen, NULL, &Read_Touchscreen_attr);  
	t_DW = osThreadNew(Thread_Draw_Waveforms, NULL, &Draw_Waveforms_attr);
	t_DUC = osThreadNew(Thread_Draw_UI_Controls, NULL, &Update_UI_Controls_attr);
	t_USP = osThreadNew(Thread_Update_Setpoint, NULL, &Update_Setpoint_attr);
	t_Read_Accelerometer = osThreadNew(Thread_Read_Accelerometer, NULL, &Read_Accelerometer_attr);
}

void Thread_Read_Touchscreen(void * arg) {
	PT_T p;
	uint32_t tick;
 
	tick = osKernelGetTickCount();
	while (1) {
		DEBUG_START(DBG_TREADTS_POS);
		if (LCD_TS_Read(&p)) { 
			UI_Process_Touch(&p);
		}
		DEBUG_STOP(DBG_TREADTS_POS);
		tick += THREAD_READ_TS_PERIOD_TICKS;
		osDelayUntil(tick);
	}
}

void Thread_Draw_Waveforms(void * arg) {
	uint32_t tick;
	
	// Initialization
	osMutexAcquire(LCD_mutex, osWaitForever);
	UI_Draw_Waveforms();
	osMutexRelease(LCD_mutex);
	tick = osKernelGetTickCount();
	
	// Regular operation
	while (1) {
		tick += THREAD_DRAW_WAVEFORM_PERIOD_TICKS;
		osDelayUntil(tick);
		DEBUG_START(DBG_T_DRAW_WVFMS_POS);
		
#if SCOPE_SYNC_WITH_RTOS
		// ===== RTOS APPROACH (Event Flags) =====
		uint32_t flags = osEventFlagsWait(scope_event_flags, 
		                                   SCOPE_BUFFERS_FULL_FLAG, 
		                                   osFlagsWaitAny, 
		                                   0);
		
		if (flags & SCOPE_BUFFERS_FULL_FLAG) {
			
#if USE_LCD_MUTEX_LEVEL==1
			DEBUG_START(DBG_BLOCKING_LCD_POS);
			osMutexAcquire(LCD_mutex, osWaitForever);
			DEBUG_STOP(DBG_BLOCKING_LCD_POS);
#endif
			UI_Draw_Waveforms();

#if USE_LCD_MUTEX_LEVEL==1
			osMutexRelease(LCD_mutex);
#endif
			
			DEBUG_STOP(DBG_PENDING_WVFM_POS);
		}

#else
		// ===== STATE MACHINE APPROACH =====
		if (g_scope_state == Full) {
			g_scope_state = Disabled;
			
#if USE_LCD_MUTEX_LEVEL==1
			DEBUG_START(DBG_BLOCKING_LCD_POS);
			osMutexAcquire(LCD_mutex, osWaitForever);
			DEBUG_STOP(DBG_BLOCKING_LCD_POS);
#endif
			UI_Draw_Waveforms();

#if USE_LCD_MUTEX_LEVEL==1
			osMutexRelease(LCD_mutex);
#endif
			
			g_scope_state = Armed;
			DEBUG_STOP(DBG_PENDING_WVFM_POS);
		}
#endif
		
		DEBUG_STOP(DBG_T_DRAW_WVFMS_POS);
	}
}

void Thread_Draw_UI_Controls(void * arg) {
	uint32_t tick;
	
	osMutexAcquire(LCD_mutex, osWaitForever);
	UI_Update_Controls(1);
	osMutexRelease(LCD_mutex);
	tick = osKernelGetTickCount();

	while (1) {
		tick += THREAD_DRAW_UI_CONTROLS_PERIOD_TICKS;
		osDelayUntil(tick);
		DEBUG_START(DBG_T_DRAW_UI_CTLS_POS);
#if USE_LCD_MUTEX_LEVEL==1
		DEBUG_START(DBG_BLOCKING_LCD_POS);
		osMutexAcquire(LCD_mutex, osWaitForever);
		DEBUG_STOP(DBG_BLOCKING_LCD_POS);
#endif
		UI_Update_Controls(0);

#if USE_LCD_MUTEX_LEVEL==1
		osMutexRelease(LCD_mutex);
#endif
		DEBUG_STOP(DBG_T_DRAW_UI_CTLS_POS);
	}
}

void Thread_Update_Setpoint(void * arg) {
	uint32_t tick;
 
	tick = osKernelGetTickCount();
	while (1) {
		tick += THREAD_UPDATE_SETPOINT_PERIOD_TICKS;
		osDelayUntil(tick); 
		DEBUG_START(DBG_TUSP_POS);
		
		// ===========================================================
		// FAULT PROTECTION 2: Check and restore IRQs
		// This runs in a thread, so it continues even when IRQs disabled
		// ===========================================================
		uint32_t primask = __get_PRIMASK();
		if (primask != 0) {
			// Interrupts are disabled! Re-enable them
			__enable_irq();
		}
		
		Update_Set_Current();
		
		DEBUG_STOP(DBG_TUSP_POS);
	}
}
 
void Thread_Read_Accelerometer(void * arg) {
	uint32_t tick = osKernelGetTickCount();
	int period;

#if MMA_USE_INTERRUPTS
	mma_set_active(0);
	enable_mma_interrupt_generation(0x01);
#endif
	mma_set_active(1);
	
	while (1) {
		tick += THREAD_READ_ACCELEROMETER_PERIOD_TICKS;
		osDelayUntil(tick);
		DEBUG_START(DBG_TREADACC_POS);
		uint8_t s = read_status();
		if ((s & 0x08)) { 
			read_full_xyz();
			convert_xyz_to_roll_pitch();
			period = 30 + (int) roll;
			period = MAX(2, period);
			g_flash_period = MIN(period, 180);
			g_flash_duration = MAX(1, g_flash_period/4);
		}
		DEBUG_STOP(DBG_TREADACC_POS);
	}
}

/*----------------------------------------------------------------------------
 *----------------------------------------------------------------------------*/
#include <MKL25Z4.h>
#include <stdio.h>
#include "math.h"
#include <cmsis_os2.h>
#include "GPIO_defs.h"
#include "threads.h"
#include "LCD.h"
#include "LCD_driver.h"
#include "font.h"
#include "LEDs.h"
#include "timers.h"
#include "sound.h"
#include "delay.h"
#include "profile.h"
#include "control.h"
#include "fault.h"
#include "I2C.h"
#include "MMA8451.h"
#include "config.h"

volatile CTL_MODE_E control_mode=DEF_CONTROL_MODE;

// Event flags for scope synchronization (RTOS approach)
#if SCOPE_SYNC_WITH_RTOS
osEventFlagsId_t scope_event_flags;
const uint32_t SCOPE_BUFFERS_FULL_FLAG = (1 << 0);
#endif

#define FAIL_FLASH_LEN (70)

/*----------------------------------------------------------------------------
  Watchdog Timer Functions
  NOTE: Watchdog is configured in SystemInit() in system_MKL25Z4.c
  We only need the Feed function here
 *----------------------------------------------------------------------------*/
void Feed_Watchdog(void) {
    // Must write 0x55 then 0xAA in sequence to refresh watchdog
    SIM->SRVCOP = 0x55;
    SIM->SRVCOP = 0xAA;
}

// Flash red LED with error code
void Fail_Flash(int n) {
	int i;
	
	while (1) {
			i = n;
			do {
				Control_RGB_LEDs(1, 0, 0);
				Delay(FAIL_FLASH_LEN);
				Control_RGB_LEDs(0, 0, 0);
				Delay(FAIL_FLASH_LEN*2);
			} while (--i > 0);
			Delay(FAIL_FLASH_LEN*10);
	}
}

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
	Init_Debug_Signals();
	Init_RGB_LEDs();
	Control_RGB_LEDs(0,0,1);
	
	// Feed watchdog early - it's already running from SystemInit()
	Feed_Watchdog();
	
	LCD_Init();
	
	// Feed watchdog during long initialization
	Feed_Watchdog();
	
	if (!LCD_Text_Init(1)) {
		/* Font bitmaps not found in memory.
		1. Ensure downloading this project doesn't erase all of flash memory. 
			Go to Target Options->Debug->(debugger) Settings->Flash Download ... Select "Erase Sectors"
			Save project and close.
		2. Open Overlay project, build it and program it into MCU. Close Overlay project.
	  3. Reopen this project, build and download.
		*/
		Fail_Flash(2);
	}
	
	Feed_Watchdog();
	
	LCD_Erase();
	LCD_Text_PrintStr_RC(0,0, "ECE 4/560 Project");
	LCD_Text_PrintStr_RC(1,0, "Testing:");
	LCD_Text_PrintStr_RC(2,0, "Accel...");
	
	Feed_Watchdog();
	
	i2c_init();
	
	// Feed watchdog before and after accelerometer init (can be slow)
	Feed_Watchdog();
	
	if (!init_mma()) {
		Fail_Flash(3);
	}
	
	Feed_Watchdog();
	
	LCD_Text_PrintStr_RC(2,9, "Done");
	
	// Break the 250ms delay into chunks to feed watchdog
	for (int i = 0; i < 5; i++) {
		Delay(50);
		Feed_Watchdog();
	}
	
	LCD_Erase();
	
	Init_Buck_HBLED();
	
	Feed_Watchdog();
	
	osKernelInitialize();
	
	Feed_Watchdog();
	
	// Create event flags for scope synchronization (RTOS approach)
#if SCOPE_SYNC_WITH_RTOS
	scope_event_flags = osEventFlagsNew(NULL);
	if (scope_event_flags == NULL) {
		Fail_Flash(4);
	}
#endif
	
	Fault_Init();
	Create_OS_Objects();
	
	// Final watchdog feed before kernel starts
	Feed_Watchdog();
	
	osKernelStart();	
}

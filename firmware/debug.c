#ifndef DEBUG_H
#include <MKL25Z4.h>
#include "debug.h"

volatile int g_enable_DBG_IDLE;
	
debug_GPIO_struct debug_GPIO[DBG_NUM_SIGNALS] = {
							{0, 0, FPTD, PORTD},
							{0, 2, FPTD, PORTD},
							{0, 3, FPTD, PORTD},
							{0, 4, FPTD, PORTD},
							{0, 8, FPTB, PORTB},
							{0, 9, FPTB, PORTB},
							{0, 10, FPTB, PORTB},
							{0, 11, FPTB, PORTB},
#if DBG_USE_SPI_SIGNALS
							{0, 2, FPTE, PORTE},
							{0, 3, FPTE, PORTE},
							{0, 1, FPTE, PORTE},
							{0, 4, FPTE, PORTE},
#endif
							{0, 31, FPTB, PORTB }, // NULL
						};	

void Init_Debug_Signals(void) {
	int i;

	// Enable clock to ports B and D
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;
#if DBG_USE_SPI_SIGNALS
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK; 
#endif	
	
	for (i=0; i<DBG_NUM_SIGNALS; i++) {
		debug_GPIO[i].Port->PCR[debug_GPIO[i].Bit] &= ~PORT_PCR_MUX_MASK; // Make pin GPIO          
		debug_GPIO[i].Port->PCR[debug_GPIO[i].Bit] |= PORT_PCR_MUX(1);          
		debug_GPIO[i].FGPIO->PDDR |= MASK(debug_GPIO[i].Bit);	 // Make an output
		debug_GPIO[i].FGPIO->PCOR = MASK(debug_GPIO[i].Bit); // Clear output
	}
	
#if DEBUG_INIT_TEST
	// Walking debug signals test code
	for (int j=0; j<10; j++) {
		for (i=0; i<DBG_NUM_SIGNALS; i++) {
			DEBUG_START(i);
		}
		for (i=0; i<DBG_NUM_SIGNALS; i++) {
			DEBUG_STOP(i);
		}
		for (i=0; i<DBG_NUM_SIGNALS; i++) {
			DEBUG_TOGGLE(i);
		}
		for (i=0; i<DBG_NUM_SIGNALS; i++) {
			DEBUG_TOGGLE(i);
		}
	}
#endif
}	

#endif // DEBUG_H

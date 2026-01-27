#include <MKL25Z4.h>
#include <stdio.h>
#include <stdint.h>

#include "GPIO_defs.h"
#include "debug.h"
#include "control.h"
#include "config.h"

#include "timers.h"
#include "delay.h"
#include "LEDs.h"
#include "UI.h"
#include "FX.h"

#if SCOPE_SYNC_WITH_RTOS
#include <cmsis_os2.h>
// External event flags for RTOS approach
extern osEventFlagsId_t scope_event_flags;
extern const uint32_t SCOPE_BUFFERS_FULL_FLAG;
#endif

// ==============================================================
// FAULT PROTECTION: External watchdog feeding function
// ==============================================================
extern void Feed_Watchdog(void);

volatile int g_duty_cycle = 5;

volatile int g_enable_flash = 1;
volatile int g_peak_set_current_mA = FLASH_CURRENT_MA;
volatile int g_flash_duration = FLASH_DURATION_MS;
volatile int g_flash_period = FLASH_PERIOD_MS; 

volatile int g_enable_control = 1;
volatile int g_set_current_mA = 0;
volatile int g_set_current_code = 0;

volatile int g_measured_current_mA = 0;
volatile int error= 0;

int32_t pGain_8 = PGAIN_8;

#if !SCOPE_SYNC_WITH_RTOS
// State machine for scope synchronization
volatile SCOPE_STATE_E g_scope_state = Armed;
#endif

volatile __ALIGNED(256) uint16_t g_set_sample[SAM_BUF_SIZE];
volatile __ALIGNED(256) uint16_t g_meas_sample[SAM_BUF_SIZE];

SPid plantPID = {0, 
	0, 
	LIM_DUTY_CYCLE, 
	-LIM_DUTY_CYCLE, 
	P_GAIN_FL, 
	I_GAIN_FL, 
	D_GAIN_FL  
};

SPidFX plantPID_FX = {FL_TO_FX(0), 
	FL_TO_FX(0), 
	FL_TO_FX(LIM_DUTY_CYCLE), 
	FL_TO_FX(-LIM_DUTY_CYCLE), 
	P_GAIN_FX, 
	I_GAIN_FX, 
	D_GAIN_FX  
};

// ==============================================================
// FAULT PROTECTION 1: PID Gains Validation
// Store backup copies and define reasonable ranges
// ==============================================================
const SPidFX plantPID_FX_BACKUP = {
	FL_TO_FX(0),                    // dState
	FL_TO_FX(0),                    // iState
	FL_TO_FX(LIM_DUTY_CYCLE),      // iMax
	FL_TO_FX(-LIM_DUTY_CYCLE),     // iMin
	P_GAIN_FX,                      // pGain - backup value
	I_GAIN_FX,                      // iGain - backup value
	D_GAIN_FX                       // dGain - backup value
};

// Define reasonable gain limits (10x the normal gains)
#define MAX_REASONABLE_GAIN FL_TO_FX(10.0)
#define MIN_REASONABLE_GAIN FL_TO_FX(-10.0)

// Counter for how many times we've detected/corrected PID faults
static volatile int pid_fault_count = 0;

// Function to validate and restore PID gains if corrupted
void Validate_And_Restore_PID_Gains(void) {
	int gains_corrupted = 0;
	
	// Check if any gain is outside reasonable bounds
	if ((plantPID_FX.pGain > MAX_REASONABLE_GAIN) || (plantPID_FX.pGain < MIN_REASONABLE_GAIN)) {
		gains_corrupted = 1;
	}
	if ((plantPID_FX.iGain > MAX_REASONABLE_GAIN) || (plantPID_FX.iGain < MIN_REASONABLE_GAIN)) {
		gains_corrupted = 1;
	}
	if ((plantPID_FX.dGain > MAX_REASONABLE_GAIN) || (plantPID_FX.dGain < MIN_REASONABLE_GAIN)) {
		gains_corrupted = 1;
	}
	
	// If any gain is corrupted, restore ALL gains from backup
	if (gains_corrupted) {
		plantPID_FX.pGain = plantPID_FX_BACKUP.pGain;
		plantPID_FX.iGain = plantPID_FX_BACKUP.iGain;
		plantPID_FX.dGain = plantPID_FX_BACKUP.dGain;
		
		// Also reset integrator and derivative states to prevent instability
		plantPID_FX.iState = FL_TO_FX(0);
		plantPID_FX.dState = FL_TO_FX(0);
		
		// Increment fault counter for debugging/monitoring
		pid_fault_count++;
	}
}

// ==============================================================
// FAULT PROTECTION 2: IRQ Re-enabling
// Periodically check and restore interrupts if disabled
// ==============================================================
#define IRQ_CHECK_PERIOD 5  // Check every 5 control loop iterations
static volatile int irq_check_counter = 0;

void Check_And_Restore_IRQs(void) {
	// Read PRIMASK register - if bit 0 is set, interrupts are disabled
	uint32_t primask = __get_PRIMASK();
	
	if (primask != 0) {
		// FAULT DETECTED: Interrupts are disabled!
		// Re-enable them immediately
		__enable_irq();
	}
}

// ==============================================================
// PID Controller Functions
// ==============================================================
float UpdatePID(SPid * pid, float error, float position){
	float pTerm, dTerm, iTerm;

	pTerm = pid->pGain * error;
	pid->iState += error;
	if (pid->iState > pid->iMax) 
		pid->iState = pid->iMax;
	else if (pid->iState < pid->iMin) 
		pid->iState = pid->iMin;
	iTerm = pid->iGain * pid->iState;
	dTerm = pid->dGain * (position - pid->dState);
	pid->dState = position;

	return pTerm + iTerm - dTerm;
}

FX16_16 UpdatePID_FX(SPidFX * pid, FX16_16 error_FX, FX16_16 position_FX){
	FX16_16 pTerm, dTerm, iTerm, diff, ret_val;

	pTerm = Multiply_FX(pid->pGain, error_FX);

	pid->iState = Add_FX(pid->iState, error_FX);
	if (pid->iState > pid->iMax) 
		pid->iState = pid->iMax;
	else if (pid->iState < pid->iMin) 
		pid->iState = pid->iMin;
	
	iTerm = Multiply_FX(pid->iGain, pid->iState);
	diff = Subtract_FX(position_FX, pid->dState);
	dTerm = Multiply_FX(pid->dGain, diff);
	pid->dState = position_FX;

	ret_val = Add_FX(pTerm, iTerm);
	ret_val = Subtract_FX(ret_val, dTerm);
	return ret_val;
}

// ==============================================================
// Main Control Function with ALL Fault Protections
// ==============================================================
void Control_HBLED(void) {
	uint16_t res;
	FX16_16 change_FX, error_FX;
	static int sample_idx = 0;
	static int prev_set_current_mA = 0;
	static int threshold_mA = SCOPE_TRIGGER_THRESHOLD_mA;
	
	DEBUG_START(DBG_CONTROLLER_POS);
	
	// ==============================================================
	// FAULT PROTECTION 0: Feed Watchdog Timer
	// This is THE MOST CRITICAL fault protection!
	// If this function stops running (e.g., interrupts disabled),
	// the watchdog will timeout and reset the entire system
	// ==============================================================
	Feed_Watchdog();
	
	// ==============================================================
	// FAULT PROTECTION 1: Validate PID Gains
	// Check BEFORE using them in control calculations
	// ==============================================================
	Validate_And_Restore_PID_Gains();
	
	// ==============================================================
	// FAULT PROTECTION 2: Re-enable IRQs if Disabled
	// Check periodically to catch TR_Disable_All_IRQs fault
	// ==============================================================
	irq_check_counter++;
	if (irq_check_counter >= IRQ_CHECK_PERIOD) {
		irq_check_counter = 0;
		Check_And_Restore_IRQs();
	}
	
	// ==============================================================
	// Read ADC Result
	// ==============================================================
#if USE_ADC_INTERRUPT
	// already completed conversion, so don't wait
#else
	while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK))
		; 
#endif
	res = ADC0->R[0];
	g_measured_current_mA = (res*1500)>>16;

	// ============================================================
	// SCOPE SYNCHRONIZATION
	// ============================================================
#if SCOPE_SYNC_WITH_RTOS
	// ===== RTOS APPROACH (Event Flags) =====
	static int is_filling = 0;  // Are we currently filling buffers?
	
	// Check for trigger: setpoint crosses threshold (low to high)
	if (!is_filling && (prev_set_current_mA <= threshold_mA) && (g_set_current_mA > threshold_mA)) {
		// Trigger detected! Start filling buffers
		is_filling = 1;
		sample_idx = 0;
		DEBUG_START(DBG_PENDING_WVFM_POS);  // Signal we're filling
	}
	
	// If we're filling, save samples to buffers
	if (is_filling) {
		if (sample_idx < SAM_BUF_SIZE) {
			g_set_sample[sample_idx] = g_set_current_code;
			g_meas_sample[sample_idx] = res;
			sample_idx++;
			
			// Check if buffers are full
			if (sample_idx >= SAM_BUF_SIZE) {
				// Buffers full! Signal the thread using event flags
				osEventFlagsSet(scope_event_flags, SCOPE_BUFFERS_FULL_FLAG);
				is_filling = 0;  // Stop filling until next trigger
				// Keep DBG_PENDING_WVFM_POS high - thread will clear it
			}
		}
	}
	
	prev_set_current_mA = g_set_current_mA;
	
#else
	// ===== STATE MACHINE APPROACH (No RTOS) =====
	switch(g_scope_state) {
		case Armed:  // Waiting for trigger
			if ((prev_set_current_mA <= threshold_mA) && (g_set_current_mA > threshold_mA)) {
				g_scope_state = Triggered;
				sample_idx = 0;
				DEBUG_START(DBG_PENDING_WVFM_POS);
			}
			break;
			
		case Triggered:  // Filling buffers
			if (sample_idx < SAM_BUF_SIZE) {
				g_set_sample[sample_idx] = g_set_current_code;
				g_meas_sample[sample_idx] = res;
				sample_idx++;
				
				if (sample_idx >= SAM_BUF_SIZE) {
					g_scope_state = Full;
				}
			}
			break;
			
		case Full:  // Ready to draw
			// Wait for thread to start drawing
			break;
			
		case Disabled:  // Thread is drawing
			// Don't modify buffers while thread is using them
			break;
			
		default:
			// Invalid state - reset to Armed
			g_scope_state = Armed;
			sample_idx = 0;
			DEBUG_STOP(DBG_PENDING_WVFM_POS);
			break;
	}
	
	prev_set_current_mA = g_set_current_mA;
#endif
	// ============================================================
	
	// ==============================================================
	// Control Loop Execution
	// At this point, PID gains have been validated
	// ==============================================================
	if (g_enable_control) {
		switch (control_mode) {
			case OpenLoop:
				break;
			case BangBang:
				if (g_measured_current_mA < g_set_current_mA)
					g_duty_cycle = LIM_DUTY_CYCLE;
				else
					g_duty_cycle = 0;
				break;
			case Incremental:
				if (g_measured_current_mA < g_set_current_mA)
					g_duty_cycle += INC_STEP;
				else
					g_duty_cycle -= INC_STEP;
				break;
			case Proportional:
				g_duty_cycle += (pGain_8*(g_set_current_mA - g_measured_current_mA))/256;
				break;
			case PID:
				g_duty_cycle += UpdatePID(&plantPID, g_set_current_mA - g_measured_current_mA, g_measured_current_mA);
				break;
			case PID_FX:
				// PID gains are guaranteed valid because we checked above
				error_FX = INT_TO_FX(g_set_current_mA - g_measured_current_mA);
				change_FX = UpdatePID_FX(&plantPID_FX, error_FX, INT_TO_FX(g_measured_current_mA));
				g_duty_cycle += FX_TO_INT(change_FX);
				break;
			default:
				break;
		}
	
		// Clamp duty cycle to valid range
		if (g_duty_cycle < 0)
			g_duty_cycle = 0;
		else if (g_duty_cycle > LIM_DUTY_CYCLE)
			g_duty_cycle = LIM_DUTY_CYCLE;
			
		PWM_Set_Value(TPM0, PWM_HBLED_CHANNEL, g_duty_cycle);
	}
	
	DEBUG_STOP(DBG_CONTROLLER_POS);
}

// ==============================================================
// DAC Functions
// ==============================================================
void Set_DAC(unsigned int code) {
	uint16_t * dac0dat = (uint16_t *)&(DAC0->DAT[0].DATL);
	*dac0dat = (uint16_t) code;
}

void Set_DAC_mA(unsigned int current) {
	unsigned int code = MA_TO_DAC_CODE(current);
	uint16_t * dac0dat = (uint16_t *)&(DAC0->DAT[0].DATL);
	*dac0dat = (uint16_t) code;
}

// ==============================================================
// Hardware Initialization Functions
// ==============================================================
void Init_DAC_HBLED(void) {
	SIM->SCGC6 |= SIM_SCGC6_DAC0_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	PORTE->PCR[DAC_POS] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[DAC_POS] |= PORT_PCR_MUX(0);	
		
	DAC0->C1 = 0;
	DAC0->C2 = 0;
	
	DAC0->C0 = DAC_C0_DACEN_MASK | DAC_C0_DACRFS_MASK;
	Set_DAC(0);
}

void Init_ADC_HBLED(void) {
	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK; 
	ADC0->CFG1 = 0x0C;
	ADC0->SC2 = ADC_SC2_REFSEL(0);

#if USE_ADC_HW_TRIGGER
	ADC0->SC2 |= ADC_SC2_ADTRG(1);
	SIM->SOPT7 = SIM_SOPT7_ADC0TRGSEL(8) | SIM_SOPT7_ADC0ALTTRGEN_MASK;
	ADC0->SC1[0] &= ~ADC_SC1_ADCH_MASK;
	ADC0->SC1[0] |= ADC_SC1_ADCH(ADC_SENSE_CHANNEL);
#endif

#if USE_ADC_INTERRUPT 
#if !USE_ADC_SERVER
	ADC0->SC1[0] |= ADC_SC1_AIEN(1);
#endif
	NVIC_SetPriority(ADC0_IRQn, 2); 
	NVIC_ClearPendingIRQ(ADC0_IRQn); 
	NVIC_EnableIRQ(ADC0_IRQn);	
#endif
}

void Init_Buck_HBLED(void) {
	Init_DAC_HBLED();
	Init_ADC_HBLED();
	
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	PORTE->PCR[31]  &= PORT_PCR_MUX(7);
	PORTE->PCR[31]  |= PORT_PCR_MUX(3);
	PWM_Init(TPM0, PWM_HBLED_CHANNEL, PWM_PERIOD, g_duty_cycle, 0, 0);
}

// ==============================================================
// Setpoint Update Functions
// ==============================================================
#if 1
void Update_Set_Current(void) {
	static volatile int delay = 0;
	
	if (delay == 0)
		delay = g_flash_period;
	
	if (g_enable_flash){
		delay--;
		if (delay == g_flash_duration) {
			g_set_current_mA = g_peak_set_current_mA;
			Set_DAC_mA(g_set_current_mA);
			g_set_current_code = (g_set_current_mA<<16)/1500; 
		} else if (delay == 0) {
			delay = g_flash_period;
			g_set_current_mA = 0;
			g_set_current_code = 0; 
			Set_DAC_mA(g_set_current_mA);
		}
	}
}
#else
void Update_Set_Current(void) {
	static volatile int t = 0;
	int phase;
	int t_ramp;
	
	if (g_enable_flash){
		t_ramp = g_flash_duration/2;
		t++;
		phase = (2*t)/g_flash_duration;
		switch (phase) {
			case 0:
				g_set_current_mA = (t*g_peak_set_current_mA)/t_ramp;
				break;
			case 1:
				g_set_current_mA = ((2*t_ramp - t)*g_peak_set_current_mA)/t_ramp;
				break;
			default:
				g_set_current_mA = 0;
				break;
		}
		Set_DAC_mA(g_set_current_mA);
		if (t >= g_flash_period)
			t = 0;
	}
}
#endif

// ==============================================================
// UI Handler Functions
// ==============================================================
void Control_OnOff_Handler (UI_FIELD_T * fld, int v) {
	if (fld->Val != NULL) {
		if (v > 0) {
			*fld->Val = 1;
		} else {
			*fld->Val = 0;
		}
	}
}

void Control_IntNonNegative_Handler (UI_FIELD_T * fld, int v) {
	int n;
	if (fld->Val != NULL) {
		n = *fld->Val + v/16;
		if (n < 0) {
			n = 0;
		}
		*fld->Val = n;
	}
}

void Control_DutyCycle_Handler(UI_FIELD_T * fld, int v) {
	int dc;
	if (fld->Val != NULL) {
		dc = g_duty_cycle + v/16;
		if (dc < 0)
			dc = 0;
		else if (dc > LIM_DUTY_CYCLE)
			dc = LIM_DUTY_CYCLE;
		*(fld->Val) = dc;
		PWM_Set_Value(TPM0, PWM_HBLED_CHANNEL, g_duty_cycle);
	}
}

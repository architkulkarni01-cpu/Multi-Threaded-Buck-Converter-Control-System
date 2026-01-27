#include <MKL25Z4.h>
#include <stdio.h>
#include <stdint.h>
#include <cmsis_os2.h>
#include "GPIO_defs.h"
#include "debug.h"

#include "config.h"
#include "control.h"
#include "LCD_driver.h"
#include "timers.h"
#include "delay.h"
#include "LEDs.h"
#include "UI.h"
#include "FX.h"
#include "ADC.h"

osMessageQueueId_t  ADC_RequestQueue;
osMessageQueueId_t  ADC_ResponseQueue;

void ADC_Update_MuxSel(uint32_t m) {
	if (m)
		ADC0->CFG2 |= ADC_CFG2_MUXSEL_MASK;
	else
		ADC0->CFG2 &= ~ADC_CFG2_MUXSEL_MASK;
}

#if USE_ADC_INTERRUPT
#if USE_ADC_SERVER        // moved to ADC.c to keep everything together.
/***************************************
 * ADC0_IRQHandler:
 ****************************************/
void ADC0_IRQHandler() {
	volatile static uint8_t modeHBLED=1;
	static ADC_Request_t req;
	static ADC_Response_t res;
	static volatile osStatus_t qstat = osOK;
	volatile static uint16_t t1,t2;
	int diff=0;
	DEBUG_START(DBG_ADC_ISR_POS);

	if (modeHBLED) {
		Control_HBLED();
		t1=TPM0->CNT;
		t2=TPM0->CNT;
		diff=PWM_PERIOD-(int)t2;
		if (t2 < t1) diff+=PWM_PERIOD;   // if cnt down, add a full PWM_PERIOD to counts left.
		if (diff > TPM_WINDOW) {
			qstat=osMessageQueueGet(ADC_RequestQueue,&req,NULL,0);
			if (qstat == osOK) {                 // did we get one?
				// Configure the low-priority conversion
				modeHBLED=0;
				ADC_Update_MuxSel(req.MuxSel);
				ADC0->SC2&=~ADC_SC2_ADTRG_MASK;     // select software trigger
				ADC0->SC1[0] = ADC_SC1_AIEN(1)|ADC_SC1_ADCH(req.Channel & ADC_SC1_ADCH_MASK);
//				DEBUG_START(DBG_LOPRI_ADC_POS);
			}  // else no pending request or unable to fetch request.		
		}
	} else {													// Else we must be here for a low-prio conversion.
		res.Sample=ADC0->R[0];        // first read the value in case we trigger right away.
		modeHBLED=1;
#if USE_SYNC_NO_FREQ_DIV
		// Re-enable TPMO overflow to trigger conversion 
		ADC0->SC2|=ADC_SC2_ADTRG(1);  // select hardware trigger
		ADC_Update_MuxSel(ADC_SENSE_MUXSEL);
		ADC0->SC1[0] = ADC_SC1_AIEN(1)|ADC_SC1_ADCH(ADC_SENSE_CHANNEL);
#else
		// Let TPM IRQ handler start ADC conversion with software
#endif
		res.Channel=req.Channel;
		res.MuxSel = req.MuxSel;
		qstat=osMessageQueuePut(req.ResponseQueue,&res,NULL,0);
//		DEBUG_STOP(DBG_LOPRI_ADC_POS);
	}
	DEBUG_STOP(DBG_ADC_ISR_POS);
}
#else // don't use ADC server
void ADC0_IRQHandler() {
	FPTB->PSOR = MASK(DBG_ADC_ISR_POS);
	Control_HBLED();
	FPTB->PCOR = MASK(DBG_ADC_ISR_POS);
}
#endif // don't use ADC server
#endif // USE_ADC_INTERRUPT

#if USE_ADC_SERVER
/***************************************
 * request_conversion:
 *
 * Request a low-priority conversion.
 *
* mostly just abstracts the conversion
 * request process into the ADC server code.
 * 
 ****************************************/
uint16_t request_conversion(uint8_t channel, uint8_t muxsel) {
	static ADC_Request_t req;
	ADC_Response_t  res;
	osStatus_t qstat;

	DEBUG_START(DBG_LOPRI_ADC_POS);
	req.Channel=channel;
	req.MuxSel = muxsel;
	req.ResponseQueue=ADC_ResponseQueue;
	qstat=osErrorResource;
	while (qstat == osErrorResource) // Keep trying if the queue is full
		qstat = osMessageQueuePut(ADC_RequestQueue,&req, 0, osWaitForever);
	if (qstat != osOK) {             // Did we error out for another reason.
		DEBUG_STOP(DBG_LOPRI_ADC_POS);
		return(0);      // return something even if we failed to enqueue
	}

	// enqueue the response.
	qstat=osMessageQueueGet(ADC_ResponseQueue,&res,NULL,osWaitForever);
	DEBUG_STOP(DBG_LOPRI_ADC_POS);
	return(res.Sample);
}

void Init_ADC(void) {

	ADC_RequestQueue=osMessageQueueNew(4,sizeof(ADC_Request_t),NULL);
	ADC_ResponseQueue=osMessageQueueNew(4,sizeof(ADC_Response_t),NULL);

	// Configure ADC to read Ch 8 (FPTB 0)
	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK; 
	ADC0->CFG1 = 0x0D; // 0000 1101
	// 0 Normal power
	// 00 ADIV clock divide ratio 1
	// 0  ADLSMP short sample time
	// 11 MODE 16 bit 
	// 01 ADICLK Bus clock/2

	ADC0->CFG2 = 4; // 0000 0100 
	// 0 MUXSEL ADxxa channels selected
	// 0 ADACKEN Don't use asynch clock output
	// 1 ADHSC Use high-speed configuration
	// 00 ADLSTS (not used)

	ADC0->SC2 = ADC_SC2_REFSEL(0);

	// Configure NVIC for ADC interrupt
	NVIC_SetPriority(ADC0_IRQn, 2); 
	NVIC_ClearPendingIRQ(ADC0_IRQn); 
	NVIC_EnableIRQ(ADC0_IRQn);	

	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK; // Why enable TMP0 here?
		
	ADC_Update_MuxSel(ADC_SENSE_MUXSEL);
	ADC0->SC1[0] = ADC_SC1_AIEN(1)|ADC_SC1_ADCH(ADC_SENSE_CHANNEL & ADC_SC1_ADCH_MASK);
	// AIEN: ADC Interrupt Enable
}
#else
void Init_ADC(void) {
	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK; 
	ADC0->CFG1 = 0x9C; // 16 bit
	ADC0->SC2 = 0;
}
#endif

#include <MKL25Z4.h>
#include <math.h>
#include <stdio.h>

#include "MMA8451.h"
#include "I2C.h"
#include "delay.h"
#include "LEDs.h"
#include "config.h"
#include "debug.h"
#include "threads.h"

#define VALIDATE 0
#define SHOW_DATA 0
#define MAX_ERROR 1

#define M_PI_2 (M_PI/2.0f)
#define M_PI_4 (M_PI/4.0f)

int16_t acc_X=0, acc_Y=0, acc_Z=0;
float roll=0.0, pitch=0.0;


#if MMA_USE_INTERRUPTS
volatile int TAI = 0;
volatile uint8_t status=0, int_source=0;

void PORTA_IRQHandler(void){
	DEBUG_START(DBG_PORTA_IRQ);
	if (MMA_INT_PORT->ISFR & MASK(MMA_INT1_POS)) {
		// Acknowledge interrupt 
		MMA_INT_PORT->ISFR = MASK(MMA_INT1_POS);
		TAI = 1;
		// handle INT1
	#if PERIODIC_READ_ACCEL
	#else
		osThreadFlagsSet(t_Read_Accelerometer, 1);
	#endif
	}
	if (MMA_INT_PORT->ISFR & MASK(MMA_INT2_POS)) {
		// Acknowledge interrupt 
		MMA_INT_PORT->ISFR = MASK(MMA_INT2_POS);
		TAI = 1;
		// handle INT2
	#if PERIODIC_READ_ACCEL
	#else
		osThreadFlagsSet(t_Read_Accelerometer, 2);
	#endif
	}
	#if 0 // DEBUG: get MMA status/source
	status = i2c_read_byte(MMA_ADDR,REG_STATUS);
	int_source = i2c_read_byte(MMA_ADDR,REG_INT_SOURCE);
	#endif
	DEBUG_STOP(DBG_PORTA_IRQ);
}
#endif

void mma_set_active(uint8_t active) {
	uint8_t val = i2c_read_byte(MMA_ADDR, REG_CTRL1);
	ShortDelay(MMA_DELAY_TBUF);
	if (active)
		val |= 0x01;
	else 
		val &= ~0x01;
	i2c_write_byte(MMA_ADDR, REG_CTRL1, val);
	ShortDelay(MMA_DELAY_TBUF);
}

void enable_mma_interrupt_generation(uint8_t mask) {
	i2c_write_byte(MMA_ADDR, REG_CTRL4, mask);
	ShortDelay(MMA_DELAY_TBUF);
}

uint8_t read_status(void) {
	uint8_t d = i2c_read_byte(MMA_ADDR, REG_STATUS);
	ShortDelay(MMA_DELAY_TBUF);
	return d;
}

/* 
  Reads full 16-bit X, Y, Z accelerations.
*/
void read_full_xyz()
{
	int i;
	uint8_t data[6];
	
	i2c_start();
	i2c_read_setup(MMA_ADDR , REG_XHI);
	
	for( i=0;i<6;i++)	{
		data[i] = i2c_repeated_read(i==5? 1 : 0);
	}
	acc_X = (((int16_t) data[0])<<8) | data[1];
	acc_Y = (((int16_t) data[2])<<8) | data[3];
	acc_Z = (((int16_t) data[4])<<8) | data[5];
	ShortDelay(MMA_DELAY_TBUF);
}

void read_xyz(void)
{
	// sign extend byte to 16 bits - need to cast to signed since function
	// returns uint8_t which is unsigned
	acc_X = ((int16_t) ((int8_t) i2c_read_byte(MMA_ADDR, REG_XHI))) << 8;
	ShortDelay(MMA_DELAY_TBUF);
	acc_Y = ((int16_t) ((int8_t) i2c_read_byte(MMA_ADDR, REG_YHI))) << 8;
	ShortDelay(MMA_DELAY_TBUF);
	acc_Z = ((int16_t) ((int8_t) i2c_read_byte(MMA_ADDR, REG_ZHI))) << 8;
	ShortDelay(MMA_DELAY_TBUF);
}

/*
 Initializes mma8451 sensor. I2C has to already be configured and enabled.
 */
int init_mma()
{
	Delay(100); // Give I2C time to stabilize? Investigate. <<<
	//check for device
	if (i2c_read_byte(MMA_ADDR, REG_WHOAMI) != WHOAMI)
		return 0;
		
	ShortDelay(MMA_DELAY_TBUF);
	// Reset device
	i2c_write_byte(MMA_ADDR, REG_CTRL2, 0x40);
	Delay(500); // Delay after software reset (1 ms recommended, actual time unknown)
			
	//select 14bit mode, low noise, data rate and standby mode
	i2c_write_byte(MMA_ADDR, REG_CTRL1, MMA_CTRL1_DR(4));
	ShortDelay(MMA_DELAY_TBUF);
		
	#if MMA_USE_INT1
	// change data ready interrupt to INT1 (PTA14)
	i2c_write_byte(MMA_ADDR, REG_CTRL5, 0x01);
	ShortDelay(MMA_DELAY_TBUF);
	// Configure for input: R/M/W
	MMA_INT_GPIO->PDDR &= ~MASK(MMA_INT1_POS);
	// Configure for interrupt on falling edge, no pull resistors (accel ints are push-pull): Write ok
	MMA_INT_PORT->PCR[MMA_INT1_POS] = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0a);
	#endif
	
	#if MMA_USE_INT2
	// Configure for input: R/M/W
	MMA_INT_GPIO->PDDR &= ~MASK(MMA_INT2_POS);
	// Configure for interrupt on falling edge, no pull resistors (accel ints are push-pull): Write ok
	MMA_INT_PORT->PCR[MMA_INT2_POS] = PORT_PCR_MUX(1) | PORT_PCR_IRQC(0x0a);
	#endif
	
	#if (MMA_USE_INTERRUPTS) 
	// enable the irq in the NVIC
	NVIC_SetPriority(MMA_INT_IRQn, 2); 
	NVIC_ClearPendingIRQ(MMA_INT_IRQn); 
	NVIC_EnableIRQ(MMA_INT_IRQn);
	#endif

	mma_set_active(1);
	ShortDelay(MMA_DELAY_TBUF);
	return 1;
}

float approx_sqrtf(float z) { // from Wikipedia
	int val_int = *(int*)&z; /* Same bits, but as an int */
	const int a = 0x4c000;

  val_int -= 1 << 23; /* Subtract 2^m. */
  val_int >>= 1; /* Divide by 2. */
  val_int += 1 << 29; /* Add ((b + 1) / 2) * 2^m. */
	val_int += a;
	//	val_int = (1 << 29) + (val_int >> 1) - (1 << 22) + a;
	return *(float*)&val_int; /* Interpret again as float */
}

float approx_atan2f(float y, float x) {
	float a, abs_a, approx, adj=0.0;
	char negate = 0;
	
	if (x == 0) { // special cases
		if (y == 0.0)
			return 0.0; // undefined, but return 0 by convention
		else if (y < 0.0)
			return -M_PI_2;
		else
			return M_PI_2;
	}	else {
		a = y/x;
		if (a>1) {
			a = x/y;
			adj = M_PI_2;
			negate = 1;
		} else if (a<-1) {
			a = x/y;
			adj = -M_PI_2;
			negate = 1;
		}
		abs_a = (a < 0)? -a : a;
		approx = M_PI_4*a - a*(abs_a - 1)*(0.2447+0.0663*abs_a);
		if (negate) {
			approx = adj - approx;
		}
		
		if (x > 0)
			return approx;
		else if (y >= 0)
			return approx + M_PI;
		else
			return approx - M_PI;
	}		
}

void convert_xyz_to_roll_pitch(void) {
	float ax = acc_X/COUNTS_PER_G,
				ay = acc_Y/COUNTS_PER_G,
				az = acc_Z/COUNTS_PER_G;
#if VALIDATE
	float roll_ref, pitch_ref;
#endif
	// See NXP/Freescale App Note AN3461 for explanations
#if UP_AXIS_Z // original code - horizontal mode
	#if 0	// select equation for roll
	roll = atan2(ay, az)*180/M_PI; 	// Eqn. 25
  #else
	roll = atan2(ay, SIGN(az)*(sqrt(az*az + MU*ax*ax)))*180/M_PI; 	// Eqn. 38
  #endif	
	pitch = atan2(ax, sqrt(ay*ay + az*az))*180/M_PI; // Eqn. 26
#endif
	
#if UP_AXIS_X 
	// vertical board: X axis (instead of Z) points up 
	roll = atan2(ay, ax)*180/M_PI;
	pitch = atan2(az, sqrt(ay*ay + ax*ax))*180/M_PI; // Eqn. 26
#endif	

#if VALIDATE
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) { 
		roll_ref = atan2f(ay, az)*(180/M_PI);
		pitch_ref = atan2f(ax, sqrt(ay*ay + az*az))*(180/M_PI);
		if (fabs(roll-roll_ref) > MAX_ERROR) {
			printf("Roll Error: %f, should be %f.\n\r ay=%f, az=%f\n\r", roll, roll_ref, ay, az);
		}
		if (fabs(pitch-pitch_ref) > MAX_ERROR) {
			printf("Pitch Error: %f, should be %f.\n\r ax=%f, ay=%f, az=%f\n\r", pitch, pitch_ref, ax, ay, az);
		}
	}
#endif
	
#if SHOW_DATA
	printf("Roll: %f \tPitch: %f\n\r", roll, pitch);
#endif
}

float Acc_Get_Magnitude(void) {
	float x, y, z;
	x = acc_X/COUNTS_PER_G;
	y = acc_Y/COUNTS_PER_G;
	z = acc_Z/COUNTS_PER_G;
	return(sqrt(x*x + y*y + z*z));
}


#if MMA_USE_INTERRUPTS
void test_acc_int(void) {
	DEBUG_START(DBG_TREADACC_POS);
	if (!init_mma()) {							// init accelerometer
		Control_RGB_LEDs(1,0,0);			// accel initialization failed, so turn on red error light
		while (1)
			;
	}
	DEBUG_STOP(DBG_TREADACC_POS);
	DEBUG_START(DBG_PORTA_IRQ);

	mma_set_active(0);
	enable_mma_interrupt_generation(0x01);
	mma_set_active(1);
	DEBUG_STOP(DBG_PORTA_IRQ);
	
	while (1) {
#if 0
		while (!TAI) {
			DEBUG_TOGGLE(DBG_PORTA_IRQ);
		}
		TAI = 0;
#else
		if (TAI) {
			TAI = 0;
			DEBUG_TOGGLE(DBG_PORTA_IRQ);
		}
#endif
		DEBUG_TOGGLE(DBG_TREADACC_POS);
		read_full_xyz();
	}
}
#endif

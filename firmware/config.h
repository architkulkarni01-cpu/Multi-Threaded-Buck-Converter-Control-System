#ifndef CONFIG_H
#define CONFIG_H
#include "GPIO_defs.h"

// Platform
// Select one compiler
// #define USING_AC5
#define USING_AC6
#define USING_RTOS

// Application Program
#define USE_ADC_SERVER 				(1)
#define USE_LCD_MUTEX_LEVEL  	(1) // Change to 2, 3 as you try out different mutex levels within the threads

// LCD and Graphics Optimizations
#define LCD_BUS_DEFAULTS_TO_DATA 1 
#define DRAW_LINE_RUNS_AS_RECTANGLES 1 
#define USE_TEXT_BITMAP_RUNS 1 

// I2C Configuration
#define READ_FULL_XYZ 1 
#define I2C_ICR_VALUE 0x20 

// ========================================
// Scope Synchronization Configuration
// ========================================

// Scope synchronization method selection
// 0 = State machine approach (no RTOS mechanisms)
// 1 = RTOS approach (event flags)
#define SCOPE_SYNC_WITH_RTOS  1  // CHANGED TO 1 FOR RTOS

// States for scope synchronization (state machine approach)
typedef enum {
    SCOPE_IDLE,           // Waiting for trigger
    SCOPE_FILLING,        // ISR filling buffers
    SCOPE_READY_TO_DRAW,  // Buffers full, ready for thread to draw
    SCOPE_DRAWING         // Thread is drawing
} scope_state_t;

#endif // CONFIG_H

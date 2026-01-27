#include "UI.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <cmsis_os2.h>

#include "LCD.h"
#include "colors.h"
#include "ST7789.h"
#include "font.h"
#include "control.h"
#include "FX.h"
#include "debug.h"
#include "timers.h"

volatile int g_scope_height = INIT_SCOPE_HEIGHT;
volatile int g_holdoff = PRE_TRIG_SAMPLES;

// Label, Units, Buffer, *Val, *VatT, {row,column},
// fg, bg, Updated, Selected, ReadOnly, Volatile, *Handler 

UI_FIELD_T Fields[] = {
	{"Duty Cycle  ", "ct", "", (volatile int *)&g_duty_cycle, NULL, {0,7}, 
	&green, &black, 1, 0, 1, 1,Control_DutyCycle_Handler},
	{"Enable Ctlr ", "", "", (volatile int *)&g_enable_control, NULL, {0,8}, 
	&green, &black, 1, 0, 0, 0, Control_OnOff_Handler},	
	{"T_flash_prd ", "ms", "", (volatile int *)&g_flash_period, NULL, {0,9}, 
	&orange, &black, 1, 0, 1, 1, NULL}, // Control_IntNonNegative_Handler},		
	{"T_flash_on  ", "ms", "", (volatile int *)&g_flash_duration, NULL, {0,10}, 
	&green, &black,  1, 0, 1, 1, Control_IntNonNegative_Handler},
	{"I_set       ", "mA", "", (volatile int *)&g_set_current_mA, NULL, {0,11}, 
	&green, &black, 1, 0, 1, 1, Control_IntNonNegative_Handler},
	{"I_set_peak  ", "mA", "", (volatile int *)&g_peak_set_current_mA, NULL, {0,12}, 
	&green, &black, 1, 0, 0, 0, Control_IntNonNegative_Handler},
	{"I_measured  ", "mA", "", (volatile int *)&g_measured_current_mA, NULL, {0,13}, 
	&orange, &black, 1, 0, 1, 1, NULL},
};

UI_SLIDER_T Slider = {
	0, {0,LCD_HEIGHT-UI_SLIDER_HEIGHT}, {UI_SLIDER_WIDTH-1,LCD_HEIGHT-1}, 
	{119,LCD_HEIGHT-UI_SLIDER_HEIGHT}, {119,LCD_HEIGHT-1}, &white, &dark_gray, &light_gray
};

int UI_sel_field = -1;

void UI_Update_Field_Values (UI_FIELD_T * f, int num) {
	int i;
	for (i=0; i < num; i++) {
		snprintf(f[i].Buffer, sizeof(f[i].Buffer), "%s%3d %s", f[i].Label, f[i].Val? *(f[i].Val) : 0, f[i].Units);
		f[i].Updated = 1;
	}	
}

void UI_Update_Volatile_Field_Values(UI_FIELD_T * f) {
	int i;
	for (i=0; i < UI_NUM_FIELDS; i++) {
		if (f[i].Volatile) {
			snprintf(f[i].Buffer, sizeof(f[i].Buffer), "%s%3d %s", f[i].Label, f[i].Val? *(f[i].Val) : 0, f[i].Units);
			f[i].Updated = 1;
		}
	}
}

void UI_Draw_Fields(UI_FIELD_T * f, int num){
	int i;
	COLOR_T * bg_color, *fg_color;
	for (i=0; i < num; i++) {
		if ((f[i].Updated) || (f[i].Volatile)) { // redraw updated or volatile fields
			f[i].Updated = 0;
			if ((f[i].Selected) && (!f[i].ReadOnly)) {
				bg_color = &dark_red;
			} else {
				bg_color = f[i].ColorBG;
			}
			if (f[i].ReadOnly) {
				fg_color = &light_gray;
			} else {
				fg_color = f[i].ColorFG;
			}
			LCD_Text_Set_Colors(fg_color, bg_color);
			LCD_Text_PrintStr_RC(f[i].RC.Y, f[i].RC.X, f[i].Buffer);
		}
	}
}

void UI_Draw_Slider(UI_SLIDER_T * s) {
	static int initialized=0;
	
	if (!initialized) {
		LCD_Fill_Rectangle(&s->UL, &s->LR, s->ColorBG);
		initialized = 1;
	}
	LCD_Fill_Rectangle(&s->BarUL, &s->BarLR, s->ColorBG); // Erase old bar
	
	s->BarUL.Y = s->UL.Y;
	s->BarLR.Y = s->LR.Y;
	s->BarUL.X = (s->LR.X - s->UL.X)/2 + s->Val;
	s->BarLR.X = s->BarUL.X + UI_SLIDER_BAR_WIDTH/2;
	s->BarUL.X -= UI_SLIDER_BAR_WIDTH/2;
	LCD_Fill_Rectangle(&s->BarUL, &s->BarLR, s->ColorFG); // Draw new bar
}

int UI_Identify_Field(PT_T * p) {
	int i, t, b, l, r;

	if ((p->X >= LCD_WIDTH) || (p->Y >= LCD_HEIGHT)) {
		return -1;
	}

	if ((p->X >= Slider.UL.X) && (p->X <= Slider.LR.X) 
		&& (p->Y >= Slider.UL.Y) && (p->Y <= Slider.LR.Y)) {
		return UI_SLIDER;
	}
  for (i=0; i<UI_NUM_FIELDS; i++) {
		l = COL_TO_X(Fields[i].RC.X);
		r = l + strlen(Fields[i].Buffer)*CHAR_WIDTH;
		t = ROW_TO_Y(Fields[i].RC.Y);
		b = t + CHAR_HEIGHT-1;
		
		if ((p->X >= l) && (p->X <= r) 
			&& (p->Y >= t) && (p->Y <= b) ) {
			return i;
		}
	}
	return -1;
}

void UI_Update_Field_Selects(int sel) {
	int i;
	for (i=0; i < UI_NUM_FIELDS; i++) {
		Fields[i].Selected = (i == sel)? 1 : 0;
	}
}

void UI_Process_Touch(PT_T * p) {  // Called by Thread_Read_TS
	int i;
	
	i = UI_Identify_Field(p);
	if (i == UI_SLIDER) {
		Slider.Val = p->X - (Slider.LR.X - Slider.UL.X)/2; // Determine slider position (value)
		if (UI_sel_field >= 0) {  // If a field is selected...
			if (Fields[UI_sel_field].Val != NULL) {
				if (Fields[UI_sel_field].Handler != NULL) {
					(*Fields[UI_sel_field].Handler)(&Fields[UI_sel_field], Slider.Val); // Have the field handle the new slider value
				}
				UI_Update_Field_Values(&Fields[UI_sel_field], 1);
			}
		}
	} else if (i>=0) {
		if (!Fields[i].ReadOnly) { // Can't select (and modify) a ReadOnly field
			UI_sel_field = i;
			UI_Update_Field_Selects(UI_sel_field);
			UI_Update_Field_Values(Fields, UI_NUM_FIELDS);
			Slider.Val = 0; // return to slider to zero if a different field is selected
		}
	} 
}

int ave_samples(volatile uint16_t *buff) {
	unsigned int sum=0;
	for (int i=0;i<SAMPLES_PER_PIXEL;i++) sum+=buff[i];
	return(sum/SAMPLES_PER_PIXEL);
}

void UI_Draw_Scope(int trig_sample) {
	PT_T tp1,tp2;
	int start_idx = 0;
	int setY, measY, idx = 0;


	// Clear the scope area.
	tp1.X = 0;	tp1.Y = 0;
	tp2.X = LCD_WIDTH;	tp2.Y = g_scope_height;
	LCD_Fill_Rectangle(&tp1,&tp2,&black);

	start_idx = 0; 
	// Loop through SAM_BUF_SIZE of samples
	for (int x = 0; x < SAM_BUF_SIZE; x+=SAMPLES_PER_PIXEL) {
		idx = (start_idx+x)%SAM_BUF_SIZE;	// active buffer position Shift by start_idx and wrap		
		tp1.X = (x/SAMPLES_PER_PIXEL);				// displayX is sampleX/4
		// plot set point
		setY = SCALE_ADC_CODE_TO_SCOPE(ave_samples(&g_set_sample[idx]));
		tp1.Y = SCALE_SCOPE(setY);		
		tp1.Y = CLIP_SCOPE(tp1.Y);
		LCD_Plot_Pixel(&tp1,&blue);
		
		// plot measured data
		measY = SCALE_ADC_CODE_TO_SCOPE(ave_samples(&g_meas_sample[idx]));
		tp1.Y = SCALE_SCOPE(measY);		
		tp1.Y = CLIP_SCOPE(tp1.Y);
		LCD_Plot_Pixel(&tp1,&orange);
	}
}

void UI_Draw_Waveforms(void) {
	UI_Draw_Scope(0);
}

void UI_Update_Controls(int first_time) { 
	if (first_time)
		UI_Update_Field_Values(Fields, UI_NUM_FIELDS);

	UI_Update_Volatile_Field_Values(Fields);
	UI_Draw_Fields(Fields, UI_NUM_FIELDS);
	UI_Draw_Slider(&Slider);
}

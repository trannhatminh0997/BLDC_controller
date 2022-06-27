
#ifndef __BLDC_CONTROL_
#define __BLDC_CONTROL_

#include "AS5047D.h"
#include <stdint.h>
#define _USE_MATH_DEFINES // for C
#include <math.h>

//#include "stm32f4xx_hal_gpio.h"
//#include "stm32f4xx_hal_gpio.h"
#include "stm32f1xx_hal.h"

#define pi 3.14159265359
#define sin60 0.86602540378
#define cos60 0.5
#define radConv(deg) (((deg)/180.0)*pi)
#define degConv(rad) (((rad)/pi)*180.0)
#define CW 1
#define CCW 0





typedef struct 
{
	TIM_HandleTypeDef *HTIM_AB_Phase,*HTIM_C_Phase;
	uint32_t AH,AL,BH,BL,CH,CL;
	uint32_t PWM_period;
}HDriver;


typedef struct 
{
	uint16_t a ;
	uint16_t b ;
	uint16_t c ;
}vec3;

typedef struct
{
	float Angle_phase;
	void* RotoPhase;
	HDriver HDriver_;
	float m_vector;
	vec3 pwm;
	
}BLDC_Handle;



void DriverStart(HDriver * HDriver_);
void DriverInit(HDriver * HDriver_);
void DoPwm(BLDC_Handle * BLDC_Handle_);
void SVPWMcalc(BLDC_Handle* BLDC_Handle_);





#endif

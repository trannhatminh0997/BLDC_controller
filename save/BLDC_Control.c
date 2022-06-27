#include "BLDC_Control.h"
#include <stdint.h>

const uint16_t DeadTime = 10; //based on period value



void DriverStart(HDriver * HDriver_)
{
	HAL_TIM_PWM_Start(HDriver_->HTIM_AB_Phase,HDriver_->AH);
	HAL_TIM_PWM_Start(HDriver_->HTIM_AB_Phase,HDriver_->AL);
	HAL_TIM_PWM_Start(HDriver_->HTIM_AB_Phase,HDriver_->BH);
	HAL_TIM_PWM_Start(HDriver_->HTIM_AB_Phase,HDriver_->BL);
	HAL_TIM_PWM_Start(HDriver_->HTIM_C_Phase,HDriver_->CH);
	HAL_TIM_PWM_Start(HDriver_->HTIM_C_Phase,HDriver_->CL);
}

void DriverInit(HDriver * HDriver_)
{
	HDriver_->PWM_period = HDriver_->HTIM_AB_Phase->Init.Period;
	DriverStart(HDriver_);
}

void DoPwm(BLDC_Handle * BLDC_Handle_)
{
	__HAL_TIM_SET_COMPARE(BLDC_Handle_->HDriver_.HTIM_AB_Phase,BLDC_Handle_->HDriver_.AH,BLDC_Handle_->pwm.a-DeadTime);
	__HAL_TIM_SET_COMPARE(BLDC_Handle_->HDriver_.HTIM_AB_Phase,BLDC_Handle_->HDriver_.AL,BLDC_Handle_->pwm.a);
	__HAL_TIM_SET_COMPARE(BLDC_Handle_->HDriver_.HTIM_AB_Phase,BLDC_Handle_->HDriver_.BH,BLDC_Handle_->pwm.b-DeadTime);
	__HAL_TIM_SET_COMPARE(BLDC_Handle_->HDriver_.HTIM_AB_Phase,BLDC_Handle_->HDriver_.BL,BLDC_Handle_->pwm.b);
	__HAL_TIM_SET_COMPARE(BLDC_Handle_->HDriver_.HTIM_C_Phase, BLDC_Handle_->HDriver_.CH,BLDC_Handle_->pwm.c-DeadTime);
	__HAL_TIM_SET_COMPARE(BLDC_Handle_->HDriver_.HTIM_C_Phase, BLDC_Handle_->HDriver_.CL,BLDC_Handle_->pwm.c);
}

float	T1 = 0;



void SVPWMcalc(BLDC_Handle* BLDC_Handle_)
{
	
	uint8_t sec;
	if((BLDC_Handle_->Angle_phase>=0) && (BLDC_Handle_->Angle_phase<60))		sec=1;
	if((BLDC_Handle_->Angle_phase>=60) && (BLDC_Handle_->Angle_phase<120))	sec=2;
	if((BLDC_Handle_->Angle_phase>=120) && (BLDC_Handle_->Angle_phase<180))	sec=3;
	if((BLDC_Handle_->Angle_phase>=180) && (BLDC_Handle_->Angle_phase<240))	sec=4;
	if((BLDC_Handle_->Angle_phase>=240) && (BLDC_Handle_->Angle_phase<300))	sec=5;
	if((BLDC_Handle_->Angle_phase>=300) && (BLDC_Handle_->Angle_phase<360))	sec=6;


			T1= 0.866f *(float)(BLDC_Handle_->HDriver_.PWM_period)* BLDC_Handle_->m_vector * sinf(radConv(sec*60.0 - BLDC_Handle_->Angle_phase));
	uint16_t	T2= 0.866f *BLDC_Handle_->HDriver_.PWM_period* BLDC_Handle_->m_vector * sinf(radConv(BLDC_Handle_->Angle_phase - (sec-1)*60.0));
	uint16_t	T0=(BLDC_Handle_->HDriver_.PWM_period - (T1+T2))/2;
	uint16_t	T7=T0;
	
	switch (sec)
		{
			case 1: 
				{
					BLDC_Handle_->pwm.a = (BLDC_Handle_->HDriver_.PWM_period-T0);
					BLDC_Handle_->pwm.b = (T7+T2);
					BLDC_Handle_->pwm.c = (T7);
					break;
				}
			case 2:
				{
					BLDC_Handle_->pwm.b = (BLDC_Handle_->HDriver_.PWM_period-T0);
					BLDC_Handle_->pwm.a = (T7+T1);
					BLDC_Handle_->pwm.c = (T7);
					break;
				}
			case 3:
				{
					BLDC_Handle_->pwm.b = (BLDC_Handle_->HDriver_.PWM_period-T0);
					BLDC_Handle_->pwm.c = (T7+T2);
					BLDC_Handle_->pwm.a = (T7);
					break;
				}
			case 4:
				{
					BLDC_Handle_->pwm.c = (BLDC_Handle_->HDriver_.PWM_period-T0);
					BLDC_Handle_->pwm.b = (T7+T1);
					BLDC_Handle_->pwm.a = (T7);
					break;
				}
			case 5:
				{
					BLDC_Handle_->pwm.c = (BLDC_Handle_->HDriver_.PWM_period-T0);
					BLDC_Handle_->pwm.a = (T7+T2);
					BLDC_Handle_->pwm.b = (T7);
					break;
				}
			case 6:
				{
					BLDC_Handle_->pwm.a = (BLDC_Handle_->HDriver_.PWM_period-T0);
					BLDC_Handle_->pwm.c = (T7+T1);
					BLDC_Handle_->pwm.b = (T7);
					break;
				}	
		}
		
	DoPwm(BLDC_Handle_);
}


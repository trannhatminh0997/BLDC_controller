
#ifndef __BLDC_CONTROL_
#define __BLDC_CONTROL_

#include "AS5047D.h"
#include <stdint.h>
#include "flash.h"
#include <math.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "pid.h"

#define pi 3.14159265359
#define sin60 0.86602540378
#define cos60 0.5
#define radConv(deg) (((deg)/180.0)*pi)
#define degConv(rad) (((rad)/pi)*180.0)
#define CW 1
#define CCW 0

#define maxNumOfPole 20




typedef struct
{
	float baseAngle;
	uint8_t numOfPoles;
}Poles;



typedef struct 
{
	uint16_t a ;
	uint16_t b ;
	uint16_t c ;
}vec3;

typedef struct 
{
	TIM_HandleTypeDef *HTIM_AB_Phase,*HTIM_C_Phase;
	uint32_t AH,AL,BH,BL,CH,CL;
	uint32_t PWM_period;
	GPIO_TypeDef* Enable_Port;
	uint16_t Enable_Pin;
	vec3 pwm;
	vec3 phaseEnable;
}HDriver;

typedef struct
{
	volatile float Angle_phase;
	volatile float m_vector;
	volatile uint16_t dir;
	
	HDriver HDriver_;
	
	Poles Poles_;
	float interstitialAgle;
	float preMagAngleRaw;
	float magAngleRaw;
	AS5047D_Handler *AS5047D_Handler_;
	float Cur;
	
	PID_Handle pid;
	
	
}BLDC_Handler;

void SaveParams(uint32_t WriteStartAddr, uint32_t dataAddr, uint32_t length);
void LoadParams(uint32_t StartAddrToReadFlash, uint32_t dataAddr, uint32_t length);
void DriverStart(HDriver * HDriver_);
void DriverStop(HDriver * HDriver_);
void DriverInit(HDriver * HDriver_);
void DriverEnableAllPhase(HDriver * HDriver_);
void DriverDisableAllPhase(HDriver * HDriver_);
void DriverDoPwm(HDriver * HDriver_);
void SVPWMcalc(BLDC_Handler* BLDC_Handler_);
void Calibration(BLDC_Handler* BLDC_Handler_);
void Rotate(BLDC_Handler* BLDC_Handler_);



#endif

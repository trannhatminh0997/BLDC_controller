#include "BLDC_Control.h"
#include <stdint.h>

const float SinTable[60] = 
{ 0.0000000,	0.0174524,	0.0348995,	0.0523360,	0.0697565,	0.0871557,	0.1045285,
	0.1218693,	0.1391731,	0.1564345,	0.1736482,	0.1908090,	0.2079117,	0.2249511,
	0.2419219,	0.2588190,	0.2756374,	0.2923717,	0.3090170,	0.3255682,	0.3420201,
	0.3583679,	0.3746066,	0.3907311,	0.4067366,	0.4226183,	0.4383711,	0.4539905,
	0.4694716,	0.4848096,	0.5000000,	0.5150381,	0.5299193,	0.5446390,	0.5591929,
	0.5735764,	0.5877853,	0.6018150,	0.6156615,	0.6293204,	0.6427876,	0.6560590,
	0.6691306,	0.6819984,	0.6946584,	0.7071068,	0.7193398,	0.7313537,	0.7431448,
	0.7547096,	0.7660444,	0.7771460,	0.7880108,	0.7986355,	0.8090170,	0.8191520,
	0.8290376,	0.8386706,	0.8480481,	0.8571673 };
const uint16_t DeadTime = 0; //based on period value

float sinSector(float angle)
{
	if(angle<0) return (0);
	if(angle >= 60) return (0.8660254);
	return(SinTable[(uint16_t)angle]);
}
	

bool IsApprox(float param1, float param2, float offset)
{
	float er = param2 - param1;
	er = (er>=0)? er:-er;
	return (er < offset)? true:false;
}

void DriverStart(HDriver * HDriver_)
{
	HAL_TIM_PWM_Start(HDriver_->HTIM_AB_Phase,HDriver_->AH);
	HAL_TIM_PWM_Start(HDriver_->HTIM_AB_Phase,HDriver_->AL);
	HAL_TIM_PWM_Start(HDriver_->HTIM_AB_Phase,HDriver_->BH);
	HAL_TIM_PWM_Start(HDriver_->HTIM_AB_Phase,HDriver_->BL);
	HAL_TIM_PWM_Start(HDriver_->HTIM_C_Phase,HDriver_->CH);
	HAL_TIM_PWM_Start(HDriver_->HTIM_C_Phase,HDriver_->CL);
	HAL_GPIO_WritePin(HDriver_->Enable_Port,HDriver_->Enable_Pin,GPIO_PIN_SET);
}
void DriverStop(HDriver * HDriver_)
{
	HAL_TIM_PWM_Stop(HDriver_->HTIM_AB_Phase,HDriver_->AH);
	HAL_TIM_PWM_Stop(HDriver_->HTIM_AB_Phase,HDriver_->AL);
	HAL_TIM_PWM_Stop(HDriver_->HTIM_AB_Phase,HDriver_->BH);
	HAL_TIM_PWM_Stop(HDriver_->HTIM_AB_Phase,HDriver_->BL);
	HAL_TIM_PWM_Stop(HDriver_->HTIM_C_Phase,HDriver_->CH);
	HAL_TIM_PWM_Stop(HDriver_->HTIM_C_Phase,HDriver_->CL);
	HAL_GPIO_WritePin(HDriver_->Enable_Port,HDriver_->Enable_Pin,GPIO_PIN_RESET);
}

void DriverInit(HDriver * HDriver_)
{
	HDriver_->PWM_period = HDriver_->HTIM_AB_Phase->Init.Period;
	DriverStart(HDriver_);
}
void DriverEnableAllPhase(HDriver * HDriver_)
{
	HDriver_->phaseEnable.a = 1;
	HDriver_->phaseEnable.b = 1;
	HDriver_->phaseEnable.c = 1;
}
void DriverDisableAllPhase(HDriver * HDriver_)
{
	HDriver_->phaseEnable.a = 0;
	HDriver_->phaseEnable.b = 0;
	HDriver_->phaseEnable.c = 0;
}
void DriverDoPwm(HDriver * HDriver_)
{
	if(HDriver_->phaseEnable.a){
		__HAL_TIM_SET_COMPARE(HDriver_->HTIM_AB_Phase,HDriver_->AH,HDriver_->pwm.a-DeadTime);
		__HAL_TIM_SET_COMPARE(HDriver_->HTIM_AB_Phase,HDriver_->AL,HDriver_->pwm.a);}
	else{
		__HAL_TIM_SET_COMPARE(HDriver_->HTIM_AB_Phase,HDriver_->AH,0);
		__HAL_TIM_SET_COMPARE(HDriver_->HTIM_AB_Phase,HDriver_->AL,HDriver_->PWM_period);}
	//------------------------------------------------------------------------------------------------------------------------------//
	if(HDriver_->phaseEnable.b){
		__HAL_TIM_SET_COMPARE(HDriver_->HTIM_AB_Phase,HDriver_->BH,HDriver_->pwm.b-DeadTime);
		__HAL_TIM_SET_COMPARE(HDriver_->HTIM_AB_Phase,HDriver_->BL,HDriver_->pwm.b);}
	else{
		__HAL_TIM_SET_COMPARE(HDriver_->HTIM_AB_Phase,HDriver_->BH,0);
		__HAL_TIM_SET_COMPARE(HDriver_->HTIM_AB_Phase,HDriver_->BL,HDriver_->PWM_period);}
	//------------------------------------------------------------------------------------------------------------------------------//
	if(HDriver_->phaseEnable.c){
		__HAL_TIM_SET_COMPARE(HDriver_->HTIM_C_Phase, HDriver_->CH,HDriver_->pwm.c-DeadTime);
		__HAL_TIM_SET_COMPARE(HDriver_->HTIM_C_Phase, HDriver_->CL,HDriver_->pwm.c);}
	else{
		__HAL_TIM_SET_COMPARE(HDriver_->HTIM_C_Phase, HDriver_->CH,0);
		__HAL_TIM_SET_COMPARE(HDriver_->HTIM_C_Phase, HDriver_->CL,HDriver_->PWM_period);}
	
}

void SVPWMcalc_V1(BLDC_Handler* BLDC_Handler_)
{
	
	uint8_t sec;
	if((BLDC_Handler_->Angle_phase>=0) && (BLDC_Handler_->Angle_phase<60))		sec=1;
	if((BLDC_Handler_->Angle_phase>=60) && (BLDC_Handler_->Angle_phase<120))	sec=2;
	if((BLDC_Handler_->Angle_phase>=120) && (BLDC_Handler_->Angle_phase<180))	sec=3;
	if((BLDC_Handler_->Angle_phase>=180) && (BLDC_Handler_->Angle_phase<240))	sec=4;
	if((BLDC_Handler_->Angle_phase>=240) && (BLDC_Handler_->Angle_phase<300))	sec=5;
	if((BLDC_Handler_->Angle_phase>=300) && (BLDC_Handler_->Angle_phase<360))	sec=6;


	uint16_t	T1,T2,T0,T7;
	float m = BLDC_Handler_->m_vector;
	m = ((m<0)?-m:m);
	
	T1= BLDC_Handler_->HDriver_.PWM_period* m * sinSector(sec*60.0 - BLDC_Handler_->Angle_phase);
	T2= BLDC_Handler_->HDriver_.PWM_period* m * sinSector(BLDC_Handler_->Angle_phase - (sec-1)*60.0);
	T0=(BLDC_Handler_->HDriver_.PWM_period - (T1+T2))/2;
	T7=T0;
	
	switch (sec)
		{
			case 1: 																																							//100->110
				{
					BLDC_Handler_->HDriver_.pwm.a = (BLDC_Handler_->HDriver_.PWM_period-T0);										//1
					BLDC_Handler_->HDriver_.pwm.b = (T7+T2);																										//0 ->1
					BLDC_Handler_->HDriver_.pwm.c = (T7);																											//0
					break;
				}
			case 2:																																								//110->010
				{
					BLDC_Handler_->HDriver_.pwm.a = (T7+T1);																										//1->0
					BLDC_Handler_->HDriver_.pwm.b = (BLDC_Handler_->HDriver_.PWM_period-T0);										//1
					BLDC_Handler_->HDriver_.pwm.c = (T7);																											//0
					break;
				}
			case 3:																																								//010->011
				{
					BLDC_Handler_->HDriver_.pwm.a = (T7);																											//0
					BLDC_Handler_->HDriver_.pwm.b = (BLDC_Handler_->HDriver_.PWM_period-T0);										//1
					BLDC_Handler_->HDriver_.pwm.c = (T7+T2);																										//0->1	
					
					break;
				}
			case 4:																																								//011->001
				{																				
					BLDC_Handler_->HDriver_.pwm.a = (T7);																											//0
					BLDC_Handler_->HDriver_.pwm.b = (T7+T1);																										//1->0
					BLDC_Handler_->HDriver_.pwm.c = (BLDC_Handler_->HDriver_.PWM_period-T0);										//1
					break;
				}
			case 5:																																								//001->101
				{
					BLDC_Handler_->HDriver_.pwm.a = (T7+T2);																										//0->1
					BLDC_Handler_->HDriver_.pwm.b = (T7);																											//0
					BLDC_Handler_->HDriver_.pwm.c = (BLDC_Handler_->HDriver_.PWM_period-T0);										//1
					break;
				}
			case 6:																																								//101->100
				{
					BLDC_Handler_->HDriver_.pwm.a = (BLDC_Handler_->HDriver_.PWM_period-T0);										//1
					BLDC_Handler_->HDriver_.pwm.b = (T7);																											//0
					BLDC_Handler_->HDriver_.pwm.c = (T7+T1);																										//1->0
					break;
				}	
		}
		
	DriverDoPwm(&BLDC_Handler_->HDriver_);
}


void SVPWMcalc(BLDC_Handler* BLDC_Handler_)
{
	
	uint8_t sec;
	if((BLDC_Handler_->Angle_phase>=0) && (BLDC_Handler_->Angle_phase<60))		sec=1;
	if((BLDC_Handler_->Angle_phase>=60) && (BLDC_Handler_->Angle_phase<120))	sec=2;
	if((BLDC_Handler_->Angle_phase>=120) && (BLDC_Handler_->Angle_phase<180))	sec=3;
	if((BLDC_Handler_->Angle_phase>=180) && (BLDC_Handler_->Angle_phase<240))	sec=4;
	if((BLDC_Handler_->Angle_phase>=240) && (BLDC_Handler_->Angle_phase<300))	sec=5;
	if((BLDC_Handler_->Angle_phase>=300) && (BLDC_Handler_->Angle_phase<360))	sec=6;



	int m = abs((int)(BLDC_Handler_->m_vector*BLDC_Handler_->HDriver_.PWM_period));
	
	
	DriverEnableAllPhase(&BLDC_Handler_->HDriver_);
	switch (sec)
		{
			case 1: 																																							
				{
					BLDC_Handler_->HDriver_.pwm.a = m;																									
					BLDC_Handler_->HDriver_.pwm.b = 0;																									
					BLDC_Handler_->HDriver_.pwm.c = m;		
					BLDC_Handler_->HDriver_.phaseEnable.c = 0;																	//10x
					break;
				}
			case 6:																																								
				{
					BLDC_Handler_->HDriver_.pwm.a = 0;																									
					BLDC_Handler_->HDriver_.pwm.b = 0;																									
					BLDC_Handler_->HDriver_.pwm.c = m;																									
					BLDC_Handler_->HDriver_.phaseEnable.a = 0;																	//x01					
					break;
				}
			case 5:																																								
				{
					BLDC_Handler_->HDriver_.pwm.a = 0;																									//0
					BLDC_Handler_->HDriver_.pwm.b = m;																											
					BLDC_Handler_->HDriver_.pwm.c = m;																									//1	
					BLDC_Handler_->HDriver_.phaseEnable.b = 0;																	//0x1						
					
					break;
				}
			case 4:																																								
				{																				
					BLDC_Handler_->HDriver_.pwm.a = 0;																									
					BLDC_Handler_->HDriver_.pwm.b = m;																									
					BLDC_Handler_->HDriver_.pwm.c = 0;																										
					BLDC_Handler_->HDriver_.phaseEnable.c = 0;																	//01x
					break;
				}
			case 3:																																								
				{
					BLDC_Handler_->HDriver_.pwm.a = m;																									
					BLDC_Handler_->HDriver_.pwm.b = m;																									
					BLDC_Handler_->HDriver_.pwm.c = 0;																									
					BLDC_Handler_->HDriver_.phaseEnable.a = 0;																	//x10					
					break;
				}
			case 2:																																								
				{
					BLDC_Handler_->HDriver_.pwm.a = m;																									//1
					BLDC_Handler_->HDriver_.pwm.b = 0;																									
					BLDC_Handler_->HDriver_.pwm.c = 0;																									//0
					BLDC_Handler_->HDriver_.phaseEnable.b = 0;																	//1x0					
					break;
				}	
		}
		
	DriverDoPwm(&BLDC_Handler_->HDriver_);
}




void SaveParams(uint32_t WriteStartAddr, uint32_t dataAddr, uint32_t length)
{
	Flash_Write_Obj(WriteStartAddr, dataAddr, length);
}
void LoadParams(uint32_t StartAddrToReadFlash, uint32_t dataAddr, uint32_t length)
{
	Flash_Read_Obj(StartAddrToReadFlash, dataAddr, length);
}

void Calibration(BLDC_Handler* BLDC_Handler_)
{
	bool isPolesBaseAngleTaken = false;
	BLDC_Handler_->Poles_.numOfPoles = 0;
	BLDC_Handler_->Angle_phase = 0;
	BLDC_Handler_->m_vector =0.05;
	HAL_Delay(3000);
	DriverStart(&BLDC_Handler_->HDriver_);
	BLDC_Handler_->preMagAngleRaw = AS5047D_Get_True_Angle_Value(BLDC_Handler_->AS5047D_Handler_);
	while(1)
	{
		SVPWMcalc(BLDC_Handler_);
		HAL_Delay(1);
		BLDC_Handler_->Angle_phase++;
		if (BLDC_Handler_->Angle_phase >= 360)
			{
				BLDC_Handler_->Angle_phase = 0;
				HAL_Delay(1000);
				BLDC_Handler_->magAngleRaw = AS5047D_Get_True_Angle_Value(BLDC_Handler_->AS5047D_Handler_);
				if(!IsApprox(BLDC_Handler_->magAngleRaw,BLDC_Handler_->preMagAngleRaw,3.0))
				{
					if(!isPolesBaseAngleTaken)
					{
						BLDC_Handler_->Poles_.baseAngle = BLDC_Handler_->magAngleRaw;
						isPolesBaseAngleTaken = true;
					}
					else if (!IsApprox(BLDC_Handler_->magAngleRaw,BLDC_Handler_->Poles_.baseAngle,3.0))
					{
						BLDC_Handler_->Poles_.numOfPoles++;
						if (BLDC_Handler_->Poles_.numOfPoles > maxNumOfPole){break;}
					}
					else
					{
						BLDC_Handler_->Poles_.numOfPoles++;
						break;
					}
				}
				else{}
			}	
	}
	DriverStop(&BLDC_Handler_->HDriver_);
	if(1)
	{
		SaveParams(AddrPage127,(uint32_t)(&BLDC_Handler_->Poles_),sizeof(BLDC_Handler_->Poles_));
	}
}


float dq = 0;
void Rotate(BLDC_Handler* BLDC_Handler_)
{
		BLDC_Handler_->magAngleRaw = AS5047D_Get_True_Angle_Value(BLDC_Handler_->AS5047D_Handler_);
		float d = ((BLDC_Handler_->magAngleRaw - BLDC_Handler_->Poles_.baseAngle)/BLDC_Handler_->interstitialAgle);
		d = (d-(int32_t)d)*360.0;
		d = d + ((d<0)? 360.0:0);
		
		BLDC_Handler_->dir = ((BLDC_Handler_->m_vector >= 0)?1:0);
		float q = d + ((BLDC_Handler_->dir==1)?-90:90);
		q = q + ((q<0)? 360:0) - ((q>=360)?360:0);
		BLDC_Handler_->Angle_phase = q;
		SVPWMcalc(BLDC_Handler_);
}







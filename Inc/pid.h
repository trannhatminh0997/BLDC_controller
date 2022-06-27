
#ifndef __PID_
#define __PID_
#include "stm32f1xx_hal.h"
#include <stdbool.h>



typedef struct{
	float	Error, pre_Error,pre_pre_Error,
				pre_Out,																							//
				P_part,																								//
				I_part,																								//		
				D_part;																								//
}PID_ProcessPart;
typedef struct{
	float	kp,																								//
				ki,																							//
				kd;																							//
	uint16_t SampleTime;							
	float outMin, outMax;	
	bool IsFbPositive;  																	// mean feedback and output in the same direction
}PID_Params;

typedef struct{
	
	volatile	float * setPoint,*feedback,*Output;
	PID_Params Params;
	PID_ProcessPart ProcessPart;
}PID_Handle;

void PID_Compute( PID_Handle* PID_Handle_);



#endif



#include "pid.h"
#include <stdint.h>
void PID_Compute( PID_Handle* PID_Handle_)
{
			/*Compute all the working error variables*/
		
		PID_Handle_->ProcessPart.Error =(int16_t)( *PID_Handle_->setPoint - *PID_Handle_->feedback);
	
	  PID_Handle_->ProcessPart.Error += ((PID_Handle_->ProcessPart.Error > 180)?-360.0:((PID_Handle_->ProcessPart.Error < -180)?360.0:0));
	
		PID_Handle_->ProcessPart.P_part = PID_Handle_->Params.kp * (PID_Handle_->ProcessPart.Error-PID_Handle_->ProcessPart.pre_Error );
		PID_Handle_->ProcessPart.I_part = 0.5 * PID_Handle_->Params.ki * PID_Handle_->Params.SampleTime * (PID_Handle_->ProcessPart.Error + PID_Handle_->ProcessPart.pre_Error);
		if(PID_Handle_->ProcessPart.I_part > PID_Handle_->Params.outMax) PID_Handle_->ProcessPart.I_part = PID_Handle_->Params.outMax;
		else if(PID_Handle_->ProcessPart.I_part < PID_Handle_->Params.outMin) PID_Handle_->ProcessPart.I_part = PID_Handle_->Params.outMin;
		PID_Handle_->ProcessPart.D_part = PID_Handle_->Params.kd/PID_Handle_->Params.SampleTime*(PID_Handle_->ProcessPart.Error - 2 * PID_Handle_->ProcessPart.pre_Error + PID_Handle_->ProcessPart.pre_pre_Error);
		
		*PID_Handle_->Output = PID_Handle_->ProcessPart.pre_Out + PID_Handle_->ProcessPart.P_part + PID_Handle_->ProcessPart.I_part + PID_Handle_->ProcessPart.D_part ;
		if (*PID_Handle_->Output > PID_Handle_->Params.outMax) *PID_Handle_->Output = PID_Handle_->Params.outMax;
		else if (*PID_Handle_->Output < PID_Handle_->Params.outMin) *PID_Handle_->Output = PID_Handle_->Params.outMin;
		PID_Handle_->ProcessPart.pre_pre_Error = PID_Handle_->ProcessPart.pre_Error;
		PID_Handle_->ProcessPart.pre_Error = PID_Handle_->ProcessPart.Error;
		PID_Handle_->ProcessPart.pre_Out = *PID_Handle_->Output;
		if(!PID_Handle_->Params.IsFbPositive) *PID_Handle_->Output = -*PID_Handle_->Output ;
		*PID_Handle_->Output /=100.0;
}

#include "pid.h"


//pid初始化
void pidInit(PidObject* pid, const float desired, const float kp,const float ki, const float kd, const float dt)
{
	pid->error     = 0;
	pid->prevError = 0;
	pid->integ     = 0;
	pid->deriv     = 0;
	pid->desired	 = desired;
	pid->kp 			 = kp;
	pid->ki 			 = ki;
	pid->kd 			 = kd;
	pid->iLimit    = 0;
	pid->deadband  = 0;
	pid->dt        = dt;
}
//pid参数设置
void pidParameterSet(PidObject* pid,float kp,float ki,float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

//pid参数通用修改
void pidSetParameter(PidObject* pid ,PidParameter para ,float value)
{
	switch(para)
	{
		case kp : 				pid->kp = value;break;
		case ki : 				pid->ki = value;break;
		case kd : 				pid->kd = value;break;
		case iLimit : 		pid->iLimit = value;break;
		case deadband : 	pid->deadband = value;break;
		case dt : 				pid->dt = value;break;
		case desired : 		pid->desired = value;break;
	}
		
}

//pid计算
float pidUpdate(PidObject* pid, const float measured)//
{
	float output;
	//error
	pid->error = pid->desired - measured;
	if(pid->error < pid->deadband && pid->error > -pid->deadband) pid->error = 0;
	
	//i
	pid->integ += pid->error * pid->dt;
	if (pid->integ > pid->iLimit) 				pid->integ =  pid->iLimit;
	else if (pid->integ < -pid->iLimit) 	pid->integ = -pid->iLimit;
	
	//d
	pid->deriv = (pid->error - pid->prevError) / pid->dt;

	pid->outP = pid->kp * pid->error;
	pid->outI = pid->ki * pid->integ;
	pid->outD = pid->kd * pid->deriv;
	output = pid->outP + pid->outI + pid->outD;
	
	pid->prevError = pid->error;

	return output;
}

//清除积分
void pidClearIntegral(PidObject *pid)
{
	pid->integ=0;
	pid->outI=0;
}










////增量式PID
//float incPidUpdate(IncPidObject* pid, const float measured)
//{
//	float output;
//	pid->curError=pid->desired-measured;
//	output=pid->Kp*pid->curError-pid->Ki*pid->lastError+pid->Kd*pid->prevError;
//	pid->prevError=pid->lastError;
//	pid->lastError=pid->curError;
//	return output;
//}
//void incPidInit(IncPidObject *pid,float Kp,float Ki,float Kd,float desired)
//{
//	pid->Kp=Kp;
//	pid->Ki=Ki;
//	pid->Kd=Kd;
//	pid->desired=desired;
//}
//void incPidSetDesired(IncPidObject *pid,float desired)
//{
//	pid->desired=desired;
//}

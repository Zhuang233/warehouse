#ifndef __PID_H
#define __PID_H


typedef struct
{
	float desired;      //< set point
	float error;        //< error
	float prevError;    //< previous error
	float integ;        //< integral
	float deriv;        //< derivative
	float kp;           //< proportional gain
	float ki;           //< integral gain
	float kd;           //< derivative gain
	float outP;         //< proportional output (debugging)
	float outI;         //< integral output (debugging)
	float outD;         //< derivative output (debugging)
	float iLimit;       //< integral limit
	float deadband;			//< deadband
	float dt;           //< delta-time dt
} PidObject;

typedef enum
{	
	kp=0,
	ki=1,
	kd=2,
	iLimit=3,
	deadband=4,	
	dt=5,
	desired=6,
}PidParameter;



typedef struct
{
	float desired;
	float curError;
	float Kp;
	float Ki;
	float Kd;
	float lastError;
	float prevError;
} IncPidObject;

void pidInit(PidObject* pid, const float desired, const float kp,const float ki, const float kd, const float dt);
void pidSetIntegLimit(PidObject* pid, const float limit, const float limitLow);
float pidUpdate(PidObject* pid, const float measured);
void pidSetParameter(PidObject* pid ,PidParameter para ,float value);
void pidClearIntegral(PidObject *pid);
void pidParameterSet(PidObject* pid,const float kp,const float ki, const float kd);

//void incPidSetDesired(IncPidObject *pid,float desired);
//void incPidInit(IncPidObject *pid,float Kp,float Ki,float Kd,float desired);
//float incPidUpdate(IncPidObject* pid, const float measured);
#endif

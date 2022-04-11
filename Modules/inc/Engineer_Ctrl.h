#ifndef __INFANTRY_CTRL_
#define __INFANTRY_CTRL_

#include "stm32f4xx_hal.h"
#include "pid.h"

//extern uint8_t PITCHRATECTRL;
//extern uint8_t YAWATECTRL;

//extern PidObject pid_pitch;//holder pitch control.
//extern PidObject pid_pitchRate;//holder pitch rate control.
//extern PidObject pid_yaw;//holder yaw control.
//extern PidObject pid_yawRate;//holder yaw rate control.
//extern PidObject pid_friction_1,pid_friction_2;
extern PidObject pid_chassis_gyro;

extern PidObject pid_testmoto_spd[2];
extern PidObject pid_testmoto_pos[2];
extern PidObject pid_sync[2];;
//extern PidObject pid_chassis_spd[6];
//extern float yawRateDesired;
//extern float pitchRateDesired;

void SingleMotoTest(int32_t left,int32_t right);
void Testmoto_PID_Init(void);
void Engineer_Pid_Init(void);
void Zeropoint_Init(void);
//void Chassis_Control(void);
void Engineer_Control(void);
void SteerengineControl(void);
void int16_constraint(int16_t *data,int16_t max,int16_t min);
void float_constraint(float *data,float max,float min);

#endif 

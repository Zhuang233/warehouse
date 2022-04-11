#include "motor.h"
#include "can.h"
#include "FreeRTOS.h"
#include "Remotecontrol.h"
#include "task.h"
#include "timer.h"
#include "stdlib.h"
#include "pid.h"
#include "arm_math.h"




Motor_Msg motor_msg[7];
                   /* [0]~[3]  chassis	
                      [4]~[5]  lift
                      [6]      pan
                   */


void Motor_Msg_Init(void)
{
	uint8_t i;
	for(i=0; i<7; i++)
	{
		motor_msg[i].speed_actual=0;
		motor_msg[i].speed_last=0;
		motor_msg[i].speed_desired=0;
		motor_msg[i].angle_actual=0;
		motor_msg[i].angle_desired=0;
		motor_msg[i].angle_last=0;
		motor_msg[i].angle=0;
		motor_msg[i].given_current=0;
		motor_msg[i].real_current=0;
		motor_msg[i].turns=0;
		motor_msg[i].original_position=0;
    motor_msg[i].first_run = true;
		motor_msg[i].whe_use_pid=true;
		motor_msg[i].time.time_now=TIM2->CNT;
		motor_msg[i].time.time_last=motor_msg[i].time.time_now;
		motor_msg[i].time.dt=0;
	}
}

void Stop_Motor(Motor_Msg *msg)
{
	msg->speed_desired = 0;
	msg->given_current = 0;
}

void Stop_AllMotor(void)
{
	uint8_t i;
	for(i=0; i<7; i++)
	{
		motor_msg[i].given_current=0;
    motor_msg[i].speed_desired=0;
	}
}

void Set_MotorZeropoint(Motor_Msg *msg)
{
  msg->first_run = true;
}

void SingleMotoTest(int32_t left,int32_t right)  //left- back right+ back
{
  motor_msg[0].angle_desired = left;
  motor_msg[1].angle_desired = right;
}

#ifndef __MOTOR_H
#define __MOTOR_H
#include "stdint.h"
#include "stdbool.h"
#include "timer.h"
#include "pid.h"

typedef struct 
{
	int16_t		speed_actual;
	int16_t 	speed_last;
	int16_t 	speed_desired;
	int16_t 	real_current;
	int16_t 	given_current;
	int32_t  	angle_desired;
	uint16_t 	angle_actual;
	uint16_t	angle_vague;
	uint16_t 	angle_last;
	int32_t		angle;
	int16_t 	turns;
	int8_t		temperature;
	int16_t		original_position;
  bool      first_run;
	bool 		whe_use_pid;
	TIME 		time;
}Motor_Msg;

extern Motor_Msg motor_msg[7];



void Motor_Msg_Init(void);
void Set_MotorZeropoint(Motor_Msg *msg);
void Stop_Motor(Motor_Msg *msg);
void Stop_AllMotor(void);


#define MOTOR_START		0
#define MOTOR_STOP		1

#define STEE1_BASE			2400
#define STEE2_BASE			2400
#define STEE3_BASE			2400
#define STEE4_BASE			2400

void Motor_Init(void);
void Stop_Motor(Motor_Msg *msg);
void Stop_AllMotor(void);
#endif

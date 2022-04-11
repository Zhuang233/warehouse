#include "servo.h"


#define SOFT_PARA 0.1f

#define ARM_DEFAULT_ANGLE 		30
#define TOP_DEFAULT_ANGLE			50
#define YINDAO_DEFAULT_ANGLE	160
#define CLIP_DEFAULT_ANGLE		80
#define BO_DEFAULT_ANGLE			46
#define MOVE_DEFAULT_ANGLE		280


#define ARM_MAX_ANGLE			180
#define ARM_MIN_ANGLE			20
#define TOP_MAX_ANGLE			300
#define TOP_MIN_ANGLE			50
#define YINDAO_MAX_ANGLE	160
#define YINDAO_MIN_ANGLE	1
#define CLIP_MAX_ANGLE		155
#define CLIP_MIN_ANGLE		80
#define BO_MAX_ANGLE			142
#define BO_MIN_ANGLE			0
#define MOVE_MAX_ANGLE		280
#define MOVE_MIN_ANGLE		40


servos_t servos;
servos_t servos_soft;
uint8_t servoMode = hard_mode;


void servos_init()
{
	servos.arm 		= ARM_DEFAULT_ANGLE;
	servos.bo  		= BO_DEFAULT_ANGLE;
	servos.clip 	= CLIP_DEFAULT_ANGLE;
	servos.move 	= MOVE_DEFAULT_ANGLE;
	servos.top 		= TOP_DEFAULT_ANGLE;
	servos.yindao = YINDAO_DEFAULT_ANGLE;
}


void servos_control_loop()
{
	if(servoMode == soft_mode) hardFollowSoft();
	servos_angle_limit();
	set_servo_angle(servos.arm,servo_arm_180_25);
	set_servo_angle(servos.bo,servo_bo_180_25);
	set_servo_angle(servos.clip,servo_clip_180_25);
	set_servo_angle(servos.move,servo_move_270_15);
	set_servo_angle(servos.top,servo_top_360_25);
	set_servo_angle(servos.yindao,servo_yindao_180_15);
	
}

void servos_angle_limit()
{
	if(servos.arm 	> ARM_MAX_ANGLE) 		servos.arm 		= ARM_MAX_ANGLE;
	if(servos.arm 	< ARM_MIN_ANGLE) 		servos.arm 		= ARM_MIN_ANGLE;
	if(servos.bo 		> BO_MAX_ANGLE) 		servos.bo 		= BO_MAX_ANGLE;
	if(servos.bo 		< BO_MIN_ANGLE) 		servos.bo 		= BO_MIN_ANGLE;
	if(servos.clip 	> CLIP_MAX_ANGLE) 	servos.clip 	= CLIP_MAX_ANGLE;
	if(servos.clip 	< CLIP_MIN_ANGLE) 	servos.clip 	= CLIP_MIN_ANGLE;
	if(servos.move 	> MOVE_MAX_ANGLE) 	servos.move 	= MOVE_MAX_ANGLE;
	if(servos.move	< MOVE_MIN_ANGLE) 	servos.move 	= MOVE_MIN_ANGLE;
	if(servos.top 	> TOP_MAX_ANGLE) 		servos.top 		= TOP_MAX_ANGLE;
	if(servos.top 	< TOP_MIN_ANGLE) 		servos.top 		= TOP_MIN_ANGLE;
	if(servos.yindao> YINDAO_MAX_ANGLE) servos.yindao = YINDAO_MAX_ANGLE;
	if(servos.yindao< YINDAO_MIN_ANGLE) servos.yindao = YINDAO_MIN_ANGLE;
}


void changeServoModeSoft()
{
	servos_soft = servos;
	servoMode = soft_mode;
}

void changeServoModeHard()
{
	servoMode = hard_mode;
}

void hardFollowSoft()
{
	if(servos_soft.arm 		> servos.arm) 		servos.arm 		+= SOFT_PARA;
	if(servos_soft.arm 		< servos.arm) 		servos.arm 		-= SOFT_PARA;
	if(servos_soft.bo 		> servos.bo) 			servos.bo 		+= SOFT_PARA;
	if(servos_soft.bo 		< servos.bo) 			servos.bo 		-= SOFT_PARA;
	if(servos_soft.clip 	> servos.clip)		servos.clip 	+= SOFT_PARA;
	if(servos_soft.clip 	< servos.clip) 		servos.clip 	-= SOFT_PARA;
	if(servos_soft.move 	> servos.move) 		servos.move 	+= SOFT_PARA;
	if(servos_soft.move 	< servos.move) 		servos.move 	-= SOFT_PARA;
	if(servos_soft.top 		> servos.top) 		servos.top 		+= SOFT_PARA;
	if(servos_soft.top 		< servos.top) 		servos.top 		-= SOFT_PARA;
	if(servos_soft.yindao > servos.yindao) 	servos.yindao += SOFT_PARA;
	if(servos_soft.yindao < servos.yindao) 	servos.yindao -= SOFT_PARA;
}


//���ö���ĽǶȣ������������ţ�Ŀ��Ƕ�
void set_servo_angle(uint16_t angle,uint8_t id)
{	
	float rate;//ռ�ձȷ�Χ�Ͷ���Ƕȷ�Χ�ı���
	uint16_t temp;
	
	switch(id)
	{
		case servo_top_360_25: 		rate = 5.555;	break;
		case servo_move_270_15: 	rate = 7.407;	break;
		case servo_yindao_180_15: rate = 11.11;	break;
		case servo_arm_180_25: 		rate = 11.11;	break;
		case servo_clip_180_25: 	rate = 11.11;	break;
		case servo_bo_180_25: 		rate = 11.11;	break;
	}

	temp = 19500-rate*angle;
	
	switch(id)
	{
		case servo_top_360_25 : 	TIM_SetTIM2Compare2(temp);	break;
		case servo_move_270_15:		TIM_SetTIM2Compare1(temp);	break;
		case servo_yindao_180_15: TIM_SetTIM2Compare3(temp);	break;
		case servo_arm_180_25: 		TIM_SetTIM9Compare1(temp);	break;
		case servo_clip_180_25: 	TIM_SetTIM9Compare2(temp);	break;
		case servo_bo_180_25: 		TIM_SetTIM11Compare1(temp);	break;
	}
}

//����TIM2ͨ����ռ�ձ�
//compare:�Ƚ�ֵ
void TIM_SetTIM2Compare1(uint32_t compare)
{
	TIM2->CCR1=compare; 
}

void TIM_SetTIM2Compare2(uint32_t compare)
{
	TIM2->CCR2=compare; 
}

void TIM_SetTIM2Compare3(uint32_t compare)
{
	TIM2->CCR3=compare; 
}

//����TIM9ͨ����ռ�ձ�
//compare:�Ƚ�ֵ
void TIM_SetTIM9Compare1(uint32_t compare)
{
	TIM9->CCR1=compare; 
}

void TIM_SetTIM9Compare2(uint32_t compare)
{
	TIM9->CCR2=compare; 
}

void TIM_SetTIM11Compare1(uint32_t compare)
{
	TIM11->CCR1=compare;
}


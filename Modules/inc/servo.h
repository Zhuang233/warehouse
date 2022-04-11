#ifndef __servo_H
#define __servo_H
#include "stdint.h"
#include "stm32f4xx_hal.h"
enum servo
{
	servo_top_360_25=0,
	servo_move_270_15=1,
	servo_yindao_180_15=2,
	servo_arm_180_25=3,
	servo_clip_180_25=4,
	servo_bo_180_25=5
	
};

enum servo_mode
{
	hard_mode,
	soft_mode
};

typedef struct
{
	float top;
	float move;
	float yindao;
	float arm;
	float clip;
	float bo;
}servos_t;

extern servos_t servos;
extern servos_t servos_soft;

void servos_init(void);
void servos_control_loop(void);
void changeServoModeSoft(void);
void changeServoModeHard(void);

void servos_angle_limit(void);
void set_servo_angle(uint16_t angle,uint8_t id);
void hardFollowSoft(void);
void TIM_SetTIM2Compare1(uint32_t compare);
void TIM_SetTIM2Compare2(uint32_t compare);
void TIM_SetTIM2Compare3(uint32_t compare);
void TIM_SetTIM9Compare1(uint32_t compare);
void TIM_SetTIM9Compare2(uint32_t compare);
void TIM_SetTIM11Compare1(uint32_t compare);
void TIM_SetTIM13Compare1(uint32_t compare);
void TIM_SetTIM14Compare1(uint32_t compare);
#endif


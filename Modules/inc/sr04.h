#ifndef __SR04_H__
#define __SR04_H__
#include "stdint.h"
#include "stm32f4xx_hal.h"


extern uint8_t TIM5CH1_CAPTURE_STA;	//���벶��״̬		    				
extern uint32_t	TIM5CH1_CAPTURE_VAL;	//���벶��ֵ(TIM2/TIM5��32λ)
extern uint32_t	TIM5CH1_CAPTURE_VAL_start;
extern TIM_HandleTypeDef htim5;
void my_ic_update_callback(TIM_HandleTypeDef *htim);

double get_distance(uint8_t side);
void sr04_init(void);

#endif


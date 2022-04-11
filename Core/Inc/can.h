/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "motor.h"
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
extern bool CAN1_send_current_flag;
extern bool CAN2_send_current_flag;

typedef enum
{
	CAN_Motor_ALL_ID = 0x200,
	CAN_Motor1_ID = 0x201,
	CAN_Motor2_ID = 0x202,
	CAN_Motor3_ID = 0x203,
	CAN_Motor4_ID = 0x204,
	CAN_Motor5_ID = 0x205,
	CAN_Motor6_ID = 0x206,
	CAN_Motor7_ID = 0x207,
	CAN_Motor8_ID = 0x208,
}CAN_Message_ID;

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void Can_Filter_Init(void);
										 					
void Cal_Angle(Motor_Msg *motor);
void GetMotor_Message(Motor_Msg *motor,uint8_t *data);
void CAN2_Set_Current(void);
void CAN1_Set_Current(void);
void CAN1_Set_AheadCur(int16_t C1, int16_t C2, int16_t C3, int16_t C4);
void CAN1_Set_BackCur(int16_t C1, int16_t C2, int16_t C3, int16_t C4);
void CAN2_Set_AheadCur(int16_t C1, int16_t C2, int16_t C3, int16_t C4);
void CAN2_Set_BackCur(int16_t C1, int16_t C2, int16_t C3, int16_t C4);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */


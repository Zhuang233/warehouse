/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "main.h"
#include "motor.h"
#include <stdbool.h>


bool CAN1_send_current_flag = false;
bool CAN2_send_current_flag = false;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */







//calcuelate angle of motor
int16_t d_angle;
void Cal_Angle(Motor_Msg *motor)
{
	d_angle = motor->angle_actual - motor->angle_last;
	if(d_angle <= -4096) motor->turns += 1;
	else if(d_angle >= 4096) motor->turns -= 1;
	motor->angle = motor->turns * 8192 + motor->angle_actual - motor->original_position;
}

//decode message of motor
void GetMotor_Message(Motor_Msg *motor,uint8_t *data)
{
	motor->angle_last = motor->angle_actual;
	motor->angle_actual = (uint16_t)(data[0] << 8 | data[1]);
	motor->speed_last = motor->speed_actual;
	motor->speed_actual = (int16_t)(data[2] << 8 | data[3]); 
	motor->real_current = (int16_t)(data[4] << 8 | data[5]);
	motor->temperature = data[6];
  
  if(motor->first_run == true)
  {
    motor->angle = 0;
    motor->turns = 0;
    motor->angle_last = 0;
    motor->original_position = motor->angle_actual;
    motor->first_run = false;
  }
  else
    Cal_Angle(motor);
}



/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
* @brief          hal库回调函数，接收电机数据
  * @param[in]      hcan:can句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)  //can1
{
  CAN_RxHeaderTypeDef Rx_Msg;
  uint8_t rx_data[8];

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rx_Msg, rx_data);

  switch (Rx_Msg.StdId)			//can1 motor message decode
  {
		case CAN_Motor1_ID:
		case CAN_Motor2_ID:
		case CAN_Motor3_ID:
		case CAN_Motor4_ID:
		case CAN_Motor5_ID:
		case CAN_Motor6_ID:
		case CAN_Motor7_ID:
		case CAN_Motor8_ID:
        {
            static uint8_t i = 0;
            //get motor id
            i = Rx_Msg.StdId - CAN_Motor1_ID;
            GetMotor_Message(&motor_msg[i],rx_data);
            break;
        }
    default:
        {
            break;
        }
  }
  if(CAN1_send_current_flag == true)
  {
    CAN1_send_current_flag = false;
    CAN1_Set_AheadCur(motor_msg[0].given_current,motor_msg[1].given_current,motor_msg[2].given_current,motor_msg[3].given_current); //·¢ËÍÁ½¸ö3508 Ò»¸ö2006
    CAN1_Set_BackCur(motor_msg[4].given_current,motor_msg[5].given_current,motor_msg[6].given_current,0);     
  }
}


//发�?�前4个电机电流（底盘�?
void CAN1_Set_AheadCur(int16_t C1, int16_t C2, int16_t C3, int16_t C4)
{
  uint8_t TX_Data[8];
  uint32_t send_mail_box;
	CAN_TxHeaderTypeDef Tx_Msg;

	Tx_Msg.StdId=0x200;	 //set chassis motor current.
	Tx_Msg.IDE=CAN_ID_STD;		  //不使用拓展标识符
	Tx_Msg.RTR=CAN_RTR_DATA;		  //数据�?
	Tx_Msg.DLC=8; 
  
  TX_Data[0] = C1 >> 8;
	TX_Data[1] = C1;
	TX_Data[2] = C2 >> 8;
	TX_Data[3] = C2;
	TX_Data[4] = C3 >> 8;
	TX_Data[5] = C3;
	TX_Data[6] = C4 >> 8;
	TX_Data[7] = C4;
  
  HAL_CAN_AddTxMessage(&hcan1, &Tx_Msg, TX_Data, &send_mail_box);    //送进FIFO邮箱
}

//发�?�后4个电机电流（龙门架和储球机构�?
void CAN1_Set_BackCur(int16_t C1, int16_t C2, int16_t C3, int16_t C4)
{
  uint8_t TX_Data[8];
  uint32_t send_mail_box;
	CAN_TxHeaderTypeDef Tx_Msg;

	Tx_Msg.StdId=0x1FF;	 //set chassis motor current.
	Tx_Msg.IDE=CAN_ID_STD;		  //不使用拓展标识符
	Tx_Msg.RTR=CAN_RTR_DATA;		  //数据�?
	Tx_Msg.DLC=8; 
  
  TX_Data[0] = C1 >> 8;
	TX_Data[1] = C1;
	TX_Data[2] = C2 >> 8;
	TX_Data[3] = C2;
	TX_Data[4] = C3 >> 8;
	TX_Data[5] = C3;
	TX_Data[6] = C4 >> 8;
	TX_Data[7] = C4;
  
  HAL_CAN_AddTxMessage(&hcan1, &Tx_Msg, TX_Data, &send_mail_box);    //送进FIFO邮箱
}


void Can_Filter_Init(void)//can过滤器配�?
{

  CAN_FilterTypeDef CAN_FilterInitStructure;
  CAN_FilterInitStructure.FilterActivation = ENABLE;
  CAN_FilterInitStructure.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN_FilterInitStructure.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN_FilterInitStructure.FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.FilterIdLow = 0x0000;
  CAN_FilterInitStructure.FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.FilterBank = 0;
  CAN_FilterInitStructure.FilterFIFOAssignment = CAN_RX_FIFO0;
  HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterInitStructure);
	
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/* USER CODE END 1 */

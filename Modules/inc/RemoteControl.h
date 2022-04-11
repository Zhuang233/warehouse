#ifndef __REMOTECONTROL_H
#define __REMOTECONTROL_H
#include "stdint.h"
#include "timer.h"
#include "main.h"


#define RC_FRAME_LENGTH 18

//输入模式:遥控器/键盘鼠标/停止运行
#define OK 1
#define KEY_MOUSE_INPUT 3
#define STOP 2

//mouse control parameters
#define MOUSE_TO_PITCH_ANGLE_INC_FACT 		15.0f
#define MOUSE_TO_YAW_ANGLE_INC_FACT 		6.0f

//不能写1，要给自转留点
#define LOW_SPEED 			0.6
#define HIGH_SPEED 			0.8


#define F_POS 50000000.0f
#define VAL_LIMIT(val, min, max) \
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\

extern uint8_t input_mode;
extern TIME ramp_time;

typedef __packed struct
{
	/* rocker channel information */
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;

	/* left and right lever information */
	uint8_t sw1;
	uint8_t sw2;
	uint8_t sw1_last;
	uint8_t sw2_last;
	int16_t ch5;
} rc_info_t;

typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
} rc_info_mouse;

typedef	__packed struct
{
	uint16_t info;
}rc_info_key;

typedef __packed struct
{
	rc_info_t rc;
	rc_info_mouse mouse;
	rc_info_key key;
} RC_Ctl_t;

extern RC_Ctl_t RC_CtrlData;
extern int32_t pos1,pos2,pos3,pos4,pos5;
extern uint8_t remote_control_buffer[18];


void read_RemoteCrol_data(void);
void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void remote_control_init(void);
void DBUS_Decode(uint8_t *buff);
void Remote_Control(void);
void SetInputMode(void);


#endif

#include "RemoteControl.h"
#include "pid.h"
#include "Key.h"
#include "motor.h"
#include "math.h"
#include "stdlib.h"
#include "usart.h"
#include "arm_math.h"
#include "main.h"


#define MAX_MOTOR_RMP	9200                //������ת��
#define MAX_MOVE_RMP	7700                //ƽ������ٶ�
#define MAX_ROTATE_RMP  4000   				//��ת����ٶ�
#define KEY_MOVE_RMP 4000             //���̿����ƶ�ת��
#define MOUSE_ROTATE_RMP 3000         //��������ת�ٶ�
#define MOUSE_ROTATE_PARA 10                	//������������תϵ��

uint8_t FunctionMODE=0;
#define STANDBY		0           
#define FETCHBIGMODE 		1
#define FETCHSMALLMODE 		2
#define EXCHANGEMODE 3
#define SWIPEMODE	4
#define RESCUEMODE 5


uint8_t remote_control_buffer[18] ={0};
RC_Ctl_t RC_CtrlData;

//receive data, 18 bytes one frame, but set 36 bytes 
//����ԭʼ���ݣ�Ϊ18���ֽڣ�����36���ֽڳ��ȣ���ֹDMA����Խ��


int16_t k_rc_to_pullangle = 0;

TIME chassis_gyro_time;
TIME ramp_time;

uint8_t input_mode = STOP;


void read_RemoteCrol_data()
{
	DBUS_Decode(remote_control_buffer);
}

/**
  * @brief          ң����Э�����
  * @param          buff: ԭ������ָ��
  * @retval         none
  */
void DBUS_Decode(uint8_t *buff)
{
	RC_CtrlData.rc.sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
	RC_CtrlData.rc.sw2 = (buff[5] >> 4) & 0x0003;
	if(!(RC_CtrlData.rc.sw1 && RC_CtrlData.rc.sw2))
	{
		RC_CtrlData.rc.sw1 = RC_CtrlData.rc.sw1_last;
		RC_CtrlData.rc.sw2 = RC_CtrlData.rc.sw2_last;
		return;
	}
	RC_CtrlData.rc.ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
	RC_CtrlData.rc.ch1 -= 1024;
	RC_CtrlData.rc.ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
	RC_CtrlData.rc.ch2 -= 1024;
	RC_CtrlData.rc.ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
	RC_CtrlData.rc.ch3 -= 1024;
	RC_CtrlData.rc.ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
	RC_CtrlData.rc.ch4 -= 1024;
	RC_CtrlData.rc.ch5 =(buff[16]|buff[17] << 8) & 0x07FF;
	RC_CtrlData.rc.ch5 -= 1024;
	RC_CtrlData.mouse.x = ((int16_t)buff[6]) | ((int16_t)buff[7] << 8);
	RC_CtrlData.mouse.y = ((int16_t)buff[8]) | ((int16_t)buff[9] << 8);
	RC_CtrlData.mouse.z = ((int16_t)buff[10]) | ((int16_t)buff[11] << 8);

	RC_CtrlData.mouse.press_l = buff[12];
	RC_CtrlData.mouse.press_r = buff[13];

	RC_CtrlData.key.info = ((int16_t)buff[14]) | ((int16_t)buff[15] << 8);
	
	RC_CtrlData.rc.sw1_last = RC_CtrlData.rc.sw1;
	RC_CtrlData.rc.sw2_last = RC_CtrlData.rc.sw2;

	if (	(abs(RC_CtrlData.rc.ch1) > 660) || \
	        (abs(RC_CtrlData.rc.ch2) > 660) || \
	        (abs(RC_CtrlData.rc.ch3) > 660) || \
	        (abs(RC_CtrlData.rc.ch4) > 660)	)
	{
		memset(&RC_CtrlData.rc, 0, sizeof(rc_info_t));
	}

	//��ң�������
	if(fabs((float)RC_CtrlData.rc.ch1)<=6)	RC_CtrlData.rc.ch1=0;
	if(fabs((float)RC_CtrlData.rc.ch2)<=6)	RC_CtrlData.rc.ch2=0;
	if(fabs((float)RC_CtrlData.rc.ch3)<=6)	RC_CtrlData.rc.ch3=0;
	if(fabs((float)RC_CtrlData.rc.ch4)<=6)	RC_CtrlData.rc.ch4=0;
	if(fabs((float)RC_CtrlData.rc.ch5)<=6)	RC_CtrlData.rc.ch5=0;

	if(RC_CtrlData.rc.ch1>650)	RC_CtrlData.rc.ch1 =  650;
	if(RC_CtrlData.rc.ch1<-650)	RC_CtrlData.rc.ch1 = -650;
	if(RC_CtrlData.rc.ch2>650)	RC_CtrlData.rc.ch2 =  650;
	if(RC_CtrlData.rc.ch2<-650)	RC_CtrlData.rc.ch2 = -650;
	if(RC_CtrlData.rc.ch3>650)	RC_CtrlData.rc.ch3 =  650;
	if(RC_CtrlData.rc.ch3<-650)	RC_CtrlData.rc.ch3 = -650;
	if(RC_CtrlData.rc.ch4>650)	RC_CtrlData.rc.ch4 =  650;
	if(RC_CtrlData.rc.ch4<-650)	RC_CtrlData.rc.ch4 = -650;

}


void SetInputMode(void)	//��������ģʽ
{
	if (RC_CtrlData.rc.sw1 == 1)		//��
		input_mode = OK;
	else if (RC_CtrlData.rc.sw1 == 3) 	//�м�
		input_mode = STOP;
	else if(RC_CtrlData.rc.sw1 == 2) 	//��
		input_mode = STOP;
}


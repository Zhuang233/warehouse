#ifndef __ANO_H
#define __ANO_H

#include "main.h"

typedef struct
{
	uint8_t send_status;
	uint8_t send_motor;
	uint8_t send_speed;
	uint8_t send_gps;
}_dt_send_;
void ANO_Send_UserData(float temp1,float temp2,float temp3,float temp4,float temp5,float temp6);
//void ANO_Send_Status(void);
//void ANO_Send_Speed(void);
//void ANO_Send_GPS(void);
//void ANO_Send_Motor(void);
//void ANO_Send_Altitude(void);
//void ANO_Send_UserData(void);
//void ANO_Send_UserData_2(void);
//void ANO_Send_To_GroundStation(void);
//uint8_t ANO_Data_Analysis(uint8_t *buf,uint8_t length);


extern uint32_t pid_setting;
#endif

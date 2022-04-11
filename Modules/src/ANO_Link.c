#include "ANO_Link.h"

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)	) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

extern UART_HandleTypeDef huart6;
unsigned char ANO_data_to_send[50];

uint32_t pid_setting=100000;

void ANO_Send_UserData(float temp1,float temp2,float temp3,float temp4,float temp5,float temp6)
{
	uint8_t cnt=0;
	uint8_t sum=0;
	uint8_t i;
	ANO_data_to_send[cnt++]=0xAA;
	ANO_data_to_send[cnt++]=0xAA;
	ANO_data_to_send[cnt++]=0xF1;
	ANO_data_to_send[cnt++]=0;

	ANO_data_to_send[cnt++]=BYTE3(temp1);
	ANO_data_to_send[cnt++]=BYTE2(temp1);
	ANO_data_to_send[cnt++]=BYTE1(temp1);
	ANO_data_to_send[cnt++]=BYTE0(temp1);

	ANO_data_to_send[cnt++]=BYTE3(temp2);
	ANO_data_to_send[cnt++]=BYTE2(temp2);
	ANO_data_to_send[cnt++]=BYTE1(temp2);
	ANO_data_to_send[cnt++]=BYTE0(temp2);
	
	ANO_data_to_send[cnt++]=BYTE3(temp3);
	ANO_data_to_send[cnt++]=BYTE2(temp3);
	ANO_data_to_send[cnt++]=BYTE1(temp3);
	ANO_data_to_send[cnt++]=BYTE0(temp3);
	
	ANO_data_to_send[cnt++]=BYTE3(temp4);
	ANO_data_to_send[cnt++]=BYTE2(temp4);
	ANO_data_to_send[cnt++]=BYTE1(temp4);
	ANO_data_to_send[cnt++]=BYTE0(temp4);
	
	ANO_data_to_send[cnt++]=BYTE3(temp5);
	ANO_data_to_send[cnt++]=BYTE2(temp5);
	ANO_data_to_send[cnt++]=BYTE1(temp5);
	ANO_data_to_send[cnt++]=BYTE0(temp5);
	
	ANO_data_to_send[cnt++]=BYTE3(temp6);
	ANO_data_to_send[cnt++]=BYTE2(temp6);
	ANO_data_to_send[cnt++]=BYTE1(temp6);
	ANO_data_to_send[cnt++]=BYTE0(temp6);
	
	ANO_data_to_send[3]=cnt-4;
	for(i=0;i<cnt;i++)
	{
		sum += ANO_data_to_send[i];
	}
	ANO_data_to_send[cnt++]=sum;

	HAL_UART_Transmit(&huart6,ANO_data_to_send,cnt,50);

}

//_dt_send_ dt_send={0,0,0,0};
////void ANO_Send_Check(uint8_t ID,uint8_t check_sum)
////{
////	uint8_t cnt=0;
////	uint8_t sum=0;
////	uint8_t i;
////	ANO_data_to_send[cnt++]=0xAA;
////	ANO_data_to_send[cnt++]=0xAA;
////	ANO_data_to_send[cnt++]=0xEF;
////	ANO_data_to_send[cnt++]=2;
////	ANO_data_to_send[cnt++]=ID;
////	ANO_data_to_send[cnt++]=check_sum;
////	
////	for(i=0;i<cnt;i++)
////	{
////		sum += ANO_data_to_send[i];
////	}
////	ANO_data_to_send[cnt++]=sum;

////	USARTSendDatasByDMA(UART4,ANO_data_to_send,cnt);
////}
////void ANO_Send_Status(void)
////{
////	uint8_t cnt=0;
////	uint8_t sum=0;
////	uint8_t i;
////	int16_t temp;
////	int32_t alt=global_params.AltitudeActual*100;//alt in cm.
////	
////	ANO_data_to_send[cnt++]=0xAA;
////	ANO_data_to_send[cnt++]=0xAA;
////	ANO_data_to_send[cnt++]=0x01;
////	ANO_data_to_send[cnt++]=0;
////	temp=Roll*100;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=Pitch*100;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=Yaw*100;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	//send altitude
////	ANO_data_to_send[cnt++]=BYTE3(alt);
////	ANO_data_to_send[cnt++]=BYTE2(alt);
////	ANO_data_to_send[cnt++]=BYTE1(alt);
////	ANO_data_to_send[cnt++]=BYTE0(alt);
////	
////	ANO_data_to_send[cnt++]=global_params.ControlMode;
////	ANO_data_to_send[cnt++]=!global_params.MotorLocked;
////	ANO_data_to_send[3]=cnt-4;
////	for(i=0;i<cnt;i++)
////	{
////		sum += ANO_data_to_send[i];
////	}
////	ANO_data_to_send[cnt++]=sum;

////	USARTSendDatasByDMA(UART4,ANO_data_to_send,cnt);
////}

////void ANO_Send_Speed(void)
////{
////	uint8_t cnt=0;
////	uint8_t sum=0;
////	uint8_t i;
////	int16_t temp;

////	ANO_data_to_send[cnt++]=0xAA;
////	ANO_data_to_send[cnt++]=0xAA;
////	ANO_data_to_send[cnt++]=0x0B;
////	ANO_data_to_send[cnt++]=0;
////	temp=EST_X[1]*100;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=EST_Y[1]*100;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=VerVelEst*100;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);

////	ANO_data_to_send[3]=cnt-4;
////	for(i=0;i<cnt;i++)
////	{
////		sum += ANO_data_to_send[i];
////	}
////	ANO_data_to_send[cnt++]=sum;

////	USARTSendDatasByDMA(UART4,ANO_data_to_send,cnt);
////}

////void ANO_Send_Motor(void)
////{
////	uint8_t cnt=0;
////	uint8_t sum=0;
////	uint8_t i;

////	ANO_data_to_send[cnt++]=0xAA;
////	ANO_data_to_send[cnt++]=0xAA;
////	ANO_data_to_send[cnt++]=0x06;
////	ANO_data_to_send[cnt++]=0;

////	ANO_data_to_send[cnt++]=BYTE1(motor1);
////	ANO_data_to_send[cnt++]=BYTE0(motor1);
////	ANO_data_to_send[cnt++]=BYTE1(motor2);
////	ANO_data_to_send[cnt++]=BYTE0(motor2);
////	ANO_data_to_send[cnt++]=BYTE1(motor3);
////	ANO_data_to_send[cnt++]=BYTE0(motor3);
////	ANO_data_to_send[cnt++]=BYTE1(motor4);
////	ANO_data_to_send[cnt++]=BYTE0(motor4);
////	//motor5
////	ANO_data_to_send[cnt++]=0;
////	ANO_data_to_send[cnt++]=0;
////	ANO_data_to_send[cnt++]=0;
////	ANO_data_to_send[cnt++]=0;
////		//motor6
////	ANO_data_to_send[cnt++]=0;
////	ANO_data_to_send[cnt++]=0;
////	ANO_data_to_send[cnt++]=0;
////	ANO_data_to_send[cnt++]=0;
////		//motor7
////	ANO_data_to_send[cnt++]=0;
////	ANO_data_to_send[cnt++]=0;
////	ANO_data_to_send[cnt++]=0;
////	ANO_data_to_send[cnt++]=0;
////		//motor8
////	ANO_data_to_send[cnt++]=0;
////	ANO_data_to_send[cnt++]=0;
////	ANO_data_to_send[cnt++]=0;
////	ANO_data_to_send[cnt++]=0;
////	//send altitude
////	ANO_data_to_send[3]=cnt-4;
////	for(i=0;i<cnt;i++)
////	{
////		sum += ANO_data_to_send[i];
////	}
////	ANO_data_to_send[cnt++]=sum;

////	USARTSendDatasByDMA(UART4,ANO_data_to_send,cnt);
////}

////void ANO_Send_GPS(void)
////{
////	uint8_t cnt=0;
////	uint8_t sum=0;
////	uint8_t i;
////	int32_t temp;
////	
////	ANO_data_to_send[cnt++]=0xAA;
////	ANO_data_to_send[cnt++]=0xAA;
////	ANO_data_to_send[cnt++]=0x04;
////	ANO_data_to_send[cnt++]=0;

////	ANO_data_to_send[cnt++]=gps.available;
////	ANO_data_to_send[cnt++]=gps.svnum;
////	//lontitude
////	temp=gps.longitude*10000000;
////	ANO_data_to_send[cnt++]=BYTE3(temp);
////	ANO_data_to_send[cnt++]=BYTE2(temp);
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=gps.latitude*10000000;
////	ANO_data_to_send[cnt++]=BYTE3(temp);
////	ANO_data_to_send[cnt++]=BYTE2(temp);
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	//back home angle
////	ANO_data_to_send[cnt++]=0;
////	ANO_data_to_send[cnt++]=0;	
////	
////	ANO_data_to_send[3]=cnt-4;
////	for(i=0;i<cnt;i++)
////	{
////		sum += ANO_data_to_send[i];
////	}
////	ANO_data_to_send[cnt++]=sum;

////	USARTSendDatasByDMA(UART4,ANO_data_to_send,cnt);
////}
////void ANO_Send_Altitude(void)
////{
////	uint8_t cnt=0;
////	uint8_t sum=0;
////	uint8_t i;
////	int32_t temp;
////	ANO_data_to_send[cnt++]=0xAA;
////	ANO_data_to_send[cnt++]=0xAA;
////	ANO_data_to_send[cnt++]=0x07;
////	ANO_data_to_send[cnt++]=0;
////	temp=altitude_baro*100;
////	ANO_data_to_send[cnt++]=BYTE3(temp);
////	ANO_data_to_send[cnt++]=BYTE2(temp);
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);

////	ANO_data_to_send[cnt++]=0;
////	ANO_data_to_send[cnt++]=0;

////	ANO_data_to_send[3]=cnt-4;
////	for(i=0;i<cnt;i++)
////	{
////		sum += ANO_data_to_send[i];
////	}
////	ANO_data_to_send[cnt++]=sum;

////	USARTSendDatasByDMA(UART4,ANO_data_to_send,cnt);
////}

//void ANO_Send_UserData(void)
//{
//	uint8_t cnt=0;
//	uint8_t sum=0;
//	uint8_t i;
//	float temp;
//	ANO_data_to_send[cnt++]=0xAA;
//	ANO_data_to_send[cnt++]=0xAA;
//	ANO_data_to_send[cnt++]=0xF1;
//	ANO_data_to_send[cnt++]=0;
//	
////	temp=Roll;
////	temp=gps.pos_y;
//	temp=altitude_baro;
//	ANO_data_to_send[cnt++]=BYTE3(temp);
//	ANO_data_to_send[cnt++]=BYTE2(temp);
//	ANO_data_to_send[cnt++]=BYTE1(temp);
//	ANO_data_to_send[cnt++]=BYTE0(temp);

////	temp=Pitch;
////	temp=EST_Y[0];
//	temp=global_params.AltitudeActual;
//	ANO_data_to_send[cnt++]=BYTE3(temp);
//	ANO_data_to_send[cnt++]=BYTE2(temp);
//	ANO_data_to_send[cnt++]=BYTE1(temp);
//	ANO_data_to_send[cnt++]=BYTE0(temp);
//	
////	temp=Yaw;
////	temp=EST_Y[1];
//	temp=VerVelEst;
//	ANO_data_to_send[cnt++]=BYTE3(temp);
//	ANO_data_to_send[cnt++]=BYTE2(temp);
//	ANO_data_to_send[cnt++]=BYTE1(temp);
//	ANO_data_to_send[cnt++]=BYTE0(temp);
//	
//	temp=roll_z;
////	temp=0;
//	ANO_data_to_send[cnt++]=BYTE3(temp);
//	ANO_data_to_send[cnt++]=BYTE2(temp);
//	ANO_data_to_send[cnt++]=BYTE1(temp);
//	ANO_data_to_send[cnt++]=BYTE0(temp);
//	
//	temp=pitch_z;
////	temp=0;
//	ANO_data_to_send[cnt++]=BYTE3(temp);
//	ANO_data_to_send[cnt++]=BYTE2(temp);
//	ANO_data_to_send[cnt++]=BYTE1(temp);
//	ANO_data_to_send[cnt++]=BYTE0(temp);
//	temp=yaw_z;
////	temp=0;
//	ANO_data_to_send[cnt++]=BYTE3(temp);
//	ANO_data_to_send[cnt++]=BYTE2(temp);
//	ANO_data_to_send[cnt++]=BYTE1(temp);
//	ANO_data_to_send[cnt++]=BYTE0(temp);
//	
//	ANO_data_to_send[3]=cnt-4;
//	for(i=0;i<cnt;i++)
//	{
//		sum += ANO_data_to_send[i];
//	}
//	ANO_data_to_send[cnt++]=sum;

//	USARTSendDatasByDMA(UART4,ANO_data_to_send,cnt);
//}

////void ANO_Send_UserData_2(void)
////{
////	uint8_t cnt=0;
////	uint8_t sum=0;
////	uint8_t i;
////	int16_t temp;
////	ANO_data_to_send[cnt++]=0xAA;
////	ANO_data_to_send[cnt++]=0xAA;
////	ANO_data_to_send[cnt++]=0xF2;
////	ANO_data_to_send[cnt++]=0;
////	
////	temp=mpu6050_raw.ax;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=mpu6050_raw.ay;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=mpu6050_raw.az;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=mpu6050_raw.gy;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	
////	temp=mpu6050.ax;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=mpu6050.ay;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=mpu6050.az;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=mpu6050.gy;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	
////	temp=mpu6050_LP.ax;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=mpu6050_LP.ay;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=mpu6050_LP.az;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=mpu6050_LP.gy;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	ANO_data_to_send[3]=cnt-4;
////	for(i=0;i<cnt;i++)
////	{
////		sum += ANO_data_to_send[i];
////	}
////	ANO_data_to_send[cnt++]=sum;

////	USARTSendDatasByDMA(UART4,ANO_data_to_send,cnt);
////}

////void ANO_Send_Rate_PID(void)
////{
////	uint8_t cnt=0;
////	uint8_t sum=0;
////	uint8_t i;
////	int16_t temp;
////	ANO_data_to_send[cnt++]=0xAA;
////	ANO_data_to_send[cnt++]=0xAA;
////	ANO_data_to_send[cnt++]=0x10;
////	ANO_data_to_send[cnt++]=18;
////	//roll
////	temp=pidRollRate.kp*1000;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=pidRollRate.ki*1000;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=pidRollRate.kd*1000;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	//pitch
////	temp=pidPitchRate.kp*1000;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=pidPitchRate.ki*1000;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=pidPitchRate.kd*1000;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	//yaw
////	temp=pidYawRate.kp*1000;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=pidYawRate.ki*1000;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=pidYawRate.kd*1000;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	
////	ANO_data_to_send[3]=cnt-4;
////	for(i=0;i<cnt;i++)
////	{
////		sum += ANO_data_to_send[i];
////	}
////	ANO_data_to_send[cnt++]=sum;

////	USARTSendDatasByDMA(UART4,ANO_data_to_send,cnt);
////}

////void ANO_Send_Att_PID(void)
////{
////	uint8_t cnt=0;
////	uint8_t sum=0;
////	uint8_t i;
////	int16_t temp;
////	ANO_data_to_send[cnt++]=0xAA;
////	ANO_data_to_send[cnt++]=0xAA;
////	ANO_data_to_send[cnt++]=0x11;
////	ANO_data_to_send[cnt++]=18;
////	//roll
////	temp=pidRoll.kp*1000;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=pidRoll.ki*1000;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=pidRoll.kd*1000;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	//pitch
////	temp=pidPitch.kp*1000;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=pidPitch.ki*1000;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=pidPitch.kd*1000;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	//yaw
////	temp=pidYaw.kp*1000;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=pidYaw.ki*1000;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=pidYaw.kd*1000;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	
////	ANO_data_to_send[3]=cnt-4;
////	for(i=0;i<cnt;i++)
////	{
////		sum += ANO_data_to_send[i];
////	}
////	ANO_data_to_send[cnt++]=sum;

////	USARTSendDatasByDMA(UART4,ANO_data_to_send,cnt);
////}

////void ANO_Send_Alt_and_Pos_PID(void)
////{
////	uint8_t cnt=0;
////	uint8_t sum=0;
////	uint8_t i;
////	int16_t temp;
////	ANO_data_to_send[cnt++]=0xAA;
////	ANO_data_to_send[cnt++]=0xAA;
////	ANO_data_to_send[cnt++]=0x12;
////	ANO_data_to_send[cnt++]=18;
////	//verVelocity
////	temp=pidVerVelocity.kp*100;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=pidVerVelocity.ki*100;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=pidVerVelocity.kd*100;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	//altitude
////	temp=pidAltitude.kp*100;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=pidAltitude.ki*100;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=pidAltitude.kd*100;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	//horVelocity
////	temp=pidVel_X.kp*1000;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=pidVel_X.ki*1000;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	temp=pidVel_X.kd*1000;
////	ANO_data_to_send[cnt++]=BYTE1(temp);
////	ANO_data_to_send[cnt++]=BYTE0(temp);
////	
////	ANO_data_to_send[3]=cnt-4;
////	for(i=0;i<cnt;i++)
////	{
////		sum += ANO_data_to_send[i];
////	}
////	ANO_data_to_send[cnt++]=sum;

////	USARTSendDatasByDMA(UART4,ANO_data_to_send,cnt);
////}

////void ANO_Send_To_GroundStation(void)
////{
////	static int8_t cnt=0;
////	cnt++;
////	if(cnt%10==0)//10,20
////	{
////		if(cnt==20) cnt=0;
////		ANO_Send_GPS();
////		return;
////	}
////	if(cnt==7||cnt==13||cnt==17)
////	{
////		ANO_Send_Speed();
////		return;
////	}
////	if(cnt==4||cnt==11||cnt==15||cnt==19)
////	{
////		ANO_Send_Motor();
////		return;
////	}
////	if(cnt==2||cnt==8||cnt==12||cnt==18)
////	{
////		ANO_Send_Altitude();
////		return;
////	}
////	ANO_Send_Status();
////}

////void Rate_PID_Set(uint8_t buf[])
////{
////	pidRollRate.kp=0.001*(buf[0]<<8|buf[1]);
////	pidRollRate.ki=0.001*(buf[2]<<8|buf[3]);
////	pidRollRate.kd=0.001*(buf[4]<<8|buf[5]);
////	
////	pidPitchRate.kp=0.001*(buf[6]<<8|buf[7]);
////	pidPitchRate.ki=0.001*(buf[8]<<8|buf[9]);
////	pidPitchRate.kd=0.001*(buf[10]<<8|buf[11]);
////	
////	pidYawRate.kp=0.001*(buf[12]<<8|buf[13]);
////	pidYawRate.ki=0.001*(buf[14]<<8|buf[15]);
////	pidYawRate.kd=0.001*(buf[16]<<8|buf[17]);
////}

////void Att_PID_Set(uint8_t buf[])
////{
////	pidRoll.kp=0.001*(buf[0]<<8|buf[1]);
////	pidRoll.ki=0.001*(buf[2]<<8|buf[3]);
////	pidRoll.kd=0.001*(buf[4]<<8|buf[5]);
////	
////	pidPitch.kp=0.001*(buf[6]<<8|buf[7]);
////	pidPitch.ki=0.001*(buf[8]<<8|buf[9]);
////	pidPitch.kd=0.001*(buf[10]<<8|buf[11]);
////	
////	pidYaw.kp=0.001*(buf[12]<<8|buf[13]);
////	pidYaw.ki=0.001*(buf[14]<<8|buf[15]);
////	pidYaw.kd=0.001*(buf[16]<<8|buf[17]);
////}
////void Alt_Pos_PID_Set(uint8_t *buf)
////{
////	pidVerVelocity.kp=0.01*(buf[0]<<8|buf[1]);
////	pidVerVelocity.ki=0.01*(buf[2]<<8|buf[3]);
////	pidVerVelocity.kd=0.01*(buf[4]<<8|buf[5]);
////	
////	pidAltitude.kp=0.01*(buf[6]<<8|buf[7]);
////	pidAltitude.ki=0.01*(buf[8]<<8|buf[9]);
////	pidAltitude.kd=0.001*(buf[10]<<8|buf[11]);
////	
////	pidVel_Y.kp=pidVel_X.kp=0.001*(buf[12]<<8|buf[13]);
////	pidVel_Y.ki=pidVel_X.ki=0.001*(buf[14]<<8|buf[15]);
////	pidVel_Y.kd=pidVel_X.kd=0.001*(buf[16]<<8|buf[17]);
////}
////uint8_t ANO_Data_Analysis(uint8_t *buf,uint8_t length)
////{
////	uint8_t sum=0;
////	uint8_t i;
////	uint8_t frameID;
////	uint8_t CMD;
////	if(buf[0]!=0xAA&&buf[1]!=0xAF)
////		return 0x01;
////	if(length!=(buf[3]+5))
////		return 0x02;
////	for(i=0;i<buf[3]+4;i++)
////	{
////		sum+=buf[i];
////	}
////	if(sum!=buf[length-1])
////		return 0x03;
////	frameID=buf[2];
////	if(frameID==0x01)
////	{
////		CMD=buf[4];
////	}
////	else if(frameID==0x02)
////	{
////		CMD=buf[4];
////		if(CMD==0x01)
////		{
////			vTaskDelay(20);
////			ANO_Send_Rate_PID();
////			vTaskDelay(20);
////			ANO_Send_Att_PID();
////			vTaskDelay(20);
////			ANO_Send_Alt_and_Pos_PID();
////			vTaskDelay(20);
////		}
////	}
////	else if(frameID==0x10)
////	{
////		Rate_PID_Set(buf+4);
////		ANO_Send_Check(frameID,sum);
////		pid_setting=0;
////	}
////	else	if(frameID==0x11)
////	{
////		Att_PID_Set(buf+4);
////		ANO_Send_Check(frameID,sum);
////		pid_setting=0;
////	}
////	else if(frameID==0x12)
////	{
////		Alt_Pos_PID_Set(buf+4);
////		ANO_Send_Check(frameID,sum);
////		pid_setting=0;
////	}
////	else if(frameID==0x13)
////	{
////		ANO_Send_Check(frameID,sum);
////		pid_setting=0;
////	}
////	else if(frameID==0x14)
////	{
////		ANO_Send_Check(frameID,sum);
////		pid_setting=0;
////	}
////	else if(frameID==0x15)
////	{
////		ANO_Send_Check(frameID,sum);
////		pid_setting=0;
////	}
////	return 0x00;
////}


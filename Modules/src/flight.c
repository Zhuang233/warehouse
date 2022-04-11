#include "flight.h"

_att_info_ att_info[5];

void get_att_info(int i)
{	
	if(i>=5)
		return;
	
}

void FlightCtrlSysInit(void)
{
	int i;
	u32 time_now=0,time_last=0;
	float dt;
	int count=0;
	for(i=0;i<20;i++)
	{
		MPU6050_Get_Motion6();
		AK8975_Get_Magnetism();
//		HMC5883L_Get_Magnetism();
		if(i==20)
		{ 
			if(mpu6050.ax==0&&mpu6050.ay==0&&mpu6050.az==0)
				i=0;
//			if(hmc5883l.x==0&&hmc5883l.y==0&&hmc5883l.z==0)
//				i=0;
		}
		delay_ms(2);
	}
	AttitudeEKF_Init(mpu6050.ax,mpu6050.ay,mpu6050.az,-ak8975.y,-ak8975.x,-ak8975.z);
	delay_ms(1);
	gyro_filter_high=0.8;
	for(i=0;i<2500;i++)
	{
		time_last=micros();
		GetAttitude();
		//get_att_info(i);
		if(count==5&&baro_available)
		{
			count=0;
			altitude_baro=MS5611_get_height();
			if(i>50)
				get_alt_baro_ground(altitude_baro);
		}
		count++;
		time_now=micros();
		calculate_dt_in_mills(time_now,time_last,&dt);
		if(dt<2)
			delay_ms(2-dt);
	}
	gyro_filter_high=0.5;
	global_params.AttitudeDesired.yaw=Yaw;
	global_params.AltitudeDesired=0;
	
	Location_Init();
}


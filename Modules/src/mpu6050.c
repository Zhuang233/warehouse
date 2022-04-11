#include "mpu6050.h"
#include "i2c.h"
#include "led.h"
#include "stdint.h"
#include "stdio.h"
#include "delay.h"

#define filter_high 0.5
#define filter_low  0.5

#define MPU6050_BUF_SIZE 10

float gyro_filter_high=0.5;

uint8_t   DeviceID=0;
uint8_t		mpu6050_buffer[14];
int16_t 	Gyro_Offset[3]={0,0,0};
inertial_sensor_data mpu6050_raw,mpu6050;
inertial_sensor_data mpu6050_2={0,0,0,0,0,0};
inertial_sensor_data mpu6050_LP;

slide_mean_struct mpu6050_acc_filter[3];
float MPU6050_ACC_FILTER_BUF[3][MPU6050_BUF_SIZE];
void MPU6050_ACC_Filter_Init(void)
{
	uint8_t i;
	for(i=0;i<3;i++)
	{
		mpu6050_acc_filter[i].Buf_Pointer=0;
		mpu6050_acc_filter[i].buf_sum=0;
		mpu6050_acc_filter[i].data_count=0;
		mpu6050_acc_filter[i].FIFO_SIZE=MPU6050_BUF_SIZE;
		mpu6050_acc_filter[i].FIFO_BUF=MPU6050_ACC_FILTER_BUF[i];
	}
}

void MPU6050_Init(void)
{
	i2cReadBytes(MPUAddress,WHO_AM_I,&DeviceID,1);
	while(DeviceID!=0x68)
	{
		delay_ms(100);
		LED0=!LED0;
		i2cReadBytes(MPUAddress,WHO_AM_I,&DeviceID,1);
	}
	i2cWriteByte(MPUAddress,PWR_MGMT1,0x00);
	i2cWriteByte(MPUAddress,SMPLRT_DIV,0x07);
	i2cWriteByte(MPUAddress,CONFIG_1,0x06);	
	i2cWriteByte(MPUAddress,GYRO_CONFIG,0x18);	
	i2cWriteByte(MPUAddress,ACCEL_CONFIG,0x18);	//+/-16g
	MPU6050_ACC_Filter_Init();
	//capsule test platform
//	Gyro_Offset[0]=-9;
//	Gyro_Offset[1]=-10;
//	Gyro_Offset[2]=11;
	//small_copter
//	Gyro_Offset[0]=-6;
//	Gyro_Offset[1]=-14;
//	Gyro_Offset[2]=6;
	
//	//capsule No.1
//	Gyro_Offset[0]=12;
//	Gyro_Offset[1]=24;
//	Gyro_Offset[2]=-67;
	//capsule No.2
	Gyro_Offset[0]=52;
	Gyro_Offset[1]=14;
	Gyro_Offset[2]=-17;	
}

void Gyro_Get_Offset(void)
{
	Gyro_Offset[0]=mpu6050.gx;
	Gyro_Offset[1]=mpu6050.gy;
	Gyro_Offset[2]=mpu6050.gz;
	mpu6050.gx=0;
	mpu6050.gy=0;
	mpu6050.gz=0;
}
	

int16_t acc_Xo=33,acc_Yo=-5,acc_Zo=-450;
void MPU6050_Get_Motion6(void)
{
	//__disable_irq();
	i2cReadBytes(MPUAddress,ACCEL_XOUTH,mpu6050_buffer,14);
	//__enable_irq();
	mpu6050_raw.ax=((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) - acc_Xo;
	mpu6050_raw.ay=((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) - acc_Yo;
	mpu6050_raw.az=((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]) - acc_Zo;
	//Ìø¹ýÎÂ¶ÈADC
	mpu6050_raw.gx=((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9])  -Gyro_Offset[0];//75
	mpu6050_raw.gy=((((int16_t)mpu6050_buffer[10]) << 8) | mpu6050_buffer[11])-Gyro_Offset[1];//41
	mpu6050_raw.gz=((((int16_t)mpu6050_buffer[12]) << 8) | mpu6050_buffer[13])-Gyro_Offset[2];//+25
	
//	mpu6050.ax = mpu6050.ax*filter_high+filter_low*mpu6050_raw.ax;
//	mpu6050.ay = mpu6050.ay*filter_high+filter_low*mpu6050_raw.ay;
//	mpu6050.az = mpu6050.az*filter_high+filter_low*mpu6050_raw.az;
	
	mpu6050.ax=SildeMeanFilter(&mpu6050_acc_filter[0], -mpu6050_raw.ax);
	mpu6050.ay=SildeMeanFilter(&mpu6050_acc_filter[1], -mpu6050_raw.ay);
	mpu6050.az=SildeMeanFilter(&mpu6050_acc_filter[2], mpu6050_raw.az);
	
	mpu6050.gx =mpu6050.gx*gyro_filter_high + (-mpu6050_raw.gx)*(1.0f-gyro_filter_high);
	mpu6050.gy =mpu6050.gy*gyro_filter_high + (-mpu6050_raw.gy)*(1.0f-gyro_filter_high);
	mpu6050.gz =mpu6050.gz*gyro_filter_high + ( mpu6050_raw.gz)*(1.0f-gyro_filter_high);
	
	mpu6050_2.gx =mpu6050_2.gx*0.7f + (-mpu6050_raw.gx)*(1.0f-0.7f);
	mpu6050_2.gy =mpu6050_2.gy*0.7f + (-mpu6050_raw.gy)*(1.0f-0.7f);
	mpu6050_2.gz =mpu6050_2.gz*0.3f + ( mpu6050_raw.gz)*(1.0f-0.3f);//0.4
	
	body_gyro.x=mpu6050.gx*GYRO_SCALE;
	body_gyro.y=mpu6050.gy*GYRO_SCALE;
	body_gyro.z=mpu6050.gz*GYRO_SCALE;
	//for angluar acceleration control.
	body_gyro_2.x=mpu6050_2.gx*GYRO_SCALE;
	body_gyro_2.y=mpu6050_2.gy*GYRO_SCALE;
	body_gyro_2.z=mpu6050_2.gz*GYRO_SCALE;
	
	body_acc.x=mpu6050.ax*ACC_SCALE;
	body_acc.y=mpu6050.ay*ACC_SCALE;
	body_acc.z=mpu6050.az*ACC_SCALE;
	
//	printf("%d,%d,%d;\r\n",mpu6050_raw.ax,mpu6050_raw.ay,mpu6050_raw.az);
//	vTaskDelay(200);

//	body_gyro.x=body_gyro.y=body_gyro.z=0;
//	body_gyro_2.x=body_gyro_2.y=body_gyro_2.z=0;
//	body_acc.x=body_acc.y=0;
//	body_acc.z=1;
}



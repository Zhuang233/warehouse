#include "pan.h"

#define ANGLE_BOX 40

Pan_t bodanpan;

void pan_init()
{
	for(uint8_t i = 0;i < 9; i++)
	{
		bodanpan.date[i] = 0;
		bodanpan.box_state[i] = 0;
	}
	//test
//	bodanpan.date[0] = 0x12;
//	bodanpan.date[1] = 0x23;
//	bodanpan.date[2] = 0x21;
//	bodanpan.date[3] = 0x11;
//	bodanpan.date[4] = 0x32;
//	bodanpan.date[5] = 0x13;
//	bodanpan.date[6] = 0x33;
//	bodanpan.date[7] = 0x22;
//	bodanpan.date[8] = 0x31;	
	//
	bodanpan.angle = 0;
	bodanpan.position = 0;
	bodanpan.ptr = bodanpan.date;
}

void turn_box(uint8_t direction,uint8_t num)//通过需要转动的格数 计算对应角度
{
	if(direction)
	{
		bodanpan.angle += num*ANGLE_BOX;
		bodanpan.position -= num;
	}
	else
	{
		bodanpan.angle -= num*ANGLE_BOX;
		bodanpan.position += num;
	}
	motor_msg[6].angle_desired = bodanpan.angle*819.2;//一般角度制转换为机械角 8192/360*36
	if(bodanpan.position < 0) bodanpan.position+=9;
	if(bodanpan.position >= 9) bodanpan.position%=9;
}

//储球机构吐出一个指定的球 x为行 y为列 
void put_a_ball(uint8_t x,uint8_t y)
{
	int8_t t,d=0;
	for(uint8_t i=0;i<9;i++)
	{
		if(bodanpan.date[i]== x*16+y )
		{
			t=bodanpan.position-i;
			if(t<-4)
			{
				d=1;
				t+=9;
			}
			else if(t>4)
			{
				d=0;
				t=9-t;
			}
			else if(t>0&&t<=4)
			{
				d=1;
			}
			else if(t<0&&t>=-4)
			{
				d=0;
				t=-t;
			}
			turn_box(d,t);
		}
	}
}



#include "RFID.h"


/*
M3650A-HA协议

数据包格式:
0				1				2 				3 		4 					5-20 					21
包类型 	包长度 	返回命令 	地址 	状态 				数据信息 			校验和
0x04 		0x16 		0x03 			0x20 	0x00:成功 	16字节数据 		X

数据输出：
0x04 0x16 0x03 0x20 0x00 16字节数据 X校验和

校验和：
X=前21字节依次按位异或最后再取反

*/
bool ic_read_start = 0;
uint8_t ic_data=0;
uint8_t ic_buf[22]={0};
uint8_t ic_buf_size = 0;
uint8_t X = 0;




void decode_ic_data()
{
	
	
	if(ic_read_start == 1)
	{
		ic_buf[ic_buf_size++] = ic_data;
		
		if(ic_buf_size == 22)//接收完毕
		{
			ic_read_start = 0;
			ic_buf_size = 0;
			for(uint8_t i = 0;i < 21 ; i++)
			{
				X^=ic_buf[i];
			}
			X=~X;
			if(X == ic_buf[21])//校验成功
			{
				bool flag = 1;
				for(uint8_t i=0;i<9;i++)//判断数据是否已经存过
				{
					if(ic_buf[15]==bodanpan.date[i])
					{
						flag = 0;
					}
				}
				if(flag)//存ic数据
				{
					uint8_t which_box = 0;
					if(bodanpan.position == 0) which_box = 8;
					else which_box = bodanpan.position - 1;
					bodanpan.date[which_box] = ic_buf[15];
					bodanpan.box_state[which_box] = 1;
				}
			}
			X = 0;
		}
	}
	
	if(ic_read_start == 0 && ic_data == 0x04)//检测到帧头
	{
		
		ic_read_start = 1;
		ic_buf[ic_buf_size++] = ic_data;
	}



}


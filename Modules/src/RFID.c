#include "RFID.h"


/*
M3650A-HAЭ��

���ݰ���ʽ:
0				1				2 				3 		4 					5-20 					21
������ 	������ 	�������� 	��ַ 	״̬ 				������Ϣ 			У���
0x04 		0x16 		0x03 			0x20 	0x00:�ɹ� 	16�ֽ����� 		X

���������
0x04 0x16 0x03 0x20 0x00 16�ֽ����� XУ���

У��ͣ�
X=ǰ21�ֽ����ΰ�λ��������ȡ��

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
		
		if(ic_buf_size == 22)//�������
		{
			ic_read_start = 0;
			ic_buf_size = 0;
			for(uint8_t i = 0;i < 21 ; i++)
			{
				X^=ic_buf[i];
			}
			X=~X;
			if(X == ic_buf[21])//У��ɹ�
			{
				bool flag = 1;
				for(uint8_t i=0;i<9;i++)//�ж������Ƿ��Ѿ����
				{
					if(ic_buf[15]==bodanpan.date[i])
					{
						flag = 0;
					}
				}
				if(flag)//��ic����
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
	
	if(ic_read_start == 0 && ic_data == 0x04)//��⵽֡ͷ
	{
		
		ic_read_start = 1;
		ic_buf[ic_buf_size++] = ic_data;
	}



}


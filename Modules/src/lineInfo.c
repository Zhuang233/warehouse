#include "lineInfo.h"



bool line_read_start = 0;
uint8_t line_data = 0;
uint8_t line_buf_size = 0;
uint8_t line_buf[2];

bool beyoundred[15] = {0};

void decode_line_data()
{
	if(line_read_start == 1)
	{
		line_buf[line_buf_size++] = line_data;
		if(line_buf_size == 2)
		{
			line_read_start = 0;
			line_buf_size = 0;
			beyoundred[0] =  line_buf[0] & 0x01;
			beyoundred[1] = (line_buf[0] & 0x02)>>1;
			beyoundred[2] = (line_buf[0] & 0x04)>>2;
			beyoundred[3] = (line_buf[0] & 0x08)>>3;
			beyoundred[4] = (line_buf[0] & 0x10)>>4;
			beyoundred[5] = (line_buf[0] & 0x20)>>5;
			beyoundred[6] = (line_buf[0] & 0x40)>>6;
			beyoundred[7] = (line_buf[0] & 0x80)>>7;
			
			beyoundred[8]  =  line_buf[1] & 0x01;
			beyoundred[9]  = (line_buf[1] & 0x02)>>1;
			beyoundred[10] = (line_buf[1] & 0x04)>>2;
			beyoundred[11] = (line_buf[1] & 0x08)>>3;
			beyoundred[12] = (line_buf[1] & 0x10)>>4;
			beyoundred[13] = (line_buf[1] & 0x20)>>5;
			beyoundred[14] = (line_buf[1] & 0x40)>>6;
		}
	}
	if(line_read_start == 0 && line_data == 0xff)//¼ì²âµ½Ö¡Í·
	{
		line_read_start = 1;
	}
}

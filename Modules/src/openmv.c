#include "openmv.h"


bool openmv_sw = 1;
bool openmv_read_start = 0;
uint8_t openmv_data = 0;
uint8_t openmv_buf_size = 0;
uint8_t openmv_buf[2];
uint8_t ball_x = 0;
uint8_t ball_y = 0;


void decode_openmv_data()
{
	if(openmv_read_start == 1)
	{
		openmv_buf[openmv_buf_size++] = openmv_data;
		if(openmv_buf_size == 2)
		{
			openmv_read_start = 0;
			openmv_buf_size = 0;
			ball_x = openmv_buf[0];
			ball_y = openmv_buf[1];
		}
	}
	if(openmv_sw == 1 && openmv_read_start == 0 && openmv_data == 0xff)//¼ì²âµ½Ö¡Í·
	{
		openmv_read_start = 1;
	}
}

void close_openmv()
{
	openmv_sw = 0;
	ball_x = 0;
	ball_y = 0;
}

void open_openmv()
{
	openmv_sw = 1;
}


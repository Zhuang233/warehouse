#ifndef __BASECALCULATE_H
#define __BASECALCULATE_H

#include "main.h"
#include "timer.h"

typedef struct 
{
	int16_t FIFO_SIZE;
	int16_t Buf_Pointer;
	int16_t data_count;
	float buf_sum;
	float *FIFO_BUF;
}slide_mean_struct;

typedef struct
{
  float output;
  float slope;
}ramp_struct;

extern ramp_struct chassis_ramp[2];
float SildeMeanFilter(slide_mean_struct *obj,float newValue);
void Ramp_Calculate(ramp_struct *obj,float target);
void Ramp_Init(float slope,float dt);

#endif

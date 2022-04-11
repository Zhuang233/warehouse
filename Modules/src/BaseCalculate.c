#include "BaseCalculate.h"
#include "Remotecontrol.h"

float SildeMeanFilter(slide_mean_struct *obj,float newValue)
{
	int16_t pointer=obj->Buf_Pointer;
	float outValue;
	
	obj->buf_sum +=newValue;
	obj->data_count++;
	if(obj->data_count>obj->FIFO_SIZE)
	{
		obj->data_count=obj->FIFO_SIZE;
		obj->buf_sum-=obj->FIFO_BUF[pointer];
	}
	
	obj->FIFO_BUF[pointer]=newValue;
	obj->Buf_Pointer++;	
	if(obj->Buf_Pointer==obj->FIFO_SIZE)
		obj->Buf_Pointer=0;
	
	outValue=obj->buf_sum/obj->data_count;
	
	return outValue;
}

void Ramp_Init(float slope,float dt)
{
  uint8_t i;
  for(i=0;i<2;i++)
  {
    chassis_ramp[i].output = 0;
    chassis_ramp[i].slope = slope;  
  }
  ramp_time.dt = dt;
}

void Ramp_Calculate(ramp_struct *obj,float target)
{
  if(target > 0)
  {
    if(obj->output < target)   
      obj->output += obj->slope*ramp_time.dt;   
    if(obj->output > target) 
      obj->output = target;
  }
  else if(target < 0)
  {
    if(obj->output > target)
      obj->output -= obj->slope*ramp_time.dt;
    if(obj->output < target)
      obj->output = target;
  }
  else 
  {
    if(obj->output > 5.0f)
      obj->output -= 10.0f;
    else if(obj->output < -5.0f)
      obj->output += 10.0f;
    else obj->output = 0;
  }
}
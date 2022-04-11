#include "timer.h"

TIME ctrl_time={0,0,0,0};


void get_dt_in_micros(TIME *time)
{
	time->time_now = TIM5->CNT;
	if(time->time_now <= time->time_last-0xFFFF)
	{
		time->dt = (float)(time->time_now + (0xffffffff - time->time_last));
		time->time_last=time->time_now;
	}
	else if(time->time_now > time->time_last)
	{
		time->dt = (float)(time->time_now - time->time_last)/1000.0f;
		time->time_last=time->time_now;
	}
}

void get_dt_in_millis(TIME *time)
{
	time->time_now = TIM5->CNT;
	if(time->time_now <= time->time_last-0xFFFF)
	{
		time->dt = (float)(time->time_now + (0xffffffff - time->time_last))/1000.0f;
		time->time_last=time->time_now;
	}
	else if(time->time_now > time->time_last)
	{
		time->dt = (float)(time->time_now - time->time_last)/1000.0f;
		time->time_last=time->time_now;
	}
}

void get_dt_in_seconds(TIME *time)
{
	time->time_now=TIM5->CNT;
	if(time->time_now <= time->time_last) 
	{
		time->dt = (float)(time->time_now + (0xffffffff - time->time_last))/1000000.0f;
	}
	else 
	{
		time->dt = (float)(time->time_now - time->time_last)/1000000.0f;
	}
	time->time_last=time->time_now;
}

void calculate_dt_in_micros(uint32_t time_now,uint32_t time_last,uint32_t *dt)
{

	if(time_now <= time_last)
		*dt = time_now + (0xffffffff- time_last);
	else
		*dt = time_now - time_last;
}

void calculate_dt_in_mills(uint32_t time_now,uint32_t time_last,float *dt)
{
	if(time_now <= time_last)
		*dt = (float)(time_now + (0xffffffff- time_last))/1000.0f;
	else
		*dt = (float)(time_now - time_last)/1000.0f;
}

void calculate_dt_in_seconds(uint32_t time_now,uint32_t time_last,float *dt)
{
	if(time_now <= time_last)
		*dt = (float)(time_now + (0xffffffff- time_last))/1000000.0f;
	else
		*dt = (float)(time_now - time_last)/1000000.0f;
}

void get_totaltime_in_seconds(TIME *time)
{
	time->time_now = TIM5->CNT;
	if(time->time_now <= time->time_last-0xFFFF)
	{
		time->dt =(float)(time->time_now + 0xffffffff - time->time_last)/1000000.0f;
		time->time_last=time->time_now;
		time->time_total += time->dt;
	}
	else if(time->time_now > time->time_last)
	{
		time->dt =(float)(time->time_now - time->time_last)/1000000.0f;
		time->time_last=time->time_now;
		time->time_total += time->dt;
	}
}



#include "sr04.h"

uint8_t 	TIM5CH1_CAPTURE_STA=0;	//输入捕获状态		    				
uint32_t	TIM5CH1_CAPTURE_VAL;	//输入捕获值(TIM2/TIM5是32位)
uint32_t    TIM5CH1_CAPTURE_VAL_start=0;


//定时器更新中断（计数溢出）中断处理回调函数， 该函数在HAL_TIM_IRQHandler中会被调用
void my_ic_update_callback(TIM_HandleTypeDef *htim)//更新中断（溢出）发生时执行
{
	if((TIM5CH1_CAPTURE_STA&0X80)==0)//成功捕获后不执行
	{
			if(TIM5CH1_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((TIM5CH1_CAPTURE_STA&0X3F)==0X3F)//高电平太长了，STA寄存器满了
				{
					TIM5CH1_CAPTURE_STA|=0X80;		//标记成功捕获了一次，两个中断都拟停止了
					TIM5CH1_CAPTURE_VAL=0XFFFFFFFF;
				}else TIM5CH1_CAPTURE_STA++;
			}	 
	}		
}

//定时器输入捕获中断处理回调函数，该函数在HAL_TIM_IRQHandler中会被调用
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//捕获中断发生时执行
{
	if((TIM5CH1_CAPTURE_STA&0X80)==0)//成功捕获后不执行
	{
		if(TIM5CH1_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
			{	  			
				TIM5CH1_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽，，两个中断都拟停止了
                TIM5CH1_CAPTURE_VAL=HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_1);//获取当前的捕获值.
                TIM_RESET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_1);   //一定要先清除原来的设置！！
                TIM_SET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_1,TIM_ICPOLARITY_RISING);//配置TIM2通道1上升沿捕获
			}else  								//还未开始,第一次捕获上升沿
			{
				TIM5CH1_CAPTURE_STA=0;			//清空
				TIM5CH1_CAPTURE_VAL=0;
				//TIM5CH1_CAPTURE_VAL_start=HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_1);//获取当前的捕获值.
				TIM5CH1_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
				__HAL_TIM_DISABLE(&htim5);        //关闭定时器2
				__HAL_TIM_SET_COUNTER(&htim5,0);//重置寄存器值为0
				TIM_RESET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_1);   //一定要先清除原来的设置！！
				TIM_SET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_1,TIM_ICPOLARITY_FALLING);//定时器2通道1设置为下降沿捕获
				__HAL_TIM_ENABLE(&htim5);//使能定时器2
			}		    
	}		
}



double get_distance(uint8_t side)
{
	    long long temp=0;  
		double s = 0;
	
	    if(TIM5CH1_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
		{
			temp=TIM5CH1_CAPTURE_STA&0X3F;  //获取溢出次数
			temp*=0XFFFFFFFF;		 	    //溢出时间总和   溢出次数*arr
			temp=temp+TIM5CH1_CAPTURE_VAL+TIM5CH1_CAPTURE_VAL_start;      //得到总的高电平时间us  
			//if(temp-last_temp>)
			TIM5CH1_CAPTURE_STA=0;          //开启下一次捕获
			TIM5CH1_CAPTURE_VAL_start=0;
			s = 340.0*temp/20000;				//计算距离
		}
		
//		//下面去除毛刺
//		if((s-last_distance>0.5)||(last_distance-s>0.5))//太陡
//		{
//			if(last_distance)//不是刚上电，确实是毛刺
//			{
//				return last_distance;
//			}
//			last_distance=s;
//			return s;
//		}
//		last_distance=s;
		return s;
}

void sr04_init()
{
  	HAL_TIM_IC_Start_IT(&htim5,TIM_CHANNEL_1);   //开启TIM5的捕获通道1，并且开启捕获中断
	__HAL_TIM_ENABLE_IT(&htim5,TIM_IT_UPDATE);   //使能更新中断
}



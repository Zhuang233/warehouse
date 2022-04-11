#include "encoder.h"
#include "timer.h"

void TIM5_Encoder_init(void)//编码器模式
{
		GPIO_InitTypeDef         GPIO_InitStructure; 
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef        TIM_ICInitStructure;
 
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5 , ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
		
  
		GPIO_PinAFConfig(GPIOH, GPIO_PinSource10, GPIO_AF_TIM5);//ch1
		GPIO_PinAFConfig(GPIOH, GPIO_PinSource11, GPIO_AF_TIM5);//ch2
		//编码器A,B项------
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_Init(GPIOH, &GPIO_InitStructure);
		//电源控制---------------------------
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_3 | GPIO_Pin_5;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_Init(GPIOH, &GPIO_InitStructure);
		GPIO_SetBits(GPIOH,GPIO_Pin_2);
		GPIO_SetBits(GPIOH,GPIO_Pin_4);
		GPIO_SetBits(GPIOH,GPIO_Pin_3);
		GPIO_SetBits(GPIOH,GPIO_Pin_5);
		//------------------------------------
		
		TIM_TimeBaseStructure.TIM_Period = 60000; 
    TIM_TimeBaseStructure.TIM_Prescaler = 0; 
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 
 
    TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10; 
    TIM_ICInit(TIM5, &TIM_ICInitStructure);
    
		TIM_ClearFlag(TIM5, TIM_FLAG_Update); 
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE); 
    TIM5->CNT = 30000;
    TIM_Cmd(TIM5, ENABLE);
}

void TIM8_Encoder_init(void)//编码器模式
{
		GPIO_InitTypeDef         GPIO_InitStructure; 
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef        TIM_ICInitStructure;
 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);
  
    GPIO_PinAFConfig(GPIOI,GPIO_PinSource5,GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOI,GPIO_PinSource6,GPIO_AF_TIM8);
 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6; //GPIOB0,GPIOB1
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOI,&GPIO_InitStructure); 
 
    TIM_TimeBaseStructure.TIM_Period = 60000; 
    TIM_TimeBaseStructure.TIM_Prescaler = 0; 
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); 
 
    TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10; 
    TIM_ICInit(TIM8, &TIM_ICInitStructure);
    
		TIM_ClearFlag(TIM8, TIM_FLAG_Update); 
    TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE); 
    TIM8->CNT = 30000;
    TIM_Cmd(TIM8, ENABLE);
}


void TIM3_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM5时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM5
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

//编码器信息
//编号0对应左边的电机，1对应右边电机
//顺时针转计数增加

uint16_t encoder_data[2];
uint16_t encoder_data_last[2]={30000,30000};
int16_t fric_spd[2];
int16_t spd_last[2]={0,0};
void pid_spd_ctrl(int16_t spd_desired,int16_t spd_actual);
//定时器2中断服务函数
//void TIM3_IRQHandler(void)
//{
//	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
//	{
//		//LED1=!LED1;//DS1翻转
//		encoder_data[0]=TIM8->CNT;
//		encoder_data[1]=TIM5->CNT;
//		
//		for(int i=0;i<2;i++)
//		{
//			fric_spd[i]=encoder_data[i]-encoder_data_last[i];
//	
//			if(fric_spd[i]>30000)
//			{
//				fric_spd[i]-=60000;
//			}
//			else if(fric_spd[i]<-30000)
//			{
//				fric_spd[i]+=60000;
//			}
//			if(fric_spd[i]>700||fric_spd[i]<-700)
//				fric_spd[i]=spd_last[i];
//			else
//				spd_last[i]=fric_spd[i];
//			encoder_data_last[i]=encoder_data[i];
//		}
//		
//	}
//	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
//}

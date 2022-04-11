#include "sr04.h"

uint8_t 	TIM5CH1_CAPTURE_STA=0;	//���벶��״̬		    				
uint32_t	TIM5CH1_CAPTURE_VAL;	//���벶��ֵ(TIM2/TIM5��32λ)
uint32_t    TIM5CH1_CAPTURE_VAL_start=0;


//��ʱ�������жϣ�����������жϴ���ص������� �ú�����HAL_TIM_IRQHandler�лᱻ����
void my_ic_update_callback(TIM_HandleTypeDef *htim)//�����жϣ����������ʱִ��
{
	if((TIM5CH1_CAPTURE_STA&0X80)==0)//�ɹ������ִ��
	{
			if(TIM5CH1_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM5CH1_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫���ˣ�STA�Ĵ�������
				{
					TIM5CH1_CAPTURE_STA|=0X80;		//��ǳɹ�������һ�Σ������ж϶���ֹͣ��
					TIM5CH1_CAPTURE_VAL=0XFFFFFFFF;
				}else TIM5CH1_CAPTURE_STA++;
			}	 
	}		
}

//��ʱ�����벶���жϴ���ص��������ú�����HAL_TIM_IRQHandler�лᱻ����
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//�����жϷ���ʱִ��
{
	if((TIM5CH1_CAPTURE_STA&0X80)==0)//�ɹ������ִ��
	{
		if(TIM5CH1_CAPTURE_STA&0X40)		//����һ���½��� 		
			{	  			
				TIM5CH1_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ�����������ж϶���ֹͣ��
                TIM5CH1_CAPTURE_VAL=HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_1);//��ȡ��ǰ�Ĳ���ֵ.
                TIM_RESET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_1);   //һ��Ҫ�����ԭ�������ã���
                TIM_SET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_1,TIM_ICPOLARITY_RISING);//����TIM2ͨ��1�����ز���
			}else  								//��δ��ʼ,��һ�β���������
			{
				TIM5CH1_CAPTURE_STA=0;			//���
				TIM5CH1_CAPTURE_VAL=0;
				//TIM5CH1_CAPTURE_VAL_start=HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_1);//��ȡ��ǰ�Ĳ���ֵ.
				TIM5CH1_CAPTURE_STA|=0X40;		//��ǲ�����������
				__HAL_TIM_DISABLE(&htim5);        //�رն�ʱ��2
				__HAL_TIM_SET_COUNTER(&htim5,0);//���üĴ���ֵΪ0
				TIM_RESET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_1);   //һ��Ҫ�����ԭ�������ã���
				TIM_SET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_1,TIM_ICPOLARITY_FALLING);//��ʱ��2ͨ��1����Ϊ�½��ز���
				__HAL_TIM_ENABLE(&htim5);//ʹ�ܶ�ʱ��2
			}		    
	}		
}



double get_distance(uint8_t side)
{
	    long long temp=0;  
		double s = 0;
	
	    if(TIM5CH1_CAPTURE_STA&0X80)        //�ɹ�������һ�θߵ�ƽ
		{
			temp=TIM5CH1_CAPTURE_STA&0X3F;  //��ȡ�������
			temp*=0XFFFFFFFF;		 	    //���ʱ���ܺ�   �������*arr
			temp=temp+TIM5CH1_CAPTURE_VAL+TIM5CH1_CAPTURE_VAL_start;      //�õ��ܵĸߵ�ƽʱ��us  
			//if(temp-last_temp>)
			TIM5CH1_CAPTURE_STA=0;          //������һ�β���
			TIM5CH1_CAPTURE_VAL_start=0;
			s = 340.0*temp/20000;				//�������
		}
		
//		//����ȥ��ë��
//		if((s-last_distance>0.5)||(last_distance-s>0.5))//̫��
//		{
//			if(last_distance)//���Ǹ��ϵ磬ȷʵ��ë��
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
  	HAL_TIM_IC_Start_IT(&htim5,TIM_CHANNEL_1);   //����TIM5�Ĳ���ͨ��1�����ҿ��������ж�
	__HAL_TIM_ENABLE_IT(&htim5,TIM_IT_UPDATE);   //ʹ�ܸ����ж�
}



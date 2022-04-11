
#include "usart.h"
#include "ANO-Tech.h"

#define ANO_U2
// Note: v2.6 is more stable than v4.6
/////////////////////////////////////////////////////////////////////////////////////
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

dt_flag_t f;					//?????????
u8 data_to_send[50];	//??????
u8 checkdata_to_send,checksum_to_send;

/**
  **********************************
  * @brief  ����1����1���ֽ�
  * @param  c����������
  * @retval None
  **********************************
*/
void usart1_send_char(u8 c)
{
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); //ѭ������,ֱ���������   
	USART_SendData(USART1, c);
}

/**
  **********************************
  * @brief  �������ݸ�����������λ�����(V2.6�汾)
  * @param  fun:������. 0XA0~0XAF  
						data:���ݻ�����,���28�ֽ�!!
						len:data����Ч���ݸ���
  * @retval None
  **********************************
*/
void usart1_niming_report(u8 fun, u8*data, u8 len)
{
	u8 send_buf[32];
	u8 i;
	if (len>28)return;	//���28�ֽ����� 
	send_buf[len + 3] = 0;	//У��������
	send_buf[0] = 0X88;	//֡ͷ
	send_buf[1] = fun;	//������
	send_buf[2] = len;	//���ݳ���
	for (i = 0;i<len;i++)send_buf[3 + i] = data[i];			//��������
	for (i = 0;i<len + 3;i++)send_buf[len + 3] += send_buf[i];	//����У���	
	for (i = 0;i<len + 4;i++)usart1_send_char(send_buf[i]);	//�������ݵ�����1 
}

/**
  **********************************
  * @brief  ���ͼ��ٶȴ��������ݺ�����������
  * @param  aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ  
						gyrox,gyroy,gyroz:x,y,z�������������������ֵ
  * @retval None
  **********************************
*/
void mpu6050_send_data(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz)
{
	u8 tbuf[12];
	tbuf[0] = (aacx >> 8) & 0XFF;
	tbuf[1] = aacx & 0XFF;
	tbuf[2] = (aacy >> 8) & 0XFF;
	tbuf[3] = aacy & 0XFF;
	tbuf[4] = (aacz >> 8) & 0XFF;
	tbuf[5] = aacz & 0XFF;
	tbuf[6] = (gyrox >> 8) & 0XFF;
	tbuf[7] = gyrox & 0XFF;
	tbuf[8] = (gyroy >> 8) & 0XFF;
	tbuf[9] = gyroy & 0XFF;
	tbuf[10] = (gyroz >> 8) & 0XFF;
	tbuf[11] = gyroz & 0XFF;
	usart1_niming_report(0XA1, tbuf, 12);//�Զ���֡,0XA1
}

/**
  **********************************
  * @brief  ͨ������1�ϱ���������̬���ݸ�����
  * @param  aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
						gyrox,gyroy,gyroz:x,y,z�������������������ֵ
						roll:�����.��λ0.01�ȡ� -18000 -> 18000 ��Ӧ -180.00  ->  180.00��
						pitch:������.��λ 0.01�ȡ�-9000 - 9000 ��Ӧ -90.00 -> 90.00 ��
						yaw:�����.��λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
  * @retval None
  **********************************
*/
void usart1_report_imu(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz, short roll, short pitch, short yaw)
{
	u8 tbuf[28];
	u8 i;
	for (i = 0;i<28;i++)tbuf[i] = 0;//��0
	tbuf[0] = (aacx >> 8) & 0XFF;
	tbuf[1] = aacx & 0XFF;
	tbuf[2] = (aacy >> 8) & 0XFF;
	tbuf[3] = aacy & 0XFF;
	tbuf[4] = (aacz >> 8) & 0XFF;
	tbuf[5] = aacz & 0XFF;
	tbuf[6] = (gyrox >> 8) & 0XFF;
	tbuf[7] = gyrox & 0XFF;
	tbuf[8] = (gyroy >> 8) & 0XFF;
	tbuf[9] = gyroy & 0XFF;
	tbuf[10] = (gyroz >> 8) & 0XFF;
	tbuf[11] = gyroz & 0XFF;
	tbuf[18] = (roll >> 8) & 0XFF;
	tbuf[19] = roll & 0XFF;
	tbuf[20] = (pitch >> 8) & 0XFF;
	tbuf[21] = pitch & 0XFF;
	tbuf[22] = (yaw >> 8) & 0XFF;
	tbuf[23] = yaw & 0XFF;
	usart1_niming_report(0XAF, tbuf, 28);//�ɿ���ʾ֡,0XAF
}

/**
  **********************************
  * @brief  �������ݸ���������վ���(V4.0�汾)
  * @param  data:���ݻ�����,���28�ֽ�!!
						len:data����Ч���ݸ���
  * @retval None
  **********************************
*/
void ANO_DT_Send_Data(u8 *dataToSend , u8 length)
{
#if defined ANO_U1
	u8 i;
	if (length>28)return;	//���28�ֽ����� 
	for (i = 0;i<length;i++)usart1_send_char(dataToSend[i]);	//�������ݵ�����1

#elif defined ANO_U2
	Usart2_Send(data_to_send, length);
#elif defined ANO_U4
	Uart4_Send(data_to_send, length);

#endif
}

static void ANO_DT_Send_Check(u8 head, u8 check_sum)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	
	u8 sum = 0;
	for(u8 i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	ANO_DT_Send_Data(data_to_send, 7);
}

static void ANO_DT_Send_Msg(u8 id, u8 data)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEE;
	data_to_send[3]=2;
	data_to_send[4]=id;
	data_to_send[5]=data;
	
	
	u8 sum = 0;
	for(u8 i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	ANO_DT_Send_Data(data_to_send, 7);
}


/**
  **********************************
  * @brief  ���ͼ��ٶȴ��������ݺ�����������
  * @param  aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ  
						gyrox,gyroy,gyroz:x,y,z�������������������ֵ
  * @retval None
  **********************************
*/
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
/////////////////////////////////////////
	_temp = 0;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
//void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z)
//{
//	u8 _cnt=0;
//	vs16 _temp;
//	u8 sum = 0;
//	u8 i;
//	
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0x02;
//	data_to_send[_cnt++]=0;
//	
//	_temp = a_x;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = a_y;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = a_z;	
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	
//	_temp = g_x;	
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = g_y;	
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = g_z;	
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	
//	data_to_send[3] = _cnt-4;
//	
//	
//	for(i=0;i<_cnt;i++)
//		sum += data_to_send[i];
//	data_to_send[_cnt++] = sum;
//	
//	ANO_DT_Send_Data(data_to_send, _cnt);
//}

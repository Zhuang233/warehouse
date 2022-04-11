#include "decode.h"
#include "math.h"
#include "RemoteControl.h"

int pitch_offset=80;

//���ڲ����л����˲�����������������������������������������������
float Auto_pitch_ref=0;         //����ʱ��̨����ֵ (-240)(y)
float Auto_yaw_ref=0;           //                 (-320)(x)

u8 Auto_aim_ready_flag=0;       //0->no 1->yes
u8 decode_done=0;              	//������ɱ�־

extern RC_Ctl_t RC_CtrlData;
extern u8 inputmode;

//����-------------------------------------------------------------------------
//����У��λ
u8 Add_CRC(u8 InputBytes[],u8 data_lenth)
{
	u8 byte_crc=0;
	for(u8 i=0;i<data_lenth;i++)
	{
    	byte_crc+=InputBytes[i];
	}
  	return byte_crc;
}

//����ƴװ
u16 Data_assemble(u8 *LData,u8 *HData)
{
	u16 DATA=*HData;
	DATA=(DATA<<8)|(*LData);
	return DATA;
}

//���ݽ���
u16 x_Rec_Data;			//��ʵ����
u16 y_Rec_Data;	
u16 Data_Decode(u8 *rc_buf)
{		
		u16 status;					//0x01 -> У��ʧ��
												//0x02 -> ��ʧ֡ͷ
												//0x03 -> id����
		
		u8 data_length = rc_buf[1];
  	u8 data_id     = rc_buf[2];
  	u8 data_crc    = Add_CRC(rc_buf,data_length-1);
	
  	if(rc_buf[0]!=0xFF)
  	{
    	status=0x02;//data lost the frame header.
			decode_done=1;
    	return status;
  	}
  	if(data_crc!=rc_buf[data_length-1])
  	{
    	status=0x01;//data verify error
			decode_done=1;
    	return status;
  	}
  	if(data_id!=0x02) 
  	{
  		status=0x03;//data x_y property error
			decode_done=1;
  		return status;
  	};

  	//receive non error
		status=0;
		
		x_Rec_Data=Data_assemble(&rc_buf[3],&rc_buf[4]);
		y_Rec_Data=Data_assemble(&rc_buf[5],&rc_buf[6]);
		
		Auto_yaw_ref   = x_Rec_Data - 320;
		Auto_pitch_ref = y_Rec_Data - 180 - pitch_offset;
			
		if(x_Rec_Data>800 || y_Rec_Data>800)//�ж϶�ʧĿ�꣬���ܴ���ֵ
			Auto_aim_ready_flag=0,pid_yawRate.integ=0;
		else																//û�ж�ʧĿ��
			Auto_aim_ready_flag=1;
				
		decode_done=1;
		return status;
} 



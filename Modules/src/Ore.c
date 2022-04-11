#include "Ore.h"
#include "Key.h"
#include "motor.h"

#define SPINSPD 500
uint8_t fetch_stage = 0;
uint8_t fetch_flag = 0;
uint8_t fetch_which = 0;
uint8_t ore_suckin_flag = 0;


//Zȡ�������� X������
void ONE_Key_Fetch(void)  //open--600 close--370
{
//  if(fetch_flag == BIG)
//  {
//    if(Key_Check_Press(&Keys.KEY_Z))
//    {
//      if(fetch_stage == 0)
//        Ore_Suckin();
//    }
//    else if(Key_Check_Press(&Keys.KEY_X))
//    {
//      
//    }
//  }
//  else if(fetch_flag == SMALL)
//  {
//    if(Key_Check_Press(&Keys.KEY_Z))
//    {
//      
//    }
//    else if(Key_Check_Press(&Keys.KEY_X))
//    {
//      
//    }
//  }
  if(Key_Check_Press(&Keys.KEY_Z))  
  {
    fetch_which = 1;
    fetch_stage = 1;
  }
  else if(Key_Check_Press(&Keys.KEY_X))
  {
    fetch_which = 2;
    fetch_stage = 1;
  }
  
  if(fetch_stage == 0)
    fetch_which = 0;
  
  if(fetch_which == 1)
  {
    if(fetch_stage == 1)
    { 
      if(fetch_flag == BIG)
      {
        Ore_Suckin();
      }
      else if(fetch_flag == SMALL)
      {
    
      }
    }
  }
}

//Z�һ����ϵĿ� X�һ��������Ŀ�
void ONE_Key_Exchange(void)
{
  
}


//C��x��ת(����ĵ����ʱ��ת������ĵ��˳ʱ��ת) V��y��ת(����ĵ��˳ʱ��ת������ĵ����ʱ��ת)
void Ore_Spin(void)  //������ʱ��ת��������ת
{
  if(Key_Check_Hold(&Keys.KEY_C))
  {
    motor_msg[8].speed_desired = SPINSPD;
    motor_msg[9].speed_desired = -SPINSPD;
  }
  else if(Key_Check_Hold(&Keys.KEY_V))
  {
    motor_msg[8].speed_desired = -SPINSPD;
    motor_msg[9].speed_desired = SPINSPD;
  }
  else
  {
    Stop_Motor(&motor_msg[8]);
    Stop_Motor(&motor_msg[9]);
  }
}

void Ore_Suckin(void)
{
  static bool suck_first_run = true;
  
  if(suck_first_run == true)
  {
    suck_first_run = false;
    motor_msg[8].angle = 0;
    motor_msg[8].angle_last = 0;
    motor_msg[8].turns = 0;
  }
  
  if(motor_msg[8].angle < 70000)
    motor_msg[8].speed_desired = motor_msg[9].speed_desired = SPINSPD;
  else 
  {
    Stop_Motor(&motor_msg[8]);
    Stop_Motor(&motor_msg[9]);
    suck_first_run = true;
    fetch_stage = 2;
  }
}

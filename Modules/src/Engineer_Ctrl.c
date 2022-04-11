#include "main.h"
#include "sys.h"
#include "stdlib.h"
#include "RemoteControl.h"
#include "math.h"
#include "motor.h"
#include "pid.h"
#include "can.h"
#include "Key.h"
#include "ANO_Link.h"
#include "Engineer_Ctrl.h"
#include "BaseCalculate.h"

//底盘电机
#define PID_SPEED_KP	2.5f//2.5f
#define PID_SPEED_KI	2.5f//2.5f
#define PID_SPEED_KD	0.0f//0.0f

//底盘陀螺仪
#define PID_CHASSIS_KP 	20.0f
#define PID_CHASSIS_KI 	0.0f
#define PID_CHASSIS_KD 	0.0f

//抬升
#define PID_LIFT_SPD_KP 0.0f	//8.0f
#define PID_LIFT_SPD_KI 0.0f
#define PID_LIFT_SPD_KD 0.0f
#define PID_LIFT_POS_KP 0.0f
#define PID_LIFT_POS_KI 0.0f
#define PID_LIFT_POS_KD 0.0f
//同步
#define PID_LIFT_SYN_KP 0.0f
#define PID_LIFT_SYN_KI 0.0f
#define PID_LIFT_SYN_KD 0.0f

//翻转
#define PID_FLIP_SPD_KP 1.0f  //4.0
#define PID_FLIP_SPD_KI 0.5f  //1.0
#define PID_FLIP_SPD_KD 0.0f
#define PID_FLIP_POS_KP 0.3f  //0.3
#define PID_FLIP_POS_KI 0.0f
#define PID_FLIP_POS_KD 0.01f //0.01

//伸缩
#define PID_STRE_SPD_KP 3.0f
#define PID_STRE_SPD_KI 2.0f
#define PID_STRE_SPD_KD 0.0f
#define PID_STRE_POS_KP 0.0f
#define PID_STRE_POS_KI 0.0f
#define PID_STRE_POS_KD 0.0f

//转矿
#define PID_SPIN_SPD_KP 3.0f
#define PID_SPIN_SPD_KI 2.0f
#define PID_SPIN_SPD_KD 0.0f

//test parameter
#define PID_TEST_SPD_KP 4.0f
#define PID_TEST_SPD_KI 1.0f
#define PID_TEST_SPD_KD 0.0f

#define PID_TEST_POS_KP 0.3f
#define PID_TEST_POS_KI 0.0f
#define PID_TEST_POS_KD 0.01f

#define PID_TEST_SYN_KP 0.0f
#define PID_TEST_SYN_KI 0.0f
#define PID_TEST_SYN_KD 0.0f

PidObject pid_testmoto_spd[2];
PidObject pid_testmoto_pos[2];
PidObject pid_sync[2];
//

PidObject pid_chassis_spd[4];//chassis motor speed pid 3508

PidObject pid_chassis_gyro;		//chassis gyro pid

PidObject pid_flip_spd[2];    //flip motor speed pid 3508
PidObject pid_flip_pos[2];    //flip motor position pid 

PidObject pid_stretch_spd[2];  //stretch motor speed pid 2006
PidObject pid_stretch_pos[2];  //stretch motor position pid

PidObject pid_lift_spd[4];    //lift motor speed pid 3508
PidObject pid_lift_pos[4];    //lift motor position pid
PidObject pid_lift_syn[4];    //four lift motors sync pid 

PidObject pid_spin_spd[2];    //spin ore motor spd pid 2006



TIME ctrl_time={0,0,0,0};

//uint32_t fetcher_time_now=0,fetcher_time_last=0;
//float ctrl_time.dt=0.02;

int16_t Comp[2]= {-2300,-1700};

extern RC_Ctl_t RC_CtrlData;
extern uint8_t input_mode;
extern bool CAN1_send_current_flag;
extern bool CAN2_send_current_flag;
extern KEYS Keys;

extern int32_t pos1,pos2,pos3,pos4,pos5;

void float_constraint(float *data,float max,float min)
{
	if(*data>max)*data=max;
	else if(*data<min)*data=min;
}
void int16_constraint(int16_t *data,int16_t max,int16_t min)
{
	if(*data>max)*data=max;
	else if(*data<min)*data=min;
}

void Engineer_Pid_Init(void)
{
	uint8_t i;
	for(i=0;i<4;i++)
	{
		pidInit(&pid_chassis_spd[i],0,PID_SPEED_KP,PID_SPEED_KI,PID_SPEED_KD,0.002);    //chassis speed
		pidSetIntegLimit(&pid_chassis_spd[i],0,0);
    
    pidInit(&pid_lift_spd[i],0,PID_LIFT_SPD_KP,PID_LIFT_SPD_KI,PID_LIFT_POS_KD,0.002); //lift speed
    pidSetIntegLimit(&pid_lift_spd[i],0,0);
    pidInit(&pid_lift_pos[i],0,PID_LIFT_POS_KP,PID_LIFT_POS_KI,PID_LIFT_POS_KD,0.002); //lift pos
    pidSetIntegLimit(&pid_lift_pos[i],0,0);
		pidInit(&pid_lift_syn[i],0,PID_LIFT_SYN_KP,PID_LIFT_SYN_KI,PID_LIFT_SYN_KD,0.002);  //lift sync
		pidSetIntegLimit(&pid_lift_syn[i],0,0);
	}

  for(i=0;i<2;i++)
	{
    pidInit(&pid_flip_spd[i],0,PID_FLIP_SPD_KP,PID_FLIP_SPD_KI,PID_FLIP_SPD_KD,0.002);  //flip speed
    pidSetIntegLimit(&pid_flip_spd[i],2000,-2000);
    pidInit(&pid_flip_pos[i],0,PID_FLIP_POS_KP,PID_FLIP_POS_KI,PID_FLIP_POS_KD,0.002);  //flip pos
    pidSetIntegLimit(&pid_flip_pos[i],0,0);
    
    pidInit(&pid_stretch_spd[i],0,PID_STRE_SPD_KP,PID_STRE_SPD_KI,PID_STRE_SPD_KD,0.002); //stretch speed
    pidSetIntegLimit(&pid_stretch_spd[i],10000,-10000);
    pidInit(&pid_stretch_pos[i],0,PID_STRE_POS_KP,PID_STRE_POS_KI,PID_STRE_POS_KD,0.002); //stretch pos
    pidSetIntegLimit(&pid_stretch_pos[i],0,0);
    
    pidInit(&pid_spin_spd[i],0,PID_SPIN_SPD_KP,PID_SPIN_SPD_KI,PID_SPIN_SPD_KD,0.002);    //spin speed
    pidSetIntegLimit(&pid_spin_spd[i],10000,-10000);
  }

}

void Testmoto_PID_Init(void)
{
  pidInit(&pid_testmoto_spd[0],0,PID_TEST_SPD_KP,PID_TEST_SPD_KI,PID_TEST_SPD_KD,0.002);
  pidInit(&pid_testmoto_spd[1],0,PID_TEST_SPD_KP,PID_TEST_SPD_KI,PID_TEST_SPD_KD,0.002);
  pidInit(&pid_testmoto_pos[0],0,PID_TEST_POS_KP,PID_TEST_POS_KI,PID_TEST_POS_KD,0.002);
  pidInit(&pid_testmoto_pos[1],0,PID_TEST_POS_KP,PID_TEST_POS_KI,PID_TEST_POS_KD,0.002);
  pidInit(&pid_sync[0],0,PID_TEST_SYN_KP,PID_TEST_SYN_KI,PID_TEST_SYN_KD,0.002);
  pidInit(&pid_sync[1],0,PID_TEST_SYN_KP,PID_TEST_SYN_KI,PID_TEST_SYN_KD,0.002);
}

void Zeropoint_Init()
{
  uint8_t i;
  int16_t temp=0;
  
  TIM4->CCR1 = 600;       //夹子张开
  
  //伸缩校准
  if(HAL_GPIO_ReadPin(SWITCH_STRETCH_GPIO_Port,SWITCH_STRETCH_Pin) != GPIO_PIN_SET)
  {
    motor_msg[6].speed_desired = -500;
    motor_msg[7].speed_desired = 500;
    
    while(HAL_GPIO_ReadPin(SWITCH_STRETCH_GPIO_Port,SWITCH_STRETCH_Pin) != GPIO_PIN_SET)
    {
      for(i=0;i<2;i++)
      {        
        pidSetDesired(&pid_stretch_spd[i],motor_msg[i+6].speed_desired);
        pidSetDt(&pid_stretch_spd[i],0.002);
        temp = pidUpdate(&pid_stretch_spd[i],motor_msg[i+6].speed_actual);
        int16_constraint(&temp,10000,-10000);
        motor_msg[i+6].given_current = temp;
      }
      CAN1_send_current_flag = true;
      ANO_Send_UserData(motor_msg[6].speed_actual,0,0,0,0,0);
    }
  }
  Stop_Motor(&motor_msg[6]);
  Stop_Motor(&motor_msg[7]);
  CAN1_send_current_flag = true;
  while(motor_msg[6].speed_actual != 0 && motor_msg[7].speed_actual != 0);
  Set_MotorZeropoint(&motor_msg[6]);
  Set_MotorZeropoint(&motor_msg[7]);
  
  //翻转校准
  if(HAL_GPIO_ReadPin(SWITCH_FLIP_GPIO_Port,SWITCH_FLIP_Pin) != GPIO_PIN_SET)
  {
    motor_msg[4].speed_desired = -500;
		motor_msg[5].speed_desired = 500;

    while(HAL_GPIO_ReadPin(SWITCH_FLIP_GPIO_Port,SWITCH_FLIP_Pin) != GPIO_PIN_SET)
    {
      for(i=0;i<2;i++)
      {        
        pidSetDesired(&pid_flip_spd[i],motor_msg[i+4].speed_desired);
        pidSetDt(&pid_flip_spd[i],0.002);
        temp = pidUpdate(&pid_flip_spd[i],motor_msg[i+4].speed_actual);
        int16_constraint(&temp,10000,-10000);
        motor_msg[i+4].given_current = temp;
      }
      CAN1_send_current_flag = true;
    }
  }
  Stop_Motor(&motor_msg[4]);
  Stop_Motor(&motor_msg[5]);
  CAN1_send_current_flag = true;
  while(motor_msg[4].speed_actual != 0 && motor_msg[5].speed_actual != 0);
  Set_MotorZeropoint(&motor_msg[4]);
  Set_MotorZeropoint(&motor_msg[5]);
}


                   /* can1[0]~[3] 	底盘电机	
                          [4]~[5]		翻转		  
                          [6]~[7]		伸缩		  
                      can2[8]~[9]	转矿		  
                          [10]~[13]	抬升	
                   */
void Engineer_Control(void)
{
  uint8_t i;
  int16_t temp = 0;
  uint16_t avg_angle = 0;
  static uint8_t eng_first_run = 1;
  
  get_dt_in_seconds(&ctrl_time);
  if(eng_first_run)
  {
    eng_first_run = 0;
    ctrl_time.dt = 0.002;
  }
  
  for(i=0;i<4;i++)
  {
		pidSetDesired(&pid_chassis_spd[i],motor_msg[i].speed_desired);
		pidSetDt(&pid_chassis_spd[i],ctrl_time.dt);
		temp=pidUpdate(&pid_chassis_spd[i],motor_msg[i].speed_actual);
		int16_constraint(&temp,16384,-16384);
		motor_msg[i].given_current=temp;    
  }
  
  for(i=0;i<2;i++)
  {
		pidSetDesired(&pid_spin_spd[i],motor_msg[i+8].speed_desired);
		pidSetDt(&pid_spin_spd[i],ctrl_time.dt);
		temp=pidUpdate(&pid_spin_spd[i],motor_msg[i+8].speed_actual);
		int16_constraint(&temp,10000,-10000);
		motor_msg[i+8].given_current=temp;       
  }
  
  CAN1_send_current_flag = 1;
  CAN2_send_current_flag = 1;
//  avg_angle = (abs(motor_msg[0].angle) + abs(motor_msg[1].angle))/2;
  
//  if(motor_msg[0].whe_use_pid == true)
//  {
//    pidSetDesired(&pid_sync[0],avg_angle);
//    pidSetDt(&pid_sync[0],ctrl_time.dt);
//    temp = pidUpdate(&pid_sync[0],abs(motor_msg[0].angle));
//    temp *= (motor_msg[0].angle < 0 ? -1 : 1);
//    
//    pidSetDesired(&pid_testmoto_pos[0],motor_msg[0].angle_desired);
//    pidSetDt(&pid_testmoto_pos[0],ctrl_time.dt);
//    motor_msg[0].speed_desired = pidUpdate(&pid_testmoto_pos[0],motor_msg[0].angle) + temp;
//    int16_constraint(&motor_msg[0].speed_desired,2000,-2000);
//    pidSetDesired(&pid_testmoto_spd[0],motor_msg[0].speed_desired);
//    pidSetIntegLimit(&pid_testmoto_spd[0], abs(motor_msg[0].speed_desired), -abs(motor_msg[0].speed_desired));
//    pidSetDt(&pid_testmoto_spd[0],ctrl_time.dt);
//    temp = pidUpdate(&pid_testmoto_spd[0],motor_msg[0].speed_actual);
//    int16_constraint(&temp,10000,-10000);
//    motor_msg[0].given_current = temp;
//  }
//  
//  if(motor_msg[1].whe_use_pid == true)
//  {
//    pidSetDesired(&pid_sync[1],avg_angle);
//    pidSetDt(&pid_sync[1],ctrl_time.dt);
//    temp = pidUpdate(&pid_sync[1],abs(motor_msg[1].angle));
//    temp *= (motor_msg[1].angle < 0 ? -1 : 1);
//    
//    pidSetDesired(&pid_testmoto_pos[1],motor_msg[1].angle_desired);
//    pidSetDt(&pid_testmoto_pos[1],ctrl_time.dt);
//    motor_msg[1].speed_desired = pidUpdate(&pid_testmoto_pos[1],motor_msg[1].angle);
//    int16_constraint(&motor_msg[1].speed_desired,2000,-2000);
//    pidSetDesired(&pid_testmoto_spd[1],motor_msg[1].speed_desired);
//    pidSetIntegLimit(&pid_testmoto_spd[1], abs(motor_msg[1].speed_desired), -abs(motor_msg[1].speed_desired));
//    pidSetDt(&pid_testmoto_spd[1],ctrl_time.dt);
//    temp = pidUpdate(&pid_testmoto_spd[1],motor_msg[1].speed_actual);
//    int16_constraint(&temp,16384,-16384);
//    motor_msg[1].given_current = temp;
//  }
////  if(input_mode == STOP)
////    motor_msg[0].given_current = motor_msg[1].given_current = 0;
//  CAN1_send_current_flag = true;
}
/**		从左至右  	红	翻转	PF0		LED4(PG4)
					橙	夹取	PE4		LED3(PG3)
					黄	抬升	PE12	LED1(PG1)
					绿	伸缩	PB0		LED2(PG2)
					蓝	补弹	PB1		LED5(PG5)
  */

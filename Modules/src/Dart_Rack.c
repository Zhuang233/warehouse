#include "Dart_Rack.h"
#include "pid.h"
#include "timer.h"
#include "motor.h"
#include "can.h"

#define K_d_angle 0.0f  //3508同步补偿系数
#define PID_3508_SPD_KP 1.0f
#define PID_3508_SPD_KI 0.0f
#define PID_3508_SPD_KD 0.0f

#define PID_3508_POS_KP 1.0f
#define PID_3508_POS_KI 0.0f
#define PID_3508_POS_KD 0.0f

#define PID_2006_SPD_KP 3.0f
#define PID_2006_SPD_KI 2.0f
#define PID_2006_SPD_KD 0.0f

#define PID_2006_POS_KP 1.0f
#define PID_2006_POS_KI 0.0f
#define PID_2006_POS_KD 0.0f

#define PID_6020_SPD_KP 30.0f
#define PID_6020_SPD_KI 10.0f
#define PID_6020_SPD_KD 0.0f

#define PID_6020_POS_KP 1.0f
#define PID_6020_POS_KI 0.0f
#define PID_6020_POS_KD 0.0f

PidObject pid_3508_spd[2];
PidObject pid_3508_pos[2];
int32_t angle_pull=0;
int32_t angle_2006=0;
int32_t angle_6020=0;
float rc_to_6020=0.001;

PidObject pid_2006_spd;
PidObject pid_2006_pos;

PidObject pid_6020_spd;
PidObject pid_6020_pos;

TIME ctrl_time={0,0,0,0};

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

void Dart_Rack_PidInit(void)
{
  uint8_t i;
  
  for(i=0;i<2;i++)
  {
    pidInit(&pid_3508_spd[i],0,PID_3508_SPD_KP,PID_3508_SPD_KI,PID_3508_SPD_KD,0.002);
    pidSetIntegLimit(&pid_3508_spd[i],2000,-2000);
    pidInit(&pid_3508_pos[i],0,PID_3508_POS_KP,PID_3508_POS_KI,PID_3508_POS_KD,0.002);
    pidSetIntegLimit(&pid_3508_spd[i],0,0);    
  }
  
  pidInit(&pid_2006_spd,0,PID_2006_SPD_KP,PID_2006_SPD_KI,PID_2006_SPD_KD,0.002);
  pidSetIntegLimit(&pid_2006_spd,2000,-2000);
  pidInit(&pid_2006_pos,0,PID_2006_POS_KP,PID_2006_POS_KI,PID_2006_POS_KD,0.002);
  pidSetIntegLimit(&pid_2006_pos,0,0);
  
  pidInit(&pid_6020_spd,0,PID_6020_SPD_KP,PID_6020_SPD_KI,PID_6020_SPD_KD,0.002);
  pidSetIntegLimit(&pid_6020_spd,2000,-2000);
  pidInit(&pid_6020_pos,0,PID_6020_POS_KP,PID_6020_POS_KI,PID_6020_POS_KD,0.002);
  pidSetIntegLimit(&pid_6020_pos,0,0);
    
}

void Dart_Rack_Control(void)
{
  uint8_t i;
  int16_t temp = 0;
  int16_t d_angle;//两个3508 总角度差
  uint16_t avg_angle = 0;
  static uint8_t Dart_first_run = 1;
  
  get_dt_in_seconds(&ctrl_time);
  if(Dart_first_run)
  {
    Dart_first_run = 0;
    ctrl_time.dt = 0.002;
  }
  
  
  //位置
  
  
  d_angle = motor_msg[0].angle + motor_msg[1].angle;
  motor_msg[0].angle_desired = angle_pull;
  motor_msg[1].angle_desired = -angle_pull;

  motor_msg[2].angle_desired = angle_2006*0.1;
 
  motor_msg[3].angle_desired = angle_6020*rc_to_6020;
  
  for(int i=0;i<2;i++)
  {
	  pidSetDesired(&pid_3508_pos[i],motor_msg[i].angle_desired);//设定目标值
	  pidSetDt(&pid_3508_pos[i],ctrl_time.dt);//算时间差
	  temp=pidUpdate(&pid_3508_pos[i],motor_msg[i].angle);//算pid
	 // int16_constraint(&temp,10000,-10000);//限幅
	  motor_msg[i].speed_desired = temp - K_d_angle*d_angle;  //输出赋值
  }
  
  

  pidSetDesired(&pid_2006_pos,motor_msg[2].angle_desired);
  pidSetDt(&pid_2006_pos,ctrl_time.dt);
  temp=pidUpdate(&pid_2006_pos,motor_msg[2].angle);
  //int16_constraint(&temp,10000,-10000);
  motor_msg[2].speed_desired=temp;  
  
  pidSetDesired(&pid_6020_pos,motor_msg[3].angle_desired);
  pidSetDt(&pid_6020_pos,ctrl_time.dt);
  temp=pidUpdate(&pid_6020_pos,motor_msg[3].angle );
  //int16_constraint(&temp,30000,-30000);
  motor_msg[3].speed_desired=temp;
  
  
  //速度环
  for(int i=0;i<2;i++)
  {
	  pidSetDesired(&pid_3508_spd[i],motor_msg[i].speed_desired);
	  pidSetDt(&pid_3508_spd[i],ctrl_time.dt);
	  temp=pidUpdate(&pid_3508_spd[i],motor_msg[i].speed_actual);
	  int16_constraint(&temp,10000,-10000);
	  motor_msg[i].given_current=temp;  
  }

  pidSetDesired(&pid_2006_spd,motor_msg[2].speed_desired);
  pidSetDt(&pid_2006_spd,ctrl_time.dt);
  temp=pidUpdate(&pid_2006_spd,motor_msg[2].speed_actual);
  int16_constraint(&temp,10000,-10000);
  motor_msg[2].given_current=temp;  
  
  pidSetDesired(&pid_6020_spd,motor_msg[3].speed_desired);
  pidSetDt(&pid_6020_spd,ctrl_time.dt);
  temp=pidUpdate(&pid_6020_spd,motor_msg[3].speed_actual);
  int16_constraint(&temp,30000,-30000);
  motor_msg[3].given_current=temp;    

  CAN1_send_current_flag = true;  
}
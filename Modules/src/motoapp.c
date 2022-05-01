#include "motoapp.h"
#include "pid.h"
#include "timer.h"
#include "motor.h"
#include "can.h"

//底盘
#define PID_CHASSIS_SPD_KP 1.235f
#define PID_CHASSIS_SPD_KI 60.0f
#define PID_CHASSIS_SPD_KD 0.00002f

#define KP_SLOW 1.235f //tochange(低速跑时的专用pid)
#define KI_SLOW 60.0f
#define KD_SLOW 0.00002f

#define PID_CHASSIS_POS_KP 0.0f
#define PID_CHASSIS_POS_KI 0.0f
#define PID_CHASSIS_POS_KD 0.0f

#define PID_CHASSIS_YAW_SPD_KP 1.0f //只用了pos单环（yaw->wz）
#define PID_CHASSIS_YAW_SPD_KI 0.0f
#define PID_CHASSIS_YAW_SPD_KD 0.0f

#define PID_CHASSIS_YAW_POS_KP 100.0f//可能tochange (变大误差小时可变硬，注意最大值控制，不然误差大时太猛)
#define PID_CHASSIS_YAW_POS_KI 0.0f
#define PID_CHASSIS_YAW_POS_KD 0.0f

#define CHASSIS_I_LIMIT_SPD 3000
#define CHASSIS_I_LIMIT_POS 3000

#define CHASSIS_YAW_I_LIMIT_SPD 3000
#define CHASSIS_YAW_I_LIMIT_POS 3000

#define CHASSIS_VX_MAX	10000
#define CHASSIS_VX_MIN	-10000
#define CHASSIS_VY_MAX	10000
#define CHASSIS_VY_MIN	-10000
#define CHASSIS_WZ_MAX	8000  //tochange slow
#define CHASSIS_WZ_MIN	-8000

#define MAX_WHEEL_SPEED 19000.0f


//龙门架
#define K_d_angle 0.0f  //龙门架电机同步补偿系数
#define ANGLE_LIFT_MAX 500000
#define ANGLE_LIFT_MIN 0

#define PID_LIFT_SPD_KP 1.0f
#define PID_LIFT_SPD_KI 0.0f
#define PID_LIFT_SPD_KD 0.0f

#define PID_LIFT_POS_KP 0.25f
#define PID_LIFT_POS_KI 0.0f
#define PID_LIFT_POS_KD 0.0f

#define LIFT_I_LIMIT_SPD 3000
#define LIFT_I_LIMIT_POS 3000

//储球机构
#define PID_PAN_SPD_KP 1.0f
#define PID_PAN_SPD_KI 0.0f
#define PID_PAN_SPD_KD 0.0f

#define PID_PAN_POS_KP 0.1f
#define PID_PAN_POS_KI 0.0f
#define PID_PAN_POS_KD 0.0f

#define PAN_I_LIMIT_SPD 3000
#define PAN_I_LIMIT_POS 3000

#define PAN_MAX_SPD 1200


PidObject pid_chassis_spd[4];
PidObject pid_chassis_pos[4];
PidObject pid_chassis_yaw_spd;
PidObject pid_chassis_yaw_pos;


PidObject pid_lift_spd[2];
PidObject pid_lift_pos[2];

PidObject pid_pan_spd;
PidObject pid_pan_pos;

chassis_move_t chassis;
int32_t angle_lift=0;
uint8_t ramp = 3;




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
int constraint(int32_t data,int32_t max,int32_t min)
{
	if(data>max)data=max;
	else if(data<min)data=min;
	return data;
}

void motos_pid_Init()
{
	uint8_t i;
	
	//底盘
  for(i=0;i<4;i++)
  {
    pidInit(&pid_chassis_spd[i],0,PID_CHASSIS_SPD_KP,PID_CHASSIS_SPD_KI,PID_CHASSIS_SPD_KD,0.002);
    pidSetParameter(&pid_chassis_spd[i],iLimit,CHASSIS_I_LIMIT_SPD);
    pidInit(&pid_chassis_pos[i],0,PID_CHASSIS_POS_KP,PID_CHASSIS_POS_KI,PID_CHASSIS_POS_KD,0.002);
    pidSetParameter(&pid_chassis_pos[i],iLimit,CHASSIS_I_LIMIT_POS);    
  }
	
    pidInit(&pid_chassis_yaw_spd,0,PID_CHASSIS_YAW_SPD_KP,PID_CHASSIS_YAW_SPD_KI,PID_CHASSIS_YAW_SPD_KD,0.002);
    pidSetParameter(&pid_chassis_yaw_spd,iLimit,CHASSIS_YAW_I_LIMIT_SPD);
    pidInit(&pid_chassis_yaw_pos,0,PID_CHASSIS_YAW_POS_KP,PID_CHASSIS_YAW_POS_KI,PID_CHASSIS_YAW_POS_KD,0.002);
    pidSetParameter(&pid_chassis_yaw_pos,iLimit,CHASSIS_YAW_I_LIMIT_POS);
	
	//龙门架
	for(i=0;i<2;i++)
  {
    pidInit(&pid_lift_spd[i],0,PID_LIFT_SPD_KP,PID_LIFT_SPD_KI,PID_LIFT_SPD_KD,0.002);
    pidSetParameter(&pid_lift_spd[i],iLimit,LIFT_I_LIMIT_SPD);
    pidInit(&pid_lift_pos[i],0,PID_LIFT_POS_KP,PID_LIFT_POS_KI,PID_LIFT_POS_KD,0.002);
    pidSetParameter(&pid_lift_pos[i],iLimit,LIFT_I_LIMIT_POS);    
  }
	
	//储球机构
    pidInit(&pid_pan_spd,0,PID_PAN_SPD_KP,PID_PAN_SPD_KI,PID_PAN_SPD_KD,0.002);
    pidSetParameter(&pid_pan_spd,iLimit,PAN_I_LIMIT_SPD);
    pidInit(&pid_pan_pos,0,PID_PAN_POS_KP,PID_PAN_POS_KI,PID_PAN_POS_KD,0.002);
    pidSetParameter(&pid_pan_pos,iLimit,PAN_I_LIMIT_POS);    
}

void chassis_init()
{
	  chassis.last_yaw = 0.0f;
		chassis.yaw_turns = 0;
		chassis.position_x = 0;
		chassis.position_y = 0;
		chassis.vx_max_speed = CHASSIS_VX_MAX;
		chassis.vx_min_speed = CHASSIS_VX_MIN;
		chassis.vy_max_speed = CHASSIS_VY_MAX;
		chassis.vy_min_speed = CHASSIS_VY_MIN;
		chassis.wz_max_speed = CHASSIS_WZ_MAX;
		chassis.wz_min_speed = CHASSIS_WZ_MIN;
}


void lift_moto_Control(void)
{
  float temp = 0;
  int32_t d_angle;
   
	if(angle_lift > ANGLE_LIFT_MAX) angle_lift = ANGLE_LIFT_MAX;
	if(angle_lift < ANGLE_LIFT_MIN) angle_lift = ANGLE_LIFT_MIN;
	
  //位置
  d_angle = motor_msg[4].angle + motor_msg[5].angle;
  motor_msg[4].angle_desired = -angle_lift;
  motor_msg[5].angle_desired = angle_lift;
  for(uint8_t i = 0;i <= 1 ; i++)
  {
	  pidSetParameter(&pid_lift_pos[i],desired,motor_msg[i+4].angle_desired);
	  pidSetParameter(&pid_lift_pos[i],dt,ctrl_time.dt);
	  temp=pidUpdate(&pid_lift_pos[i],motor_msg[i+4].angle);
		float_constraint(&temp,5000,-5000);
	  motor_msg[i+4].speed_desired = temp - K_d_angle*d_angle;
  }
  
  //速度
  for(uint8_t i = 0; i <= 1 ; i++)
  {
	  pidSetParameter(&pid_lift_spd[i],desired,motor_msg[i+4].speed_desired);
	  pidSetParameter(&pid_lift_spd[i],dt,ctrl_time.dt);
	  temp=pidUpdate(&pid_lift_spd[i],motor_msg[i+4].speed_actual);
	  float_constraint(&temp,10000,-10000);
	  motor_msg[i+4].given_current=temp;  
  }
}




void pan_moto_Control(void)
{
  float temp = 0;
  //位置
	  pidSetParameter(&pid_pan_pos,desired,motor_msg[6].angle_desired);
	  pidSetParameter(&pid_pan_pos,dt,ctrl_time.dt);
	  temp=pidUpdate(&pid_pan_pos,motor_msg[6].angle);
		float_constraint(&temp,PAN_MAX_SPD,-PAN_MAX_SPD);
	  motor_msg[6].speed_desired = temp;
  
  //速度
	  pidSetParameter(&pid_pan_spd,desired,motor_msg[6].speed_desired);
	  pidSetParameter(&pid_pan_spd,dt,ctrl_time.dt);
	  temp=pidUpdate(&pid_pan_spd,motor_msg[6].speed_actual);
		float_constraint(&temp,PAN_MAX_SPD,-PAN_MAX_SPD);
	  motor_msg[6].given_current=temp;    
}


void chassis_moto_Control(void)
{
  float temp = 0;
	
	chassis_feedback_update(&chassis);
	chassis_set_contorl(&chassis);
	chassis_control_loop(&chassis);
  
  //速度
  for(uint8_t i = 0; i <= 4 ; i++)
  {
	  pidSetParameter(&pid_chassis_spd[i],desired,motor_msg[i].speed_desired);
	  pidSetParameter(&pid_chassis_spd[i],dt,ctrl_time.dt);
	  temp=pidUpdate(&pid_chassis_spd[i],motor_msg[i].speed_actual);
	  float_constraint(&temp,10000,-10000);
	  motor_msg[i].given_current=temp;  
  }
  
}





void chassis_reset()
{
	for(uint8_t i = 0; i < 4 ;i++)
	{
		motor_msg[i].first_run = true;
	}
}

void change_pid_slow(void)
{
	for(uint8_t i = 0; i < 4; i++)
	{
		pidParameterSet(&pid_chassis_spd[i],KP_SLOW,KI_SLOW,KD_SLOW);
	}
}
void change_pid_nomal(void)
{
	for(uint8_t i = 0; i < 4; i++)
	{
		pidParameterSet(&pid_chassis_spd[i], PID_CHASSIS_SPD_KP, PID_CHASSIS_SPD_KI, PID_CHASSIS_SPD_KD);
	}
}






/***************************底盘速度更新细节*******************************/

void chassis_feedback_update(chassis_move_t *chassis)
{
    //更新底盘前进速度x,平移速度y,旋转速度Wz，右手系
    chassis->vx = (-motor_msg[0].speed_actual + motor_msg[1].speed_actual + motor_msg[2].speed_actual - motor_msg[3].speed_actual);
    chassis->vy = (-motor_msg[0].speed_actual - motor_msg[1].speed_actual + motor_msg[2].speed_actual + motor_msg[3].speed_actual);
    chassis->wz = (-motor_msg[0].speed_actual - motor_msg[1].speed_actual - motor_msg[2].speed_actual - motor_msg[3].speed_actual);
		chassis->position_x = -motor_msg[0].angle + motor_msg[1].angle + motor_msg[2].angle - motor_msg[3].angle;
		chassis->position_y = -motor_msg[0].angle - motor_msg[1].angle + motor_msg[2].angle + motor_msg[3].angle;
}


void chassis_set_contorl(chassis_move_t *chassis)
{
	//位置(180度float角度算出int16_t的转速)
	float temp;
	pidSetParameter(&pid_chassis_yaw_pos,desired,chassis->chassis_yaw_set);
	pidSetParameter(&pid_chassis_yaw_pos,dt,ctrl_time.dt);
	temp=pidUpdate(&pid_chassis_yaw_pos,chassis->chassis_yaw);
	chassis->wz_set=temp;
	
	int16_constraint(&chassis->wz_set,chassis->wz_max_speed,chassis->wz_min_speed);
	int16_constraint(&chassis->vx_set,chassis->vx_max_speed,chassis->vx_min_speed);
	int16_constraint(&chassis->vy_set,chassis->vy_max_speed,chassis->vy_min_speed);
	
}


void chassis_control_loop(chassis_move_t *chassis)
{
    float max_speed = 0.0f; //四轮最大数
		float	rate = 0.0f;//限速比
    float temp = 0.0f;
    uint8_t i = 0;
		float wheel_speed[4]={0,0,0,0};
		
		
		if(chassis->vx_set > 0)
		{
			if(chassis->vx_set > chassis->vx_buf)
			{
				chassis->vx_buf += ramp;
			}
			if(chassis->vx_set <= chassis->vx_buf)
			{
				chassis->vx_buf = chassis->vx_set;
			}
		}
		else if(chassis->vx_set < 0)
		{
			if(chassis->vx_set < chassis->vx_buf)
			{
				chassis->vx_buf -=ramp;
			}
			if(chassis->vx_set >= chassis->vx_buf)
			{
				chassis->vx_buf = chassis->vx_set;
			}
		}
		else
		{
			if(chassis->vx_buf > 0) chassis->vx_buf -= ramp;
			if(chassis->vx_buf < 0) chassis->vx_buf += ramp;
		}
		
		
		
		if(chassis->vy_set > 0)
		{
			if(chassis->vy_set > chassis->vy_buf)
			{
				chassis->vy_buf += ramp;
			}
			if(chassis->vy_set <= chassis->vy_buf)
			{
				chassis->vy_buf = chassis->vy_set;
			}
		}
		else if(chassis->vy_set <= 0)
		{
			if(chassis->vy_set < chassis->vy_buf)
			{
				chassis->vy_buf -= ramp;
			}
			if(chassis->vy_set >= chassis->vy_buf)
			{
				chassis->vy_buf = chassis->vy_set;
			}
		}
		else
		{
			if(chassis->vy_buf > 0) chassis->vy_buf -= ramp;
			if(chassis->vy_buf < 0) chassis->vy_buf += ramp;
		}
		
		
    //底盘目标速度转轮速
    chassis_vector_to_mecanum_wheel_speed(chassis->vx_set,chassis->vy_set, chassis->wz_set, wheel_speed);
		
		//找4轮最大速 超速则4轮同比降速
    for (i = 0; i < 4; i++)
    {
        motor_msg[i].speed_desired = wheel_speed[i];
				if(motor_msg[i].speed_desired>=0) temp =  motor_msg[i].speed_desired;
				if(motor_msg[i].speed_desired<0)  temp = -motor_msg[i].speed_desired;
        if (max_speed < temp)
        {
            max_speed = temp;
        }
    }

    if (max_speed > MAX_WHEEL_SPEED)
    {
        rate = MAX_WHEEL_SPEED / max_speed;
        for (i = 0; i < 4; i++)
        {
            motor_msg[i].speed_desired *= rate;
        }
    }
		
}



void chassis_vector_to_mecanum_wheel_speed(const float vx_set, const float vy_set, const float wz_set, float wheel_speed[4])
{

    wheel_speed[0] = -vx_set - vy_set -wz_set;
    wheel_speed[1] = vx_set - vy_set -wz_set;
    wheel_speed[2] = vx_set + vy_set -wz_set;
    wheel_speed[3] = -vx_set + vy_set -wz_set;
}


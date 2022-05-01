#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "main.h"
#include "pid.h"

typedef struct
{
  int16_t vx;                
  int16_t vy;                
  int16_t wz;     
	uint8_t yaw_turns;
	float last_yaw;
	int32_t position_x;
	int32_t position_y;
  float	chassis_yaw;
	
	int16_t vx_buf;
	int16_t vy_buf;
  int16_t vx_set;            
  int16_t vy_set;            
  int16_t wz_set;            
  float chassis_yaw_set;

  int16_t vx_max_speed;  
  int16_t vx_min_speed;  
  int16_t vy_max_speed;  
  int16_t vy_min_speed;
	
  int16_t wz_max_speed;  
  int16_t wz_min_speed;  
	
} chassis_move_t;



extern PidObject pid_chassis_spd[4];
extern PidObject pid_chassis_pos[4];
extern PidObject pid_chassis_yaw_spd;
extern PidObject pid_chassis_yaw_pos;


extern PidObject pid_lift_spd[2];
extern PidObject pid_lift_pos[2];

extern PidObject pid_pan_spd;
extern PidObject pid_pan_pos;

extern chassis_move_t chassis;
extern int32_t angle_lift;

//接口函数
void motos_pid_Init(void);
void chassis_init(void);
void chassis_moto_Control(void);
void pan_moto_Control(void);
void lift_moto_Control(void);
void chassis_reset(void);
void change_pid_slow(void);
void change_pid_nomal(void);




//.c文件用函数
void chassis_feedback_update(chassis_move_t *chassis_move_update);
void chassis_vector_to_mecanum_wheel_speed(const float vx_set, const float vy_set, const float wz_set, float wheel_speed[4]);
void chassis_set_contorl(chassis_move_t *chassis_move_control);
void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

void float_constraint(float *data,float max,float min);
void int16_constraint(int16_t *data,int16_t max,int16_t min);
int constraint(int32_t data,int32_t max,int32_t min);


#endif


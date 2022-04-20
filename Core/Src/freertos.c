/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RemoteControl.h"
#include "pid.h"
#include "motoapp.h"
#include "can.h"
#include "timer.h"
#include "pan.h"
#include "servo.h"
#include "openmv.h"
#include "lineInfo.h"
#include "delay.h"
#include "sr04.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum Stage
{
	reset,
	gotoFirstPlatform,
	takeFirstPlatformBalls,
	gotoSecondPlatform,
	takeSecondPlatformBalls,
	gotoWearhouse,
	PutBalls,
	goHome
} stage;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RED  //RED OR BLUE


#ifdef BLUE

#define RUN_SPD 6500

#define GF1_FRONT 1822905
#define GF2_LEFT 3018980
#define GF3_YAW 90
#define GF4_RIGHT 5700000

#define LAD_FRONT 1076000
#define LAD_TOTAL_DISTANCE -5771216
#define LAD1_DIS -1717362
#define LAD2_DIS -4034873
#define LAD1_HIGH 170000
#define LAD2_HIGH 450000
#define LAD3_HIGH 330000
#define LAD_SPD 1500
#define LAD_BACK 1076000

#define GS1_RIGHT 5709349
#define GS2_YAW 0
#define GS3_FRONT 1000000

#define BAR_TOTAL_DISTANCE 5771216
#define BAR_SPD 1500
#define BAR_BACK 1194242

#define GW1_YAW -90
#define GW2_RIGHT 5000000

#define WEAR_SPD 1500
#define Aim_lattice_SPD 1500 //+
#define APPROACH_SPD 1500
#define WEAR_BACK_SPD 1500

#define GH1_RINGT 8820000
#define GH2_YAW 0
#define GH3_LEFT 2414980
#define GH4_BACK 1972905

#endif

#ifdef RED

#define RUN_SPD 6500
#define GF1_FRONT 1822905
#define GF2_RIGHT 2914980
#define GF3_YAW -90
#define GF4_LEFT 5700000

#define LAD_FRONT 650000
#define LAD_TOTAL_DISTANCE 5771216
#define LAD1_DIS 2444382
#define LAD2_DIS 4830186
#define LAD1_HIGH 330000
#define LAD2_HIGH 450000
#define LAD3_HIGH 170000
#define LAD_SPD 1500
#define LAD_BACK 650000

#define GS1_LEFT 5709349
#define GS2_YAW 0
#define GS3_FRONT 1244242

#define BAR_TOTAL_DISTANCE 5771216
#define BAR_SPD 1500
#define BAR_BACK 1194242

#define GW1_YAW 90
#define GW2_LEFT 5909349+6240000

#define WEAR_SPD -1500
#define Aim_lattice_SPD 1500 //+
#define APPROACH_SPD 1500
#define WEAR_BACK_SPD 1500

#define GH1_LEFT 8820000+6240000
#define GH2_YAW 0
#define GH3_RIGHT 2414980
#define GH4_BACK 1972905

#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

//所有电机软件开关
bool sw = 0;
//测试时间
uint16_t runtime = 0;
//测试速度
int16_t test_spd = 0;
//电机初启动标志
bool first_run = 1;

bool a_new_ball_in = 0;

bool sw_cal_lattice = false;

//测试数据
int a = 6290000,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q;
uint8_t x[3] = {1,2,3};
uint8_t y[3]= {1,2,3};
uint8_t test_x = 1;
uint8_t test_y = 1;
bool start_put = 0;
bool auto_clip = 0;
bool start_clip = 0;
bool auto_output = 0;
bool start_output = 0;
int32_t line_err = 0;
bool far = 1;
bool moto_clear = false;


/* USER CODE END Variables */
osThreadId AllMotoBaseTaskHandle;
osThreadId AutoChassisTaskHandle;
osThreadId AutoLiftTask_xyHandle;
osThreadId PanTask_xyzHandle;
osThreadId FuncTestTasksHandle;
osThreadId ServoTaskHandle;
osThreadId BarPlatformTaskHandle;
osThreadId putBallTaskHandle;
osThreadId sr04TaskHandle;
osThreadId gotoFirstPlatFoHandle;
osThreadId ladderTaskHandle;
osThreadId gotoSecondHandle;
osThreadId gotowearhouseHandle;
osThreadId gohomeHandle;
osThreadId cal_latticeHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void car_reset(void);
void Aim(bool*);
void find_lattice(void);
void find_lattice_v2(void);
void Aim_lattice(int32_t first_line_angle,int32_t second_line_angle,bool* found);
void approach(int32_t * approach_y);
void put_in(uint8_t x ,uint8_t y);
bool check_pan(uint8_t lattice);
bool check_ball(uint8_t x,uint8_t y );
void run_right(int32_t distance , int16_t speed);
void run_left(int32_t distance , int16_t speed);
void run_front(int32_t distance , int16_t speed);
void run_back(int32_t distance , int16_t speed);
void boit(void);
void clipit(void);
void prepare(void);
/* USER CODE END FunctionPrototypes */

void MotoBaseTask(void const * argument);
void ChassisTask(void const * argument);
void LiftTask(void const * argument);
void PanTask(void const * argument);
void FuncTestTask(void const * argument);
void Servotask(void const * argument);
void BarPlatform(void const * argument);
void PutBallTask(void const * argument);
void Sr04Task(void const * argument);
void GotoFirstPlatForm(void const * argument);
void LadderTask(void const * argument);
void GotoSecond(void const * argument);
void GotoWearhouse(void const * argument);
void Gohome(void const * argument);
void Cal_lattice(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of AllMotoBaseTask */
  osThreadDef(AllMotoBaseTask, MotoBaseTask, osPriorityNormal, 0, 128);
  AllMotoBaseTaskHandle = osThreadCreate(osThread(AllMotoBaseTask), NULL);

  /* definition and creation of AutoChassisTask */
  osThreadDef(AutoChassisTask, ChassisTask, osPriorityIdle, 0, 128);
  AutoChassisTaskHandle = osThreadCreate(osThread(AutoChassisTask), NULL);

  /* definition and creation of AutoLiftTask_xy */
  osThreadDef(AutoLiftTask_xy, LiftTask, osPriorityIdle, 0, 128);
  AutoLiftTask_xyHandle = osThreadCreate(osThread(AutoLiftTask_xy), NULL);

  /* definition and creation of PanTask_xyz */
  osThreadDef(PanTask_xyz, PanTask, osPriorityIdle, 0, 128);
  PanTask_xyzHandle = osThreadCreate(osThread(PanTask_xyz), NULL);

  /* definition and creation of FuncTestTasks */
  osThreadDef(FuncTestTasks, FuncTestTask, osPriorityRealtime, 0, 128);
  FuncTestTasksHandle = osThreadCreate(osThread(FuncTestTasks), NULL);

  /* definition and creation of ServoTask */
  osThreadDef(ServoTask, Servotask, osPriorityNormal, 0, 128);
  ServoTaskHandle = osThreadCreate(osThread(ServoTask), NULL);

  /* definition and creation of BarPlatformTask */
  osThreadDef(BarPlatformTask, BarPlatform, osPriorityNormal, 0, 128);
  BarPlatformTaskHandle = osThreadCreate(osThread(BarPlatformTask), NULL);

  /* definition and creation of putBallTask */
  osThreadDef(putBallTask, PutBallTask, osPriorityIdle, 0, 128);
  putBallTaskHandle = osThreadCreate(osThread(putBallTask), NULL);

  /* definition and creation of sr04Task */
  osThreadDef(sr04Task, Sr04Task, osPriorityIdle, 0, 128);
  sr04TaskHandle = osThreadCreate(osThread(sr04Task), NULL);

  /* definition and creation of gotoFirstPlatFo */
  osThreadDef(gotoFirstPlatFo, GotoFirstPlatForm, osPriorityIdle, 0, 128);
  gotoFirstPlatFoHandle = osThreadCreate(osThread(gotoFirstPlatFo), NULL);

  /* definition and creation of ladderTask */
  osThreadDef(ladderTask, LadderTask, osPriorityIdle, 0, 128);
  ladderTaskHandle = osThreadCreate(osThread(ladderTask), NULL);

  /* definition and creation of gotoSecond */
  osThreadDef(gotoSecond, GotoSecond, osPriorityIdle, 0, 128);
  gotoSecondHandle = osThreadCreate(osThread(gotoSecond), NULL);

  /* definition and creation of gotowearhouse */
  osThreadDef(gotowearhouse, GotoWearhouse, osPriorityIdle, 0, 128);
  gotowearhouseHandle = osThreadCreate(osThread(gotowearhouse), NULL);

  /* definition and creation of gohome */
  osThreadDef(gohome, Gohome, osPriorityIdle, 0, 128);
  gohomeHandle = osThreadCreate(osThread(gohome), NULL);

  /* definition and creation of cal_lattice */
  osThreadDef(cal_lattice, Cal_lattice, osPriorityIdle, 0, 128);
  cal_latticeHandle = osThreadCreate(osThread(cal_lattice), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_MotoBaseTask */
/**
  * @brief  Function implementing the AllMotoBaseTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_MotoBaseTask */
void MotoBaseTask(void const * argument)
{
  /* USER CODE BEGIN MotoBaseTask */
  /* Infinite loop */
  for(;;)
  {	
		if(sw == 0)
		{
			for(uint8_t i = 0; i < 4 ; i++)
			{
				pidClearIntegral(&pid_chassis_spd[i]);
			}
		}
		get_dt_in_seconds(&ctrl_time);
		if(first_run)
		{
			first_run = 0;
			ctrl_time.dt = 0.002;
		}
		chassis_moto_Control();
		pan_moto_Control();
		lift_moto_Control();
		if(sw == 1)
		{
		CAN1_send_current_flag = true;
		}
    osDelay(1);
  }
  /* USER CODE END MotoBaseTask */
}

/* USER CODE BEGIN Header_ChassisTask */
/**
  * @brief  Function implementing the ChassisTask_xyz thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ChassisTask */
void ChassisTask(void const * argument)
{
  /* USER CODE BEGIN ChassisTask */
  /* Infinite loop */
  for(;;)
  {
		if(sw == 0)//启动缓冲
		{
			osDelay(5000);
			sw = 1;
		}
		if(RC_CtrlData.rc.sw1==3)
		{
		chassis.vx_set=-RC_CtrlData.rc.ch3*10;
		chassis.vy_set=-RC_CtrlData.rc.ch4*10;
		chassis.chassis_yaw_set += RC_CtrlData.rc.ch1*0.0002;
		}
		else
		{
			//chassis.vx_set = 0;
			//chassis.vy_set = 0;
		}
    osDelay(1);
  }
  /* USER CODE END ChassisTask */
}

/* USER CODE BEGIN Header_LiftTask */
/**
* @brief Function implementing the AutoLiftTask_xy thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LiftTask */
void LiftTask(void const * argument)
{
  /* USER CODE BEGIN LiftTask */
  /* Infinite loop */
  for(;;)
  {
		if(RC_CtrlData.rc.sw1==3)
		{
		angle_lift += RC_CtrlData.rc.ch2;
		}
    osDelay(1);
  }
  /* USER CODE END LiftTask */
}

/* USER CODE BEGIN Header_PanTask */
/**
* @brief Function implementing the PanTask_xyz thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PanTask */
void PanTask(void const * argument)
{
  /* USER CODE BEGIN PanTask */
  /* Infinite loop */
  for(;;)
  {
		if(a_new_ball_in)
		{
			turn_box(1,1);
			a_new_ball_in = 0;
		}
    osDelay(1);
  }
  /* USER CODE END PanTask */
}

/* USER CODE BEGIN Header_FuncTestTask */
/**
* @brief Function implementing the FuncTestTasks thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FuncTestTask */
void FuncTestTask(void const * argument)
{
  /* USER CODE BEGIN FuncTestTask */
  /* Infinite loop */
  for(;;)
  {
		//moto_clear
		if(moto_clear)
		{
			chassis_reset();
			moto_clear = false;
		}
		
		
		//start run
		if(RC_CtrlData.rc.sw2 == 2)
		{
			stage = gotoFirstPlatform;
		}
		//自动夹取加识别测试
		if(auto_clip == 1)
		{
			servos.top = 200;
			servos.arm = 112;
			osDelay(1500);
			angle_lift = 280000;
			servos.clip = 80;
			servos.bo =115;
			servos.move = 105;
			servos.yindao = 65;
			auto_clip = 0;
		}
		if(start_clip)
		{
			angle_lift = 420000;
			osDelay(500);
			servos.clip = 140;
			osDelay(500);
			angle_lift = 280000;
			osDelay(500);
			servos.clip = 155;
			osDelay(500);
			servos.top = 50;
			osDelay(800);
			servos.clip = 80;
			osDelay(1000);
			servos.move = 280;
			osDelay(1000);
			turn_box(0,1);
			servos.top = 200;
			osDelay(800);
			servos.move = 105;
		
			start_clip = false;
			open_openmv();
		}
		//自动顺序吐球测试
		if(auto_output)
		{
			servos.move = 360;
			servos.top = 300;
			servos.bo = 180;
			servos.arm = 60;
			osDelay(1000);
			servos.yindao = 46;
			osDelay(800);
			servos.clip = 140;
			servos.top = 75;
			servos.arm = 37;
			auto_output = 0;
		}
		if(start_output)
		{
			for(test_x = 1 ;test_x <=3 ; test_x++)
			{
				for(test_y = 1 ;test_y <=3 ; test_y++)
				{
					put_a_ball(test_x,test_y);
					osDelay(2000);
					servos.move = 40;
					osDelay(1200);
					servos.clip = 155;
					osDelay(300);
					servos.move = 360;
					servos.arm = 120;
					osDelay(500);
					servos.top = 300;
					osDelay(1000);
					servos.clip = 140;
					osDelay(300);
					servos.top = 75;
					osDelay(1000);
					servos.arm = 37;
				}
			}
			start_output = 0;
		}
		
//底盘测试
//		if(sw == 0)
//		{
//			chassis.vx_set = 0;
//			for(uint8_t i=0 ; i<4 ; i++)
//			{
//				pidClearIntegral(&pid_chassis_spd[i]);
//				pidClearIntegral(&pid_chassis_yaw_pos);
//			}
//		}
//		if(sw==1)
//		{
//			chassis.vx_set = test_spd;
//			osDelay(runtime);
//			sw=0;
//		}

    osDelay(1);
  }
  /* USER CODE END FuncTestTask */
}

/* USER CODE BEGIN Header_Servotask */
/**
* @brief Function implementing the ServoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Servotask */
void Servotask(void const * argument)
{
  /* USER CODE BEGIN Servotask */
  /* Infinite loop */
  for(;;)
  {
		if(RC_CtrlData.rc.sw1==1)
		{
			servos.top += RC_CtrlData.rc.ch4*0.001;
			servos.arm += RC_CtrlData.rc.ch2*0.0005;
		}
		if(RC_CtrlData.rc.sw1==2)
		{
			servos.move-=RC_CtrlData.rc.ch4*0.001;
			servos.yindao-=RC_CtrlData.rc.ch2*0.0002;
		}
		if(RC_CtrlData.rc.sw2== 1)
		{
			servos.bo -= RC_CtrlData.rc.ch5*0.001;
		}
		else
		{
		servos.clip -= RC_CtrlData.rc.ch5 * 0.0003;
		}
		servos_control_loop();
    osDelay(1);
  }
  /* USER CODE END Servotask */
}

/* USER CODE BEGIN Header_BarPlatform */
/**
* @brief Function implementing the BarPlatformTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BarPlatform */
void BarPlatform(void const * argument)
{
  /* USER CODE BEGIN BarPlatform */
	bool Aimed = false;
  /* Infinite loop */
  for(;;)
  {
		if(stage == takeSecondPlatformBalls )//进入条形平台阶段
		{
			close_openmv();
			prepare();
			chassis_reset();
			open_openmv();
			
#ifdef BLUE
			while(chassis.position_x > - BAR_TOTAL_DISTANCE)
			{
					//球没出现则平移
					if(ball_x == 0 || ball_y == 0)
					{						
						chassis.vx_set = -BAR_SPD;
					}
					//球出现则开始瞄准 瞄准完毕则自动夹取
					else
					{
						chassis.vx_set = 0;
						chassis.vy_set = 0;
						if(!Aimed) Aim(&Aimed);
						else 
						{
							Aimed = 0;
							chassis.vx_set = 0;
							chassis.vy_set = 0;
							boit();
						}
					}
					osDelay(1);
			}//endwhile
#endif			
			
#ifdef RED
			while(chassis.position_x <  BAR_TOTAL_DISTANCE)
			{
					//球没出现则平移
					if(ball_x == 0 || ball_y == 0)
					{						
						chassis.vx_set = BAR_SPD;
					}
					//球出现则开始瞄准 瞄准完毕则自动夹取
					else
					{
						chassis.vx_set = 0;
						chassis.vy_set = 0;
						if(!Aimed) Aim(&Aimed);
						else 
						{
							Aimed = 0;
							chassis.vx_set = 0;
							chassis.vy_set = 0;
							boit();
						}
					}
					osDelay(1);
			}//endwhile
#endif
			
			chassis.vx_set = 0;
			run_back(BAR_BACK,RUN_SPD);
			car_reset();
			stage = gotoWearhouse;		
		}
		osDelay(1);	
	}

  /* USER CODE END BarPlatform */
}

/* USER CODE BEGIN Header_PutBallTask */
/**
* @brief Function implementing the putBallTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PutBallTask */
void PutBallTask(void const * argument)
{
  /* USER CODE BEGIN PutBallTask */
	uint8_t lattice = 0;
  /* Infinite loop */
  for(;;)	
  {
		if(stage == PutBalls)
		{
			bool ball_exist_y = 0;
			prepare();
			while(lattice<3)//依次放3列
			{
				//走到下一格
				find_lattice_v2();
				lattice++;
				//查看当前列有无球 有就放没有就跳过
				ball_exist_y = check_pan(lattice);
				if(ball_exist_y)
				{
					bool ball_exist_x = 0;
					int32_t approach_y = 0;
					osDelay(300);
					approach(&approach_y);
					chassis.vy_set = 0;
					for(uint8_t x = 1 ; x <=3 ; x++)
					{
						ball_exist_x = check_ball(x,lattice);
						if(ball_exist_x)
						{
							put_in(x,lattice);
						}						
					}
					run_back(approach_y,WEAR_BACK_SPD);

				}
			}//end3loop
			stage = goHome;
		}
    osDelay(1);
  }
  /* USER CODE END PutBallTask */
}

/* USER CODE BEGIN Header_Sr04Task */
/**
* @brief Function implementing the sr04Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Sr04Task */
void Sr04Task(void const * argument)
{
  /* USER CODE BEGIN Sr04Task */
  /* Infinite loop */
  for(;;)
  {
			far = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6);
//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
//		delay_us(13);
//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
//    osDelay(300);
//		distance = get_distance(1);
		osDelay(1);
  }
  /* USER CODE END Sr04Task */
}

/* USER CODE BEGIN Header_GotoFirstPlatForm */
/**
* @brief Function implementing the gotoFirstPlatFo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GotoFirstPlatForm */
void GotoFirstPlatForm(void const * argument)
{
  /* USER CODE BEGIN GotoFirstPlatForm */
  /* Infinite loop */
  for(;;)
  {
		if(stage == gotoFirstPlatform)
		{
#ifdef BLUE
			run_front(GF1_FRONT,RUN_SPD);
			run_left(GF2_LEFT,RUN_SPD);
			osDelay(500);
			chassis.chassis_yaw_set = GF3_YAW;
			osDelay(1000);
			run_right(GF4_RIGHT,RUN_SPD);
#endif

#ifdef RED
			run_front(GF1_FRONT,RUN_SPD);
			run_right(GF2_RIGHT,RUN_SPD);
			osDelay(500);
			chassis.chassis_yaw_set = GF3_YAW;
			osDelay(1000);
			run_left(GF4_LEFT,RUN_SPD);
#endif
			stage = takeFirstPlatformBalls;
		}
		
    osDelay(1);
  }
  /* USER CODE END GotoFirstPlatForm */
}

/* USER CODE BEGIN Header_LadderTask */
/**
* @brief Function implementing the ladderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LadderTask */
void LadderTask(void const * argument)
{
  /* USER CODE BEGIN LadderTask */
	bool aimed2 = false;
	uint8_t ladder = 1;
  /* Infinite loop */
  for(;;)
  {
		if(stage == takeFirstPlatformBalls)
		{
			close_openmv();
			prepare();
			run_front(LAD_FRONT,RUN_SPD);
			chassis_reset();
			open_openmv();
			
			
#ifdef BLUE			
			while(chassis.position_x > LAD_TOTAL_DISTANCE)
			{
				
				//阶梯高度调整
				if(ladder == 1 && chassis.position_x < LAD1_DIS) 
				{
					chassis.vx_set = 0;
					angle_lift = LAD2_HIGH;
					osDelay(1000);
					ladder = 2;
				}
				else if(ladder == 2 && chassis.position_x < LAD2_DIS)
				{
					chassis.vx_set = 0;
					angle_lift = LAD3_HIGH;
					osDelay(2000);
					ladder = 3;
				}
				
				
				if(ball_x == 0 || ball_y == 0)//球没出现则平移
				{
					chassis.vx_set = -LAD_SPD;
				}
				else
				{
						if(!aimed2) Aim(&aimed2);
						else 
						{	
							aimed2 = 0;
							chassis.vx_set = 0;
							chassis.vy_set = 0;
							close_openmv();
							clipit();
							open_openmv();
						}
				}
				osDelay(1);
			}//endwhile
#endif
			
#ifdef RED			
			while(chassis.position_x < LAD_TOTAL_DISTANCE)
			{
				
				//阶梯高度调整
				if(ladder == 1 && chassis.position_x > LAD1_DIS) 
				{
					chassis.vx_set = 0;
					angle_lift = LAD2_HIGH;
					osDelay(1000);
					ladder = 2;
				}
				else if(ladder == 2 && chassis.position_x > LAD2_DIS)
				{
					chassis.vx_set = 0;
					angle_lift = LAD3_HIGH;
					osDelay(2000);
					ladder = 3;
				}
				
				
				if(ball_x == 0 || ball_y == 0)//球没出现则平移
				{
					chassis.vx_set = LAD_SPD;
				}
				else
				{
						if(!aimed2) Aim(&aimed2);
						else 
						{	
							aimed2 = 0;
							chassis.vx_set = 0;
							chassis.vy_set = 0;
							close_openmv();
							clipit();
							open_openmv();
						}
				}
				osDelay(1);
			}//endwhile
#endif
			chassis.vx_set = 0;
			ladder = 0;			
			run_back(LAD_BACK,RUN_SPD);
			car_reset();
			stage = gotoSecondPlatform;
		}//endstage
    osDelay(1);
  }
  /* USER CODE END LadderTask */
}

/* USER CODE BEGIN Header_GotoSecond */
/**
* @brief Function implementing the gotoSecond thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GotoSecond */
void GotoSecond(void const * argument)
{
  /* USER CODE BEGIN GotoSecond */
  /* Infinite loop */
  for(;;)
  {
		if(stage == gotoSecondPlatform)
		{
#ifdef BLUE
			run_right(GS1_RIGHT,RUN_SPD);
			osDelay(300);
			chassis.chassis_yaw_set = GS2_YAW;
			osDelay(800);
			run_front(GS3_FRONT,RUN_SPD);
			osDelay(300);
#endif
			
#ifdef RED
			run_left(GS1_LEFT,RUN_SPD);
			osDelay(300);
			chassis.chassis_yaw_set = GS2_YAW;
			osDelay(800);
			run_front(GS3_FRONT,RUN_SPD);
			osDelay(300);
#endif
			stage = takeSecondPlatformBalls;
		}
    osDelay(1);
  }
  /* USER CODE END GotoSecond */
}

/* USER CODE BEGIN Header_GotoWearhouse */
/**
* @brief Function implementing the gotowearhouse thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GotoWearhouse */
void GotoWearhouse(void const * argument)
{
  /* USER CODE BEGIN GotoWearhouse */
  /* Infinite loop */
  for(;;)
  {
		if(stage == gotoWearhouse)
		{
#ifdef BLUE
			osDelay(300);
			chassis.chassis_yaw_set = GW1_YAW;
			osDelay(800);
			run_right(GW2_RIGHT,RUN_SPD);
			osDelay(300);
#endif
			
#ifdef RED
			osDelay(300);
			chassis.chassis_yaw_set = GW1_YAW;
			osDelay(800);
			run_left(GW2_LEFT,RUN_SPD);
			osDelay(300);
#endif
			stage = PutBalls;
		}
    osDelay(1);
  }
  /* USER CODE END GotoWearhouse */
}

/* USER CODE BEGIN Header_Gohome */
/**
* @brief Function implementing the gohome thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gohome */
void Gohome(void const * argument)
{
  /* USER CODE BEGIN Gohome */
  /* Infinite loop */
  for(;;)
  {
		if(stage == goHome)
		{
#ifdef BLUE
			run_right(GH1_RINGT,RUN_SPD);
			chassis.chassis_yaw_set = GH2_YAW;
			run_left(GH3_LEFT,RUN_SPD);
			run_back(GH4_BACK,RUN_SPD);
#endif
			
#ifdef RED
			run_left(GH1_LEFT,RUN_SPD);
			chassis.chassis_yaw_set = GH2_YAW;
			run_right(GH3_RIGHT,RUN_SPD);
			run_back(GH4_BACK,RUN_SPD);
#endif
			stage = reset;
		}
    osDelay(1);
  }
  /* USER CODE END Gohome */
}

/* USER CODE BEGIN Header_Cal_lattice */
/**
* @brief Function implementing the cal_lattice thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Cal_lattice */
void Cal_lattice(void const * argument)
{
  /* USER CODE BEGIN Cal_lattice */
  /* Infinite loop */
  for(;;) //1 L   2R 
  {
		static uint8_t lattice = 0;
		static uint8_t last = 0;
		
		if(sw_cal_lattice)
		{
		 		if(beyoundred[2] == 0)
				{
					if(last == 0 || last == 2) last = 2;
					else if(last == 1)
					{
						 last = 0;
						 lattice--;
					}
				}
				
				if(beyoundred[1] == 0)
				{
					if(last == 0 || last == 1) last = 1;
					else if(last == 2)
					{
						 last = 0;
						 lattice++;
					}
				}

		}
    osDelay(1);
  }
  /* USER CODE END Cal_lattice */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */





void car_reset()
{
	if(stage ==  takeFirstPlatformBalls)
	{
		servos.yindao = 160;
		servos.move = 280;
		angle_lift = 0;
		osDelay(800);
		servos.arm = 30;
		servos.clip = 80;
		servos.bo = 46;
		servos.top = 50;
	}
	else if(stage == takeSecondPlatformBalls)
	{
		servos.yindao = 160;
		servos.move = 280;
		angle_lift = 0;
		osDelay(800);
		servos.arm = 30;
		servos.clip = 80;
		servos.bo = 46;
		servos.top = 50;
	}
	else if(stage == PutBalls)
	{
		
	}
}

void Aim(bool* Aimed)
{
	int16_t error_x = ball_x - 70;
	int16_t error_y = ball_y - 76;
//	if(error_x < 35 && error_x > -35 ) error_x *= 1.5;
//	if(error_y < 35 && error_y > -35 ) error_y *= 1.5;	
	chassis.vx_set = error_x * 12.0f; 
	chassis.vy_set = -error_y * 12.0f;
	if(error_x > - 5 && error_x < 5 && error_y > -5 && error_y <5)
	{
		*Aimed = true;
	}
	
}


void find_lattice()
{
	int32_t first_line_angle = 0;
	int32_t second_line_angle = 0;
	bool touch_second_line = 0;
	bool touch_first_line = 0;
	bool beyound_first_line = 0;
	bool found = false;
	while(!found)
	{
		if(!touch_second_line)
		{
				chassis.vx_set = -WEAR_SPD;						

				if(beyoundred[7] == 0)
				{
					if(!beyound_first_line)
					{
						touch_first_line = 1;
						first_line_angle = motor_msg[0].angle;
						osDelay(1500);
					}
					else
					{
						touch_second_line = 1;
						second_line_angle = motor_msg[0].angle;
					}						
				}
				
				if(beyoundred[7] == 1 && touch_first_line == 1)
				{
					touch_first_line = 0;
					beyound_first_line = 1;
				}
		}
		else
		{
			Aim_lattice(first_line_angle,second_line_angle,&found);

		}
	}

}

void find_lattice_v2()
{
	while(beyoundred[9])
	{
		chassis.vx_set = -WEAR_SPD;
		osDelay(1);
	}
	run_right(780000,WEAR_SPD);
}


void Aim_lattice(int32_t first_line_angle,int32_t second_line_angle,bool* found)
{
	int32_t mid_angle = (first_line_angle + second_line_angle) / 2;
	 line_err =  motor_msg[0].angle - mid_angle;
	if(line_err > 0) chassis.vx_set = Aim_lattice_SPD;
	if(line_err < 0) chassis.vx_set = -Aim_lattice_SPD;
	
//	chassis.vx_set = line_err * 0.01f;			
	if(line_err > -5000 && line_err < 5000 )
			{
				chassis.vx_set = 0;
				*found = true;
			}
}


void approach(int32_t * approach_y)
{
	chassis_reset();
	while(far)
	{
		chassis.vy_set = -APPROACH_SPD;
		osDelay(1);
	}
	*approach_y = -chassis.position_y;
}



void put_in(uint8_t x , uint8_t y)
{
	if(x == 1) angle_lift = 500000;
	if(x == 2) angle_lift = 366448;
	if(x == 3) angle_lift = 0;
	put_a_ball(x,y);
	osDelay(2000);
	servos.move = 40;
	osDelay(1200);
	servos.clip = 155;
	osDelay(300);
	servos.move = 280;
	osDelay(1200);
	if(x==1) servos.arm = 135;
	else servos.arm = 100;
	osDelay(700);
	servos.top = 300;
	osDelay(1000);
	servos.clip = 140;
	osDelay(300);
	servos.top = 74;
	osDelay(1000);
	servos.arm = 40;	
}

bool check_pan(uint8_t lattice)
{
	for(uint8_t i=0;i<9;i++)
	{
			if(bodanpan.date[i] == 16 + lattice || bodanpan.date[i] == 32 + lattice || bodanpan.date[i] == 48 + lattice) return true;
	}
	return false;
}

bool check_ball(uint8_t x,uint8_t y )
{
	for(uint8_t i=0;i<9;i++)
	{
			if(bodanpan.date[i] == 16*x + y) return true;
	}
	return false;
}



void run_left(int32_t distance , int16_t speed)
{
	chassis_reset();
	osDelay(3);
	while(chassis.position_x < distance)
	{
		chassis.vx_set = speed;
		osDelay(1);
	}
	chassis.vx_set = 0;
	
}

void run_right(int32_t distance , int16_t speed)
{
	chassis_reset();
	osDelay(3);
	while(chassis.position_x > -distance)
	{
		chassis.vx_set = -speed;
		osDelay(1);
	}
	chassis.vx_set = 0;
}

void run_front(int32_t distance , int16_t speed)
{
	chassis_reset();
	osDelay(3);
	while(chassis.position_y > -distance)
	{
		chassis.vy_set = -speed;
		osDelay(1);
	}
	chassis.vy_set = 0;
}

void run_back(int32_t distance , int16_t speed)
{
	chassis_reset();
	osDelay(3);
	while(chassis.position_y < distance)
	{
		chassis.vy_set = speed;
		osDelay(1);
	}
	chassis.vy_set = 0;
}

void prepare()
{
	if(stage == takeSecondPlatformBalls)//条形平台姿态预备
	{
		servos.clip = 80;
		servos.move = 70;
		servos.arm = 116;
		servos.bo = 45;
		servos.top = 177;
		osDelay(500);
		angle_lift = 400000;
		servos.yindao =  80;
		osDelay(500);
	}
	if(stage == takeFirstPlatformBalls )//阶梯平台姿态预备
	{
		servos.top = 300;
		servos.arm = 120;
		servos.bo = 45;
		servos.clip = 140;
		osDelay(500);
		angle_lift = LAD1_HIGH;
		servos.move = 0;
		servos.yindao =  85;
		
	}
	else if(stage == PutBalls)//仓库姿态预备
	{
		servos.bo = 142;
		servos.arm = 140;
		osDelay(800);
		servos.yindao = 46;
		osDelay(800);
		servos.top = 74;
		servos.arm = 40;
		osDelay(700);
		servos.clip = 135;	
	}
	
	
}
void boit()
{
	servos.bo = 0;
	servos.top = 120;
	osDelay(500);
	servos.bo = 45;
	servos.top = 177;	
	servos.move = 280;
	osDelay(800);
	turn_box(0,1);
	osDelay(400);
	servos.move = 100;
	osDelay(300);
}

void clipit()
{
	servos.arm = 103;
	osDelay(300);
	servos.clip = 155;
	osDelay(300);
	servos.arm = 120;
	osDelay(300);
	servos.top = 50;
	osDelay(800);
	servos.arm = 107;
	osDelay(500);
	servos.clip = 140;
	osDelay(700);
	servos.move = 280;
	servos.arm = 120;
	osDelay(700);
	turn_box(0,1);
	servos.top = 300;
	osDelay(500);
	servos.move = 0;
	osDelay(300);
}

/* USER CODE END Application */

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
#include "QRCode.h"
#include "timer.h"
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
	takeLiZhuangBalls,
	gotoDaoduo,
	daoduo,
	gotoputball,
	PutBalls,
	goHome
} stage;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BLUE  //RED OR BLUE

#define WAIT_TIME 40
#define YAW_CORRECT_PANJI 		1.5f
#define YAW_CORRECT_LIZHUANG	4.5f
#define YAW_CORRECT_PUTBALL		6.2f
#define YAW_CORRECT_GOHOME 		9.0f


#ifdef BLUE

#define RUN_SPD 6500

#define GF1_FRONT 1822905
#define GF2_LEFT 3018980
#define GF3_YAW 90
#define GF4_RIGHT 4600000

#define LAD_FRONT 192000 
#define LAD_TOTAL_DISTANCE -5771216
#define LAD1_DIS -1950000
#define LAD2_DIS -4398873
#define LAD1_HIGH 170000
#define LAD2_HIGH 450000
#define LAD3_HIGH 330000
#define LAD_SPD 1500
#define LAD_BACK 1076000

#define WEAR_SPD 1500
#define Aim_lattice_SPD 1500 //+
#define APPROACH_SPD 1500
#define WEAR_BACK_SPD 1500

#define GH1_RINGT 6000000
#define GH2_YAW 0
#define GH3_LEFT 2300000
#define GH4_BACK 2372905

#endif

#ifdef RED

#define RUN_SPD 6500
#define GF1_FRONT 1822905
#define GF2_RIGHT 3018980
#define GF3_YAW -90
#define GF4_LEFT 4600000

#define LAD_FRONT 192000
#define LAD_TOTAL_DISTANCE 5771216
#define LAD1_DIS 1600000
#define LAD2_DIS 4100000
#define LAD1_HIGH 330000
#define LAD2_HIGH 450000
#define LAD3_HIGH 170000
#define LAD_SPD 1500
#define LAD_BACK 1076000

#define WEAR_SPD 1500
#define Aim_lattice_SPD 1500 //+
#define APPROACH_SPD 1500
#define WEAR_BACK_SPD 1500

#define GH1_LEFT 52000*130
#define GH2_YAW -360
#define GH3_RIGHT 2300000
#define GH4_BACK  2112905

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
uint8_t lattice_daoduo= 0;
uint8_t layer = 2;
uint8_t last = 0;

uint8_t rwm_sq[3] = {0};

uint8_t sw1 = 0,sw2 = 0;
int found_ball_time = 0;


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
osThreadId gotoputballHandle;
osThreadId DaoduoTaskHandle;
osThreadId lizhaungtaskHandle;
osThreadId gotoDaoduoHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void car_reset(void);
void Aim(bool*);
void find_lattice(void);
void next_lattice_v2(bool right, uint8_t number);
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
bool ball_exist(void);
void clipit_daoduo(void);
void putBall_daoduo(void);
void reset_daoduo(void);
void dingwei(void);
void wait_ball(void);
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
void GotoPutball(void const * argument);
void daoduoTask(void const * argument);
void lizhuangTask(void const * argument);
void gotodaoduo(void const * argument);

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

  /* definition and creation of gotoputball */
  osThreadDef(gotoputball, GotoPutball, osPriorityIdle, 0, 128);
  gotoputballHandle = osThreadCreate(osThread(gotoputball), NULL);

  /* definition and creation of DaoduoTask */
  osThreadDef(DaoduoTask, daoduoTask, osPriorityIdle, 0, 128);
  DaoduoTaskHandle = osThreadCreate(osThread(DaoduoTask), NULL);

  /* definition and creation of lizhaungtask */
  osThreadDef(lizhaungtask, lizhuangTask, osPriorityIdle, 0, 128);
  lizhaungtaskHandle = osThreadCreate(osThread(lizhaungtask), NULL);

  /* definition and creation of gotoDaoduo */
  osThreadDef(gotoDaoduo, gotodaoduo, osPriorityIdle, 0, 128);
  gotoDaoduoHandle = osThreadCreate(osThread(gotoDaoduo), NULL);

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
			osDelay(2000);
			sw = 1;
		}
		if(0)//(RC_CtrlData.rc.sw1==3)
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
	bool first_wait = true;
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
		if(RC_CtrlData.rc.sw2 == 2 && first_wait == true)
		{
			stage = gotoFirstPlatform;
			first_wait = false;
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
//		if(RC_CtrlData.rc.sw1==1)
//		{
//			servos.top += RC_CtrlData.rc.ch4*0.001;
//			servos.arm += RC_CtrlData.rc.ch2*0.0005;
//		}
//		if(RC_CtrlData.rc.sw1==2)
//		{
//			servos.move-=RC_CtrlData.rc.ch4*0.001;
//			servos.yindao-=RC_CtrlData.rc.ch2*0.0002;
//		}
//		if(RC_CtrlData.rc.sw2== 1)
//		{
//			servos.bo -= RC_CtrlData.rc.ch5*0.001;
//		}
//		else
//		{
//		servos.clip -= RC_CtrlData.rc.ch5 * 0.0003;
//		}
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
	uint8_t ball_num = 0;
  /* Infinite loop */
  for(;;)
  {
		if(stage == takeSecondPlatformBalls )//进入圆盘机阶段
		{
			close_openmv();
			prepare();
			change_pid_slow();
			osDelay(2000);
			chassis_reset();
			run_front(1355000,1000);
			servos.bo = 50;
			osDelay(500);
			open_openmv();
			get_dt_in_seconds(&run_time);

			
			while(ball_num < 5)
			{
				wait_ball();
				if(run_time.time_total > WAIT_TIME)
				goto BALL_FINISH;
				
				boit();
				ball_num++;
			}
				
BALL_FINISH:
			change_pid_nomal();
			run_back(1300000,4000);
			car_reset();
			change_yaw_pid_turn();
			yaw_correction = YAW_CORRECT_PANJI;
			#ifdef BLUE
			chassis.chassis_yaw_set = 90;
			#endif
			
			#ifdef RED
			chassis.chassis_yaw_set = -90;
			#endif
			osDelay(2500);
			change_yaw_pid_run();
			run_back(500000,4000);

//			chassis.vx_set = 0;
//			run_back(BAR_BACK,RUN_SPD);
//			car_reset();
			stage = gotoWearhouse;		
//stage = reset; 
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
				next_lattice_v2(true,1);
				lattice++;
				run_back(500000,1500);
				osDelay(800);
				run_front(500000,1500);
				rwm_sq[lattice-1] = QRCode_data[0] - 0x30;
				
				//查看当前列有无球 有就放没有就跳过
				ball_exist_y = check_pan(lattice);
				if(ball_exist_y)
				{
					bool ball_exist_x = 0;
					int32_t approach_y = 0;
					osDelay(300);
					approach(&approach_y);
					chassis.vy_set = 0;
					for(uint8_t x = 3; x >=1; x--)
					{
						if(x == 1)
						{
							
							ball_exist_x = check_ball(x,rwm_sq[lattice-1]);
							if(ball_exist_x)
							{
								put_in(x,rwm_sq[lattice - 1]);
							}
						}
						else
						{
							ball_exist_x = check_ball(x,lattice);
							if(ball_exist_x)
							{
								put_in(x,lattice);
							}
						}
						
					}//endfor
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
			change_yaw_pid_turn();
			chassis.chassis_yaw_set = GF3_YAW;
			osDelay(1500);
			change_yaw_pid_run();
			run_right(GF4_RIGHT,RUN_SPD);
			dingwei();
#endif

#ifdef RED
			run_front(GF1_FRONT,RUN_SPD);
			run_right(GF2_RIGHT,RUN_SPD);
			osDelay(500);
			change_yaw_pid_turn();
			chassis.chassis_yaw_set = GF3_YAW;
			osDelay(1500);
			change_yaw_pid_run();
			run_left(GF4_LEFT,RUN_SPD);
			dingwei();
#endif
			stage = takeFirstPlatformBalls;
			//stage = reset;
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
			change_pid_slow();
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
					osDelay(1500);
					ladder = 3;
				}
				
				if(chassis.position_x < LAD2_DIS+250000 && chassis.position_x > LAD2_DIS)//防止阶梯未降下就空夹（范围判断）
				{
					ball_x = 0;
					ball_y = 0;
				}
				
				if(ball_x == 0 || ball_y == 0)//球没出现则平移
				{
					chassis.vx_set = -LAD_SPD;
					chassis.vy_set = 0;
				}
				else
				{
		
									if(!aimed2) 
									{
										
										if((chassis.position_x < LAD1_DIS + 150000 && chassis.position_x > LAD1_DIS - 150000) || (chassis.position_x < LAD2_DIS + 150000 && chassis.position_x > LAD2_DIS - 150000))// 角落球,强行瞄准模式（不允许升降打断）
										{
											while(!aimed2)
											{
												Aim(&aimed2);
												osDelay(1);
											}
											goto CLIP;
											
										}
										else//普通瞄准（只关心当前）
										{
											Aim(&aimed2);
										}
									}
									else 
									{	
CLIP:									aimed2 = 0;
											chassis.vx_set = 0;
											chassis.vy_set = 0;
											close_openmv();
											clipit();
											if(chassis.position_x < LAD1_DIS + 150000 && chassis.position_x > LAD1_DIS - 150000) //平台1的角落球,特殊处理
											{
												while(chassis.position_x < LAD1_DIS + 250000)
												{
													chassis.vx_set = LAD_SPD;//回走
													osDelay(1);
												}
												chassis.vx_set = 0;
												angle_lift = LAD2_HIGH;
												osDelay(500);
											}
											open_openmv();
									}
						
				}
				osDelay(1);
			}//endwhile
			
			chassis.vx_set = 0;
			ladder = 0;
			car_reset();			
			run_back(LAD_BACK,1500);
			run_left(520000,1500);
			dingwei();
			change_pid_nomal();
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
					osDelay(1500);
					ladder = 3;
				}
				
				if(chassis.position_x > LAD2_DIS-250000 && chassis.position_x < LAD2_DIS)//防止阶梯未降下就空夹（范围判断）
				{
					ball_x = 0;
					ball_y = 0;
				}
				
				if(ball_x == 0 || ball_y == 0)//球没出现则平移
				{
					chassis.vx_set = LAD_SPD;
					chassis.vy_set = 0;
				}
				else
				{
		
									if(!aimed2) 
									{
										
										if((chassis.position_x < LAD1_DIS + 150000 && chassis.position_x > LAD1_DIS - 150000) || (chassis.position_x < LAD2_DIS + 150000 && chassis.position_x > LAD2_DIS - 150000))// 角落球,强行瞄准模式（不允许升降打断）
										{
											while(!aimed2)
											{
												Aim(&aimed2);
												osDelay(1);
											}
											goto CLIP;
											
										}
										else//普通瞄准（只关心当前）
										{
											Aim(&aimed2);
										}
									}
									else 
									{	
CLIP:									aimed2 = 0;
											chassis.vx_set = 0;
											chassis.vy_set = 0;
											close_openmv();
											clipit();
											if(chassis.position_x < LAD1_DIS + 150000 && chassis.position_x > LAD1_DIS - 150000) //平台1的角落球,特殊处理
											{
												while(chassis.position_x > LAD1_DIS - 250000)
												{
													chassis.vx_set = -LAD_SPD;//回走
													osDelay(1);
												}
												chassis.vx_set = 0;
												angle_lift = LAD2_HIGH;
												osDelay(500);
											}
											open_openmv();
									}
						
				}
				osDelay(1);
			}//endwhile
			
			chassis.vx_set = 0;
			ladder = 0;
			car_reset();			
			run_back(LAD_BACK,1500);
			run_right(520000,1500);
			dingwei();
			change_pid_nomal();
#endif

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
			change_yaw_pid_run();
			chassis.chassis_yaw_set = -90;
			osDelay(3500);
			change_yaw_pid_run();
			run_front(6800000,RUN_SPD);
			run_left(4940000,RUN_SPD);
			dingwei();
			
			change_pid_slow();
			run_left(250000,1500);
			change_pid_nomal();
#endif
			
#ifdef RED
			change_yaw_pid_run();
			chassis.chassis_yaw_set = 90;
			osDelay(3500);
			change_yaw_pid_run();
			run_front(6740000,RUN_SPD);
			run_right(4940000,RUN_SPD);
			dingwei();
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
			run_left(8080000,RUN_SPD);
			
			close_openmv();
			prepare();
			change_pid_slow();
			chassis_reset();

			dingwei();
			osDelay(300);
			run_back(160000,1500);
			osDelay(300);
			run_left(156000,1500);
			
			open_openmv();
#endif
			
#ifdef RED
			run_right(8080000,RUN_SPD);
			
			close_openmv();
			prepare();
			change_pid_slow();
			chassis_reset();

			dingwei();
			osDelay(300);
			run_back(160000,1500);
			run_right(250000,1500);
			open_openmv();
#endif
			stage = takeLiZhuangBalls;
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
			run_back(500000,RUN_SPD);
			yaw_correction = YAW_CORRECT_GOHOME;
			car_reset();
			run_right(GH1_RINGT,RUN_SPD);
			dingwei();
			change_yaw_pid_turn();
			chassis.chassis_yaw_set = GH2_YAW;
			osDelay(2500);
			change_yaw_pid_run();
			run_left(GH3_LEFT,RUN_SPD);
			run_back(GH4_BACK,RUN_SPD);
#endif
			
#ifdef RED
			run_back(500000,RUN_SPD);
			car_reset();
			run_left(GH1_LEFT,RUN_SPD);
			dingwei();
			change_yaw_pid_turn();
			chassis.chassis_yaw_set = GH2_YAW;
			osDelay(2500);
			change_yaw_pid_run();
			run_right(GH3_RIGHT,RUN_SPD);
			run_back(GH4_BACK,RUN_SPD);
#endif
			stage = reset;
		}
    osDelay(1);
  }
  /* USER CODE END Gohome */
}

/* USER CODE BEGIN Header_GotoPutball */
/**
* @brief Function implementing the gotoputball thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GotoPutball */
void GotoPutball(void const * argument)
{
  /* USER CODE BEGIN GotoPutball */
  /* Infinite loop */
  for(;;)
  {
		if(stage == gotoputball)
		{
			run_left(5040000,6500);
			yaw_correction = YAW_CORRECT_PUTBALL;
			dingwei();
			run_right(520000,1500);
			stage = PutBalls;
		}
    osDelay(1);
  }
  /* USER CODE END GotoPutball */
}

/* USER CODE BEGIN Header_daoduoTask */
/**
* @brief Function implementing the DaoduoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_daoduoTask */
void daoduoTask(void const * argument)
{
  /* USER CODE BEGIN daoduoTask */
	int32_t approach_y1 = 0;
	bool cliped = false;

  /* Infinite loop */
  for(;;)
  {
		if(stage == daoduo)
		{
			change_pid_slow();
			next_lattice_v2(true,1);
			prepare();
			chassis.vx_set = 0;
			chassis.vy_set = 0;
			
			//SERVOS TEST
			//-----------------------
			//clipit_daoduo();
			//osDelay(2000);
			//putBall_daoduo();
			//-----------------------
			
			for(uint8_t j = 2; j <= 3; j++)
			{		
				
				for(uint8_t i=1; i <= 4; i++)
				{
					if(ball_exist())
					{
						approach(&approach_y1);
						chassis.vx_set = 0;
						chassis.vy_set = 0;
						clipit_daoduo();
						cliped = true;
						run_back(approach_y1,1500);
						break;
					}
					else
					{
						if(layer == 2) next_lattice_v2(1,1);
						else next_lattice_v2(0,1);
					}
				}
				
				next_lattice_v2(1, 4-lattice_daoduo);
				if(cliped) 
				{
					approach(&approach_y1);
					putBall_daoduo();
				  cliped = false;
					run_back(approach_y1,1500);
				}
				
				if(layer < 3)
				{
					reset_daoduo();
					layer++;
				}
				
		  }	
			//end daoduo
			
		  car_reset();
			change_pid_nomal();
			stage = gotoputball;
		}
    osDelay(1);
  }
  /* USER CODE END daoduoTask */
}

/* USER CODE BEGIN Header_lizhuangTask */
/**
* @brief Function implementing the lizhaungtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_lizhuangTask */
void lizhuangTask(void const * argument)
{
  /* USER CODE BEGIN lizhuangTask */
	float initial_yaw =0.0;
	int16_t d_y = 0;
  /* Infinite loop */
  for(;;)
  {
		if(stage == takeLiZhuangBalls)
		{
			initial_yaw = chassis.chassis_yaw_set;
		
		
			while(chassis.chassis_yaw > initial_yaw - 360.0 && chassis.chassis_yaw < initial_yaw+360.0)//one cycle
			{
				
				chassis.vx_set = 515;
				chassis.chassis_yaw_set -=0.0075;
				
				if(ball_x > 65 && ball_x < 75)//have found ball
				{
					chassis.vx_set = 0;
					chassis.vy_set = 0;
					chassis.chassis_yaw_set = chassis.chassis_yaw;
					
					chassis_reset();
					
					while(ball_y < 45 || ball_y > 55)
					{
						if(ball_y != 0)
						{
							chassis.vy_set = (50 - ball_y) * 12.0f;
						}
						osDelay(1);					
					}
					chassis.vy_set = 0;
					d_y = chassis.position_y;
					
					close_openmv();
					clipit();
					
					if(d_y > 0) 
					{
						run_front(d_y,1500);
					}
					else 
					{
						run_back(d_y,1500);
					}
					
					open_openmv();
				}
				osDelay(1);
			}//endwhile
			chassis.vx_set = 0;
			chassis.vy_set = 0;
			run_back(850000,1500);
			car_reset();
			run_front(850000,1500);
			change_pid_nomal();
			change_yaw_pid_turn();
			yaw_correction = YAW_CORRECT_LIZHUANG;
			chassis.chassis_yaw_set = initial_yaw - 180.0;
			osDelay(3000);
			change_yaw_pid_run();
			
			stage = gotoDaoduo;
		}
		
    osDelay(1);
  }
  /* USER CODE END lizhuangTask */
}

/* USER CODE BEGIN Header_gotodaoduo */
/**
* @brief Function implementing the gotoDaoduo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gotodaoduo */
void gotodaoduo(void const * argument)
{
  /* USER CODE BEGIN gotodaoduo */
  /* Infinite loop */
  for(;;)
  {
		if(stage == gotoDaoduo)
		{
			change_pid_nomal();
			run_left(4160000,RUN_SPD);
			dingwei();
			run_right(520000,1500);
			stage = daoduo;
		}
    osDelay(1);
  }
  /* USER CODE END gotodaoduo */
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
	else if(stage == daoduo)
	{
		angle_lift = 0;
		osDelay(1000);
	}
	else if(stage == takeLiZhuangBalls)//maybe to change
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
	else if(stage == goHome)
	{
		angle_lift = 0;
		servos.arm = 120;
		servos.move = 280;
		osDelay(800);
		servos.yindao = 160;
		servos.clip = 80;
		servos.bo = 46;
		osDelay(500);
		servos.arm = 30;
	}
}

void Aim(bool* Aimed)
{
	int16_t error_x = ball_x - 70;
	int16_t error_y = ball_y - 50;
//	if(error_x < 35 && error_x > -35 ) error_x *= 1.5;
//	if(error_y < 35 && error_y > -35 ) error_y *= 1.5;	
	chassis.vx_set = error_x * 12.0f; 
	chassis.vy_set = -error_y * 12.0f;
	if(error_x > - 5 && error_x < 5 && error_y > -5 && error_y <5)
	{
		*Aimed = true;
	}
	
}

void  wait_ball()
{
	while(ball_x < 5 || ball_x > 15)
	{
		get_totaltime_in_seconds(&run_time);
		if(run_time.time_total > WAIT_TIME) break;
		osDelay(1);
	}
	return;
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

void next_lattice_v2(bool right, uint8_t number)
{
	uint8_t beyong_red[2] = {9,5};
	change_pid_slow();
	if(stage == daoduo)
	{
		beyong_red[0] = 9;
		beyong_red[1] = 5;
	}
	else 	if(stage == PutBalls)
	{
		beyong_red[0] = 9;
		beyong_red[1] = 5;
	}
	
	for(uint8_t i=0; i<number; i++)
	{
		if(right)
		{
			while(beyoundred[beyong_red[0]])
			{
				chassis.vx_set = -WEAR_SPD;
				osDelay(1);
			}
			run_right(650000,WEAR_SPD);
			lattice_daoduo ++;
		}
		else
		{
			while(beyoundred[beyong_red[1]])
			{
				chassis.vx_set = WEAR_SPD;
				osDelay(1);
			}
			run_left(780000,WEAR_SPD);
			lattice_daoduo--;
		}
	}
	change_pid_nomal();
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
	change_pid_slow();
	while(far)
	{
		chassis.vy_set = -APPROACH_SPD;
		osDelay(1);
	}
	chassis.vy_set = 0;
	change_pid_nomal();
	*approach_y = -chassis.position_y;
}



void put_in(uint8_t x , uint8_t y)
{
	if(x == 1) angle_lift = 0;
	if(x == 2) angle_lift = 386448;
	if(x == 3) angle_lift = 500000;
	put_a_ball(x,y);
	osDelay(1000);
	servos.move = 60;
	osDelay(1200);
	servos.clip = 155;
	osDelay(150);
	servos.move = 280;
	if(x==3) servos.arm = 116;
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
		//servos.move = 70;
		servos.arm = 116;
		servos.bo = 120;
		servos.top = 110;
		osDelay(500);
		angle_lift = 435000;
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
		servos.move = 280;
		servos.yindao =  68;
		
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
	else if(stage == daoduo)
	{
		servos.top = 240;
		servos.bo = 142;
		servos.arm = 38;
		angle_lift = 84672;
		osDelay(800);
	}
	else if(stage == gotoWearhouse)
	{ 
		servos.top = 135;
		osDelay(500);
		angle_lift = 330000;
		osDelay(1500);
		servos.arm = 120;
		osDelay(500);
		servos.top = 300;
		servos.bo = 45;
		servos.clip = 140;
		servos.move = 280;
		servos.yindao =  68;
	}
	
	
}
void boit()
{
	servos.bo = 0;
	servos.top = 70;
	osDelay(300);
	servos.bo = 50;
	servos.top = 110;	
	servos.move = 280;
	osDelay(800);
	turn_box(0,1);
	osDelay(400);
	//servos.move = 100;
	osDelay(300);
}

void clipit()
{
	servos.bo = 100;
	servos.arm = 103;
	osDelay(300);
	servos.clip = 155;
	osDelay(300);
	servos.arm = 120;
	osDelay(300);
	servos.top = 50;
	osDelay(800);
	servos.arm = 88;
	osDelay(500);
	servos.clip = 140;
	osDelay(700);
	servos.move = 280;
	servos.arm = 120;
	osDelay(700);
	servos.bo = 45;
	servos.yindao = 107;
	osDelay(500);
	turn_box(0,1);
	servos.top = 300;
	osDelay(500);
	servos.yindao = 68;
//	servos.move = 0;
//	osDelay(300);
}


bool ball_exist(void)
{
	if(RC_CtrlData.rc.sw1 == 1)
	{
		sw1 = 1;
	}
	else	if(RC_CtrlData.rc.sw1 == 2)
	{
		sw1 = 3;
	}
	else if(RC_CtrlData.rc.sw1 == 3)
	{
		sw1 = 2;
	}
	if(RC_CtrlData.rc.sw2 == 1)
	{
		sw2 = 1;
	}
	else	if(RC_CtrlData.rc.sw2 == 2)
	{
		sw2 = 3;
	}
	else if(RC_CtrlData.rc.sw2 == 3)
	{
		sw2 = 2;
	}
	
	
	if(layer == 2)
	{
		if(sw1 == lattice_daoduo)
		{
			return true;
		}
		return false;
	}
	else if(layer == 3)
	{
		if(sw2 == lattice_daoduo)
		{
			return true;
		}
		return false;
	}
//	if(lattice_daoduo == 2)
//		return true;
//	else return false;
	
//	open_openmv();
//	osDelay(50);
//	if(ball_x || ball_y)
//	{
//		osDelay(50);
//		if(ball_x || ball_y) 
//		{
//			close_openmv();
//			return true;
//		}
//	}
//	close_openmv();
//	return false;
	return false;
}


void clipit_daoduo(void)
{
	if(layer == 2) angle_lift = 155866;
	else angle_lift = 520000;
	servos.clip = 109;
	servos.top = 120;
	osDelay(1800);
	servos.arm = 116;
	osDelay(500);
	servos.clip = 134;
	osDelay(1000);
	servos.top = 300;
	osDelay(1800);
	servos.arm = 110;
	osDelay(300);
	servos.clip = 155;
	osDelay(300);
	servos.arm = 116;
	osDelay(300);
	servos.top = 50;
	osDelay(1800);
}
void putBall_daoduo(void)
{
	servos.top = 270;
	osDelay(1800);
	servos.clip = 127;
	osDelay(300);
	servos.top = 120;
	osDelay(1000);
	servos.clip = 109;
	osDelay(800);
	servos.arm = 38;
	osDelay(500);
}
void reset_daoduo(void)
{
	angle_lift = 444948;
	osDelay(1000);
}


void dingwei(void)
{
#ifdef BLUE
	if(stage == gotoFirstPlatform)
	{
		while(beyoundred[0])
		{
			chassis.vx_set = -3000;
			osDelay(1);
		}
		chassis.vx_set = 0;
		while(beyoundred[4])
		{
			chassis.vy_set = -3000;
			osDelay(1);
		}
		chassis.vy_set = 0;
	}
	else if(stage == takeFirstPlatformBalls)
	{
		while(beyoundred[0])
		{
			chassis.vx_set = -3000;
			osDelay(1);
		}
		chassis.vx_set = 0;
		while(beyoundred[4])
		{
			chassis.vy_set = -3000;
			osDelay(1);
		}
		chassis.vy_set = 0;
	}
	else if(stage == gotoSecondPlatform)
	{
		while(beyoundred[5])
		{
			chassis.vx_set = 3000;
			osDelay(1);
		}
		chassis.vx_set = 0;
		while(beyoundred[9])
		{
			chassis.vy_set = 3000;
			osDelay(1);
		}
		chassis.vy_set = 0;
	}
	else if(stage == gotoWearhouse)
	{
		osDelay(1000);
		while(beyoundred[4])
		{
			chassis.vx_set = 3000;
			osDelay(1);
		}
		chassis.vx_set = 0;
		while(beyoundred[0])
		{
			chassis.vy_set = -3000;
			osDelay(1);
		}
		chassis.vy_set = 0;
	}
	else if(stage == gotoDaoduo)
	{
		while(beyoundred[0])
		{
			chassis.vx_set = -3000;
			osDelay(1);
		}
		chassis.vx_set = 0;
		while(beyoundred[4])
		{
			chassis.vy_set = -3000;
			osDelay(1);
		}
		chassis.vy_set = 0;
		run_back(1000000,1500);
	}
	else if(stage == gotoputball)
	{
		while(beyoundred[0])
		{
			chassis.vx_set = -3000;
			osDelay(1);
		}
		chassis.vx_set = 0;
		while(beyoundred[4])
		{
			chassis.vy_set = -3000;
			osDelay(1);
		}
		chassis.vy_set = 0;
		run_back(1000000,1500);
	}
	else if(stage == goHome)
	{
		while(beyoundred[0])
		{
			chassis.vx_set = -3000;
			osDelay(1);
		}
		chassis.vx_set = 0;
		while(beyoundred[4])
		{
			chassis.vy_set = -3000;
			osDelay(1);
		}
		chassis.vy_set = 0;
		run_back(1000000,1500);
	}
#endif
	
#ifdef RED
	if(stage == gotoFirstPlatform)
	{
		while(beyoundred[4])
		{
			chassis.vx_set = 3000;
			osDelay(1);
		}
		chassis.vx_set = 0;
		while(beyoundred[0])
		{
			chassis.vy_set = -3000;
			osDelay(1);
		}
		chassis.vy_set = 0;
	}
	else if(stage == takeFirstPlatformBalls)
	{
		while(beyoundred[4])
		{
			chassis.vx_set = 3000;
			osDelay(1);
		}
		chassis.vx_set = 0;
		while(beyoundred[0])
		{
			chassis.vy_set = -3000;
			osDelay(1);
		}
		chassis.vy_set = 0;
	}
	else if(stage == gotoSecondPlatform)
	{
		while(beyoundred[7])
		{
			chassis.vx_set = -3000;
			osDelay(1);
		}
		chassis.vx_set = 0;
		while(beyoundred[5])
		{
			chassis.vy_set = 3000;
			osDelay(1);
		}
		chassis.vy_set = 0;
	}
	else if(stage == gotoWearhouse)
	{
		osDelay(1000);
		while(beyoundred[9])
		{
			chassis.vx_set = -3000;
			osDelay(1);
		}
		chassis.vx_set = 0;
		while(beyoundred[4])
		{
			chassis.vy_set = -3000;
			osDelay(1);
		}
		chassis.vy_set = 0;
	}
	else if(stage == gotoDaoduo)
	{
		while(beyoundred[0])
		{
			chassis.vx_set = -3000;
			osDelay(1);
		}
		chassis.vx_set = 0;
		while(beyoundred[4])
		{
			chassis.vy_set = -3000;
			osDelay(1);
		}
		chassis.vy_set = 0;
		run_back(1000000,1500);
	}
	else if(stage == gotoputball)
	{
		while(beyoundred[0])
		{
			chassis.vx_set = -3000;
			osDelay(1);
		}
		chassis.vx_set = 0;
		while(beyoundred[4])
		{
			chassis.vy_set = -3000;
			osDelay(1);
		}
		chassis.vy_set = 0;
		run_back(1000000,1500);
	}
	else if(stage == goHome)
	{
		while(beyoundred[4])
		{
			chassis.vx_set = 3000;
			osDelay(1);
		}
		chassis.vx_set = 0;
		while(beyoundred[0])
		{
			chassis.vy_set = -3000;
			osDelay(1);
		}
		chassis.vy_set = 0;
		run_back(1000000,1500);
	}
#endif
}


/* USER CODE END Application */

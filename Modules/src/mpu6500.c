#include "mpu6500.h"
#include "math.h"
#include "spi.h"

#define filter_high 0.5
#define filter_low  0.5
#define MPU_HSPI hspi5
#define MPU_NSS_LOW  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU_NSS_HIGH HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)

//#define g_filter_high 0.8
//#define g_filter_low  0.2

static uint8_t        tx, rx;
static uint8_t        tx_buff[14] = { 0xff };

inertial_sensor_data mpu6500;
inertial_sensor_data mpu6500_raw;
inertial_sensor_data mpu6500_offest;
//the variable contains the accelation (body frame) converted from sensor data.
FLOAT_XYZ body_acc;		//unit m/s^2.
FLOAT_XYZ body_gyro;  //unit rad/s.
FLOAT_XYZ body_gyro_2; //for angluar acceleration control
FLOAT_XYZ hor_acc;		//plane acceleration in horizontal coordinates.
FLOAT_XYZ ned_acc;
//FLOAT_XYZ body_magn;

uint8_t mpu_buf[14];

/**
  * @brief  write a byte of data to specified register
  * @param  reg:  the address of register to be written
  *         data: data to be written
  * @retval 
  * @usage  call in ist_reg_write_by_mpu(),         
  *                 ist_reg_read_by_mpu(), 
  *                 mpu_master_i2c_auto_read_config(), 
  *                 ist8310_init(), 
  *                 mpu_set_gyro_fsr(),             
  *                 mpu_set_accel_fsr(), 
  *                 mpu_device_init() function
  */
uint8_t mpu_write_byte(uint8_t const reg, uint8_t const data)
{
    MPU_NSS_LOW;
    tx = reg & 0x7F;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    tx = data;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    MPU_NSS_HIGH;
    return 0;
}

/**
  * @brief  read a byte of data from specified register
  * @param  reg: the address of register to be read
  * @retval 
  * @usage  call in ist_reg_read_by_mpu(),         
  *                 mpu_device_init() function
  */
uint8_t mpu_read_byte(uint8_t const reg)
{
    MPU_NSS_LOW;
    tx = reg | 0x80;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    MPU_NSS_HIGH;
    return rx;
}

/**
  * @brief  read bytes of data from specified register
  * @param  reg: address from where data is to be written
  * @retval 
  * @usage  call in ist8310_get_data(),         
  *                 mpu_get_data(), 
  *                 mpu_offset_call() function
  */
uint8_t mpu_read_bytes(uint8_t const regAddr, uint8_t* pData, uint8_t len)
{
    MPU_NSS_LOW;
    tx         = regAddr | 0x80;
    tx_buff[0] = tx;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(&MPU_HSPI, tx_buff, pData, len, 55);
    MPU_NSS_HIGH;
    return 0;
}

/**
	* @brief  set imu 6500 gyroscope measure range
  * @param  fsr: range(0,+-250dps;1,+-500dps;2,+-1000dps;3,+-2000dps)
	* @retval 
  * @usage  call in MPU6500_Init() function
	*/
uint8_t mpu_set_gyro_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_GYRO_CONFIG, fsr << 3);
}


/**
	* @brief  set imu 6050/6500 accelerate measure range
  * @param  fsr: range(0,±2g;1,±4g;2,±8g;3,±16g)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t mpu_set_accel_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_ACCEL_CONFIG, fsr << 3); 
}

/**
	* @brief  get the offset data of MPU6500
  * @param  
	* @retval 
  * @usage  call in MPU6500_Init() function
	*/
void mpu_offset_call(void)
{
	int i;
	for (i=0; i<300;i++)
	{
		mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buf, 14);

		mpu6500_offest.ax += mpu_buf[0] << 8 | mpu_buf[1];
		mpu6500_offest.ay += mpu_buf[2] << 8 | mpu_buf[3];
		mpu6500_offest.az += mpu_buf[4] << 8 | mpu_buf[5];
	
		mpu6500_offest.gx += mpu_buf[8]  << 8 | mpu_buf[9];
		mpu6500_offest.gy += mpu_buf[10] << 8 | mpu_buf[11];
		mpu6500_offest.gz += mpu_buf[12] << 8 | mpu_buf[13];

		HAL_Delay(5);
	}
	mpu6500_offest.ax=mpu6500_offest.ax / 300;
	mpu6500_offest.ay=mpu6500_offest.ay / 300;
	mpu6500_offest.az=mpu6500_offest.az / 300;
	mpu6500_offest.gx=mpu6500_offest.gx / 300;
	mpu6500_offest.gy=mpu6500_offest.gy / 300;
	mpu6500_offest.gz=mpu6500_offest.gz / 300;
}


uint8_t id;
/**
	* @brief  initialize mpu6500
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void MPU6500_Init(void)
{

	id                               = mpu_read_byte(MPU6500_WHO_AM_I);
	uint8_t i                        = 0;
	uint8_t MPU6500_Init_Data[10][2] = {{ MPU6500_PWR_MGMT_1, 0x80 },     /* Reset Device */ 
										{ MPU6500_PWR_MGMT_1, 0x03 },     /* Clock Source - Gyro-Z */ 
										{ MPU6500_PWR_MGMT_2, 0x00 },     /* Enable Acc & Gyro */ 
										{ MPU6500_CONFIG, 0x04 },         /* LPF 41Hz */ 
										{ MPU6500_GYRO_CONFIG, 0x18 },    /* +-2000dps */ 
										{ MPU6500_ACCEL_CONFIG, 0x18 },   /* +-16G */ 
										{ MPU6500_ACCEL_CONFIG_2, 0x02 }, /* enable LowPassFilter  Set Acc LPF */ 
										{ MPU6500_USER_CTRL, 0x20 },};    /* Enable AUX */ 
	for (i = 0; i < 10; i++)
	{
		mpu_write_byte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
		HAL_Delay(1);
	}

	mpu_set_gyro_fsr(3); 		
	mpu_set_accel_fsr(2);

	mpu_offset_call();
}

void MPU6500_Get_Motion6(void)
{

	mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buf, 14);

	mpu6500_raw.ax = (((int16_t)mpu_buf[0]) << 8) | mpu_buf[1];
	mpu6500_raw.ay = (((int16_t)mpu_buf[2]) << 8) | mpu_buf[3];
	mpu6500_raw.az = (((int16_t)mpu_buf[4]) << 8) | mpu_buf[5];

	mpu6500_raw.gx = ((((int16_t)mpu_buf[8])  << 8) | mpu_buf[9]) -mpu6500_offest.gx;
	mpu6500_raw.gy = ((((int16_t)mpu_buf[10]) << 8) | mpu_buf[11])-mpu6500_offest.gy;
	mpu6500_raw.gz = ((((int16_t)mpu_buf[12]) << 8) | mpu_buf[13])-mpu6500_offest.gz;

	mpu6500.ax=mpu6500.ax*filter_high+mpu6500_raw.ax*filter_low;
	mpu6500.ay=mpu6500.ay*filter_high+mpu6500_raw.ay*filter_low;
	mpu6500.az=mpu6500.az*filter_high+mpu6500_raw.az*filter_low;


	mpu6500.gx =(mpu6500.gx+mpu6500_raw.gx)/2.0f;
	mpu6500.gy =(mpu6500.gy+mpu6500_raw.gy)/2.0f;
	mpu6500.gz =(mpu6500.gz+mpu6500_raw.gz)/2.0f;


	body_acc.x=mpu6500.ax*ACC_SCALE;
	body_acc.y=mpu6500.ay*ACC_SCALE;
	body_acc.z=mpu6500.az*ACC_SCALE;

	body_gyro.x=mpu6500.gx*GYRO_SCALE;
	body_gyro.y=mpu6500.gy*GYRO_SCALE;
	body_gyro.z=mpu6500.gz*GYRO_SCALE;

//	printf("%d\r\n", body_gyro.x );
//	printf("%d\r\n", body_gyro.y );
//	printf("%d\r\n", body_gyro.z );
}


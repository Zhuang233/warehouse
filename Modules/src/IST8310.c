#include "mpu6500.h"
#include "IST8310.h"

uint8_t               ist_buff[6];   

void IST8310_Write_Reg(uint8_t addr, uint8_t data)
{
	MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL,0x00);
	delay_ms(2);
	MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG,addr);
	delay_ms(2);
	MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, data);
	delay_ms(2);
	MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
	delay_ms(10);
}

uint8_t IST8310_Read_Reg(uint8_t addr)
{
	uint8_t data;
	MPU6500_Write_Reg(MPU6500_I2C_SLV4_REG,addr);
	delay_ms(10);
	MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x80);
	delay_ms(10);
	MPU6500_Read_Regs(&data,MPU6500_I2C_SLV4_DI,1);
	MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
	delay_ms(10);
	
	return data;
}

static void mpu6500_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
    /* 
	   * configure the device address of the IST8310 
     * use slave1, auto transmit single measure mode 
	   */
    MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, device_address);
    delay_ms(2);
    MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
    delay_ms(2);
    MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
    delay_ms(2);

    /* use slave0,auto read data */
    MPU6500_Write_Reg(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
    delay_ms(2);
    MPU6500_Write_Reg(MPU6500_I2C_SLV0_REG, reg_base_addr);
    delay_ms(2);

    /* every eight mpu6500 internal samples one i2c master read */
    MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x03);
    delay_ms(2);
    /* enable slave 0 and 1 access delay */
    MPU6500_Write_Reg(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
    delay_ms(2);
    /* enable slave 1 auto transmit */
    MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
		/* Wait 6ms (minimum waiting time for 16 times internal average setup) */
    delay_ms(6); 
    /* enable slave 0 with data_num bytes reading */
    MPU6500_Write_Reg(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
    delay_ms(2);
}

uint8_t ist8310_init()
{
	  /* enable iic master mode */
    MPU6500_Write_Reg(MPU6500_USER_CTRL, 0x30);
    delay_ms(10);
	  /* enable iic 400khz */
    MPU6500_Write_Reg(MPU6500_I2C_MST_CTRL, 0x0d); 
    delay_ms(10);

    /* turn on slave 1 for ist write and slave 4 to ist read */
    MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);  
    delay_ms(10);
    MPU6500_Write_Reg(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
    delay_ms(10);

    /* IST8310_R_CONFB 0x01 = device rst */
    IST8310_Write_Reg(IST8310_R_CONFB, 0x01);
    delay_ms(10);
    if (IST8310_DEVICE_ID_A != IST8310_Read_Reg(IST8310_WHO_AM_I))
        return 1;

		/* soft reset */
    IST8310_Write_Reg(IST8310_R_CONFB, 0x01); 
    delay_ms(10);

		/* config as ready mode to access register */
    IST8310_Write_Reg(IST8310_R_CONFA, 0x00); 
    if (IST8310_Read_Reg(IST8310_R_CONFA) != 0x00)
        return 2;
    delay_ms(10);

		/* normal state, no int */
    IST8310_Write_Reg(IST8310_R_CONFB, 0x00);
    if (IST8310_Read_Reg(IST8310_R_CONFB) != 0x00)
        return 3;
    delay_ms(10);
		
    /* config low noise mode, x,y,z axis 16 time 1 avg */
    IST8310_Write_Reg(IST8310_AVGCNTL, 0x24); //100100
    if (IST8310_Read_Reg(IST8310_AVGCNTL) != 0x24)
        return 4;
    delay_ms(10);

    /* Set/Reset pulse duration setup,normal mode */
    IST8310_Write_Reg(IST8310_PDCNTL, 0xc0);
    if (IST8310_Read_Reg(IST8310_PDCNTL) != 0xc0)
        return 5;
    delay_ms(10);

    /* turn off slave1 & slave 4 */
    MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
    delay_ms(10);
    MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
    delay_ms(10);

    /* configure and turn on slave 0 */
    mpu6500_master_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
    delay_ms(100);
    return 0;
}

void ist8310_get_data(void)
{
    MPU6500_Read_Regs(ist_buff,MPU6500_EXT_SENS_DATA_00, 6); 
}

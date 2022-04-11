#include "hmc5983.h"

void HMC5983_CS_LOW(void)
{
	GPIO_ResetBits(GPIOD, GPIO_Pin_5);
}

void HMC5983_CS_HIGH(void)
{
	GPIO_SetBits(GPIOD, GPIO_Pin_5);
}

void HMC5983_Pin_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
		GPIO_SetBits(GPIOD, GPIO_Pin_5);
}

void HMC5983_Write_Reg(unsigned char addr,unsigned char value)
{
	u8 temp=0;
	HMC5983_CS_LOW();
	SPI1_ReadWriteByte(addr,&temp);
	SPI1_ReadWriteByte(value,&temp);
	HMC5983_CS_HIGH();
}

void HMC5983_Read_Regs(unsigned char *buf,uint8_t addr,uint8_t num)
{
	uint8_t i;
	u8 temp=0;
	HMC5983_CS_LOW();
	SPI1_ReadWriteByte(addr|=0x80,&temp);
	for(i=0;i<num;i++)
		SPI1_ReadWriteByte(0xFF,buf+i);
	HMC5983_CS_HIGH();
}

unsigned char hmc5983_id;
unsigned char hmc5983_id_B;
u8 spi_err_t;
void HMC5983_Init(void)
{
	HMC5983_Pin_Init();
//	delay_ms(1);
//	HMC5983_Read_Regs(&hmc5983_id,HMC5983_IDENTIFY_REG_A,1);
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
	GPIO_SetBits(GPIOC, GPIO_Pin_0);
	GPIO_SetBits(GPIOD, GPIO_Pin_3|GPIO_Pin_5);
	HMC5983_CS_LOW();
	spi_err_t=SPI1_ReadWriteByte(HMC5983_IDENTIFY_REG_A|0x80,&hmc5983_id);
	spi_err_t=SPI1_ReadWriteByte(0xff,&hmc5983_id_B);
	HMC5983_CS_HIGH();
}

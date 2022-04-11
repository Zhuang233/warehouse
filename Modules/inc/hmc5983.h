#ifndef __HMC5983_H
#define __HMC5983_H

#include "main.h"

#define	HMC5983_CONFIG_REG_A					0x00
#define	HMC5983_CONFIG_REG_B					0x01  
#define	HMC5983_MOD_REG								0x02  
#define	HMC5983_DATA_X_MSB						0x03  
#define	HMC5983_DATA_X_LSB						0x04  
#define	HMC5983_DATA_Z_MSB						0x05  
#define	HMC5983_DATA_Z_LSB						0x06  
#define	HMC5983_DATA_Y_MSB						0x07  
#define	HMC5983_DATA_Y_LSB						0x08  
#define	HMC5983_STATE_REG	  					0x09  
#define	HMC5983_IDENTIFY_REG_A				0x0A  
#define	HMC5983_IDENTIFY_REG_B				0x0B  
#define	HMC5983_IDENTIFY_REG_C				0x0C  
#define HMC5983_TEMP_MSB							0x31
#define HMC5983_TEMP_LSB							0x32

void HMC5983_Init(void);

#endif


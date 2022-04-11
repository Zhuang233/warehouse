#ifndef _ANOUSART_H
#define _ANOUSART_H
#include "sys.h"

extern u8 Rx_Buf[];
void Usart2_ANO_Init(u32 br_num);
void Usart2_IRQ(void);
void Usart2_Send(unsigned char *DataToSend ,u8 data_num);

void Uart4_Init(u32 br_num);
void Uart4_Send(unsigned char *DataToSend ,u8 data_num);


void Uart5_Init(u32 br_num);
void Uart5_IRQ(void);
void Uart5_Send(unsigned char *DataToSend ,u8 data_num);
#endif

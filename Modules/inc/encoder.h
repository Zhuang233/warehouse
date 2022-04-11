#ifndef __ENCODER_H
#define __ENCODER_H
#include "sys.h"

void TIM5_Encoder_init(void);
void TIM8_Encoder_init(void);
void TIM3_Int_Init(u16 arr,u16 psc);

#endif


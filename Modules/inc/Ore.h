#ifndef __ORE_H
#define __ORE_H
#include "main.h"
#include <stdbool.h>

#define SMALL 1
#define BIG 2

extern uint8_t fetch_stage;
extern uint8_t fetch_flag;

void ONE_Key_Fetch(void);
void ONE_Key_Exchange(void);
void Ore_Spin(void);
void Ore_Suckin(void);

#endif

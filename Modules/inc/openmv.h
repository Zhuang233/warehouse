#ifndef __OPENMV_H
#define __OPENMV_H
#include "stdint.h"
#include "stdbool.h"

extern uint8_t openmv_data;
extern uint8_t ball_x;
extern uint8_t ball_y;


void decode_openmv_data(void);
void close_openmv(void);
void open_openmv(void);

#endif


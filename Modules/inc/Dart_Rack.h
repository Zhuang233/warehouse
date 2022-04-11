#ifndef __DARK_RACK_H
#define __DARK_RACK_H

#include "main.h"

void float_constraint(float *data,float max,float min);
void int16_constraint(int16_t *data,int16_t max,int16_t min);
void Dart_Rack_PidInit(void);
void Dart_Rack_Control(void);

#endif
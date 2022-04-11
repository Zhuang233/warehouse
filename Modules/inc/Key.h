#ifndef __KEY_H
#define __KEY_H

#include "stdbool.h"
#include "stdint.h"

typedef struct 
{
	uint16_t code;
	bool 	press_flag;
	bool	toggle_flag;
}KEY_MSG;

typedef struct
{
	KEY_MSG KEY_W;
	KEY_MSG KEY_S;
	KEY_MSG KEY_A;
	KEY_MSG KEY_D;
	KEY_MSG KEY_SHIFT;
	KEY_MSG KEY_CTRL;
	KEY_MSG	KEY_Q;
	KEY_MSG KEY_E;
	
	KEY_MSG KEY_R;
	KEY_MSG KEY_F;
	KEY_MSG KEY_G;
	KEY_MSG KEY_Z;
	KEY_MSG KEY_X;
	KEY_MSG KEY_C;
	KEY_MSG KEY_V;
	KEY_MSG KEY_B;
	
}KEYS;

void Key_Init(void);
bool Key_Check_Press(KEY_MSG* msg);
bool Key_Check_Hold(KEY_MSG* msg);
bool Key_Check_Toggle(KEY_MSG* msg);


extern KEYS Keys;

#endif	


#ifndef __DECODE_H
#define __DECODE_H
#include "sys.h"

void Resort(int *buff,int Last_One);//Êý×é»¬¶¯
void fill_buf(u16 data,int *buff,int cnt,float *sum,int X_Y);

u8 Add_CRC(u8 InputBytes[],u8 data_lenth);
void Data_disintegrate(u16 Data,u8 *LData,u8 *HData);
void Data_Code(u16 Data,int X_Y);

u16 Data_assemble(u8 *LData,u8 *HData);
u16 Data_Decode(u8 *rc_buf);


#endif


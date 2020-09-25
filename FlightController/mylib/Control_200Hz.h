#ifndef _Control_200Hz_H
#define _Control_200Hz_H

int int_constrain (int val, int low, int high);
float float_constrain (float val, float low, float high);
int int_remap(int val, int Origin_low, int Origin_high, int low, int high);
void tim4_init(void);
void my_tim4_IRQHandler(void);
void NRF_Data_Receive(void);
void PID_UPdata(void);

#endif

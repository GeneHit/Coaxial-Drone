
#ifndef _PID_H_
#define _PID_H_

void PID_calculate(void);
void PID_Tune (float *Param);
void INT_Tune (int *Param);
void PID_Write (void);
void PID_Read (void);
#endif




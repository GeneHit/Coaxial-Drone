#ifndef __MPU6050_H__
#define __MPU6050_H__
#include "sysconfig.h"

#define q30  1073741824.0f

#define INT GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)
extern float Pitch,Roll,Yaw;

void MPU6050_Init(void);
void MPU6050_Pose(void);
void MPU6050_Show_Num(u8 x,u8 y,short num,u8 mode);
void MPU6050_Interrupt_Init(void);
#endif

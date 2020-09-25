#ifndef __USART2_H
#define __USART2_H
#include "stdio.h"
#include "stm32f10x.h"
//////////////////////////////////////////////////////////////////////////////////	 
//启天科技
//串口1初始化		   
//修改日期:2015/5/27
//版本：V1.3
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 

#define USART2_MAX_RECV_LEN		64				//最大接收缓存字节数
#define USART2_MAX_SEND_LEN		64				//最大发送缓存字节数

void My_usart2_init(u32 bound);
void PrintChar2(char *s);
void Usart2_Send(u16 ch);

#endif




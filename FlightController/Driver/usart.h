#ifndef __USART_H
#define __USART_H
#include "stdio.h"
#include "stm32f10x.h"
//////////////////////////////////////////////////////////////////////////////////	 
//启天科技
//串口1初始化		   
//修改日期:2015/5/27
//版本：V1.3
//All rights reserved
//********************************************************************************
//V1.3修改说明 
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
////////////////////////////////////////////////////////////////////////////////// 
extern u8 USART_RX_BUF[64];     //接收缓冲,最大63个字节.末字节为换行符 
extern u8 USART_RX_STA;         //接收状态标记	

void uart_init(u32 bound);
void PrintChar(char *s);
void UsartSend(u16 ch);
void UART1_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);
void UART1_Put_Char(unsigned char DataToSend);
#endif




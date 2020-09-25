#ifndef __USART_H
#define __USART_H
#include "stdio.h"
#include "stm32f10x.h"
//////////////////////////////////////////////////////////////////////////////////	 
//����Ƽ�
//����1��ʼ��		   
//�޸�����:2015/5/27
//�汾��V1.3
//All rights reserved
//********************************************************************************
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
////////////////////////////////////////////////////////////////////////////////// 
extern u8 USART_RX_BUF[64];     //���ջ���,���63���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u8 USART_RX_STA;         //����״̬���	

void uart_init(u32 bound);
void PrintChar(char *s);
void UsartSend(u16 ch);
void UART1_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);
void UART1_Put_Char(unsigned char DataToSend);
#endif




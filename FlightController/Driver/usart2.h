#ifndef __USART2_H
#define __USART2_H
#include "stdio.h"
#include "stm32f10x.h"
//////////////////////////////////////////////////////////////////////////////////	 
//����Ƽ�
//����1��ʼ��		   
//�޸�����:2015/5/27
//�汾��V1.3
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 

#define USART2_MAX_RECV_LEN		64				//�����ջ����ֽ���
#define USART2_MAX_SEND_LEN		64				//����ͻ����ֽ���

void My_usart2_init(u32 bound);
void PrintChar2(char *s);
void Usart2_Send(u16 ch);

#endif




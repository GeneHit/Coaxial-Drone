#ifndef __IIC_2__H
#define __IIC_2__H

#include "stm32f10x.h"
#include "sys.h"

//IO��������
#define C02_SDA_IN()  {GPIOB->CRH&=0XFFFFF0FF;GPIOB->CRH|=0X00000800;}	//PB11����ģʽ
#define C02_SDA_OUT() {GPIOB->CRH&=0XFFFFF0FF;GPIOB->CRH|=0X00000300;}	//PB11���ģʽ

//IO��������	
#define C02_IIC_SCL    PBout(11)	//SCL
#define C02_IIC_SDA    PBout(10)	//SDA	 
#define C02_READ_SDA   PBin(10)	//����SDA

//IIC���в�������
u8 C02_IIC_Read_One_Byte(u8 daddr,u8 addr);	
u8 C02_IIC_Read_Byte(unsigned char ack);	//IIC��ȡһ���ֽ�
u8 C02_IIC_Wait_Ack(void);	//IIC�ȴ�ACK�ź�
void C02_IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
void C02_IIC_Send_Byte(u8 txd);	//IIC����һ���ֽ�
void C02_IIC_Start(void);	//����IIC��ʼ�ź�
void C02_IIC_Init(void);	//��ʼ��IIC��IO��				 
void C02_IIC_NAck(void);	//IIC������ACK�ź
void C02_IIC_Stop(void);	//����IICֹͣ�ź�
void C02_IIC_Ack(void);	//IIC����ACK�ź�

#endif


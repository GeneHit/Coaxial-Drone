#ifndef __IIC_2__H
#define __IIC_2__H

#include "stm32f10x.h"
#include "sys.h"

//IO方向设置
#define C02_SDA_IN()  {GPIOB->CRH&=0XFFFFF0FF;GPIOB->CRH|=0X00000800;}	//PB11输入模式
#define C02_SDA_OUT() {GPIOB->CRH&=0XFFFFF0FF;GPIOB->CRH|=0X00000300;}	//PB11输出模式

//IO操作函数	
#define C02_IIC_SCL    PBout(11)	//SCL
#define C02_IIC_SDA    PBout(10)	//SDA	 
#define C02_READ_SDA   PBin(10)	//输入SDA

//IIC所有操作函数
u8 C02_IIC_Read_One_Byte(u8 daddr,u8 addr);	
u8 C02_IIC_Read_Byte(unsigned char ack);	//IIC读取一个字节
u8 C02_IIC_Wait_Ack(void);	//IIC等待ACK信号
void C02_IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
void C02_IIC_Send_Byte(u8 txd);	//IIC发送一个字节
void C02_IIC_Start(void);	//发送IIC开始信号
void C02_IIC_Init(void);	//初始化IIC的IO口				 
void C02_IIC_NAck(void);	//IIC不发送ACK信�
void C02_IIC_Stop(void);	//发送IIC停止信号
void C02_IIC_Ack(void);	//IIC发送ACK信号

#endif


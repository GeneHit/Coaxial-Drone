#ifndef __IIC_2__H
#define __IIC_2__H

#include "stm32f10x.h"
#include "sys.h"

//IO·½ÏòÉèÖÃ
#define C02_SDA_IN()  {GPIOB->CRH&=0XFFFFF0FF;GPIOB->CRH|=0X00000800;}	//PB11ÊäÈëÄ£Ê½
#define C02_SDA_OUT() {GPIOB->CRH&=0XFFFFF0FF;GPIOB->CRH|=0X00000300;}	//PB11Êä³öÄ£Ê½

//IO²Ù×÷º¯Êý	
#define C02_IIC_SCL    PBout(11)	//SCL
#define C02_IIC_SDA    PBout(10)	//SDA	 
#define C02_READ_SDA   PBin(10)	//ÊäÈëSDA

//IICËùÓÐ²Ù×÷º¯Êý
u8 C02_IIC_Read_One_Byte(u8 daddr,u8 addr);	
u8 C02_IIC_Read_Byte(unsigned char ack);	//IIC¶ÁÈ¡Ò»¸ö×Ö½Ú
u8 C02_IIC_Wait_Ack(void);	//IICµÈ´ýACKÐÅºÅ
void C02_IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
void C02_IIC_Send_Byte(u8 txd);	//IIC·¢ËÍÒ»¸ö×Ö½Ú
void C02_IIC_Start(void);	//·¢ËÍIIC¿ªÊ¼ÐÅºÅ
void C02_IIC_Init(void);	//³õÊ¼»¯IICµÄIO¿Ú				 
void C02_IIC_NAck(void);	//IIC²»·¢ËÍACKÐÅº
void C02_IIC_Stop(void);	//·¢ËÍIICÍ£Ö¹ÐÅºÅ
void C02_IIC_Ack(void);	//IIC·¢ËÍACKÐÅºÅ

#endif


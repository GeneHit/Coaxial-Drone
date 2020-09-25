#ifndef __MYIIC_H
#define __MYIIC_H
#include "stm32f10x.h"
#include "sysconfig.h"
/*
                  ////////////

            /////////////////////////                   /////////////////////////////
           ////                /////                               //////
          ////                /////                               //////
         /////               /////                  /////////////////////////////////////////
        //////////////////////////                             //////  /////
       /////                                                  //////     /////
     /////    ///////////////////                            //////        /////
    ////      ////          /////                           /////            //////
   ////       ////          /////                          /////              ///////
  ////        ////          /////                         /////                ////////
 /////        ///////////////////                        /////                   /////////
//启天科技出品
//研发团队：启天四轴团队
//联系方式：QQ群：471023785
            邮箱：qitiansizhou@163.com
            淘宝：http://shop125061094.taobao.com/
//声明：此程序只可以用于学习教学，禁止用于商业用途，否则后果必究！
//版本：V2.5
//日期：20150710
//修改说明：
//
*/
#define true 1
#define false 0 

#define TRUE  0
#define FALSE -1
#define  I2C_Direction_Transmitter      ((uint8_t)0x00)	//写
#define  I2C_Direction_Receiver         ((uint8_t)0x01)	//读

#define SDA_IN()  {GPIOA->CRL&=0XFFFFFF0F;GPIOA->CRL|=0X00000080;}
#define SDA_OUT() {GPIOA->CRL&=0XFFFFFF0F;GPIOA->CRL|=0X00000030;}

//IO操作函数	 
#define IIC_SCL    PAout(0) //SCL---C0
#define IIC_SDA    PAout(1) //SDA---C1	 
#define READ_SDA   PAin(1)  //输入SDA

 
//IIC所有操作函数
void IIC_Start(void);
void IIC_Send_Byte(u8 txd);
u8 IIC_Read_Byte(void);
void IIC_Ack(void);
void IIC_NAck(void);
void IIC_Stop(void);
u8 IIC_Wait_Ack(void);
void IIC_Init(void);                					//初始化IIC的IO口				 
u8 IIC_Write_Buffer(u8 addr, u8 reg, u8 len, u8 * data);
int IIC_Write(u8 addr, u8 reg, u8 len, u8* data);
u8 IIC_Read_Buffer(u8 addr, u8 reg, u8 len, u8* buf);
int IIC_Read(u8 addr, u8 reg, u8 len, u8 *buf);
u8 IIC_WriteOneByte(u8 addr, u8 reg, u8 data);
u16 IIC_GetErrorCounter(void);
void Single_WriteI2C(unsigned char REG_Address,unsigned char REG_data);
unsigned char Single_ReadI2C(unsigned char REG_Address);//读取单字节
	  
#endif

















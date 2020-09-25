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
//����Ƽ���Ʒ
//�з��Ŷӣ����������Ŷ�
//��ϵ��ʽ��QQȺ��471023785
            ���䣺qitiansizhou@163.com
            �Ա���http://shop125061094.taobao.com/
//�������˳���ֻ��������ѧϰ��ѧ����ֹ������ҵ��;���������ؾ���
//�汾��V2.5
//���ڣ�20150710
//�޸�˵����
//
*/
#define true 1
#define false 0 

#define TRUE  0
#define FALSE -1
#define  I2C_Direction_Transmitter      ((uint8_t)0x00)	//д
#define  I2C_Direction_Receiver         ((uint8_t)0x01)	//��

#define SDA_IN()  {GPIOA->CRL&=0XFFFFFF0F;GPIOA->CRL|=0X00000080;}
#define SDA_OUT() {GPIOA->CRL&=0XFFFFFF0F;GPIOA->CRL|=0X00000030;}

//IO��������	 
#define IIC_SCL    PAout(0) //SCL---C0
#define IIC_SDA    PAout(1) //SDA---C1	 
#define READ_SDA   PAin(1)  //����SDA

 
//IIC���в�������
void IIC_Start(void);
void IIC_Send_Byte(u8 txd);
u8 IIC_Read_Byte(void);
void IIC_Ack(void);
void IIC_NAck(void);
void IIC_Stop(void);
u8 IIC_Wait_Ack(void);
void IIC_Init(void);                					//��ʼ��IIC��IO��				 
u8 IIC_Write_Buffer(u8 addr, u8 reg, u8 len, u8 * data);
int IIC_Write(u8 addr, u8 reg, u8 len, u8* data);
u8 IIC_Read_Buffer(u8 addr, u8 reg, u8 len, u8* buf);
int IIC_Read(u8 addr, u8 reg, u8 len, u8 *buf);
u8 IIC_WriteOneByte(u8 addr, u8 reg, u8 data);
u16 IIC_GetErrorCounter(void);
void Single_WriteI2C(unsigned char REG_Address,unsigned char REG_data);
unsigned char Single_ReadI2C(unsigned char REG_Address);//��ȡ���ֽ�
	  
#endif

















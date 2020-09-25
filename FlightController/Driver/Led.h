#ifndef _LED_H_
#define _LED_H_
#include "stm32f10x.h"
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
//�汾��V3.0
//���ڣ�20150720
//�޸�˵����
//
*/

#define LED1_OFF  		GPIO_SetBits(GPIOB, GPIO_Pin_9)
#define LED1_ON 			GPIO_ResetBits(GPIOB, GPIO_Pin_9)
#define LED2_OFF  		GPIO_SetBits(GPIOB, GPIO_Pin_8)
#define LED2_ON 			GPIO_ResetBits(GPIOB, GPIO_Pin_8)
#define LED3_OFF  		;//GPIO_SetBits(GPIOB, GPIO_Pin_14)
#define LED3_ON 		;//	GPIO_ResetBits(GPIOB, GPIO_Pin_14)
//#define LED4_OFF  	GPIO_SetBits(GPIOE, GPIO_Pin_2)
//#define LED4_ON 	GPIO_ResetBits(GPIOE, GPIO_Pin_2)
#define LEDALL_OFF  GPIO_SetBits(GPIOB, GPIO_Pin_8  | GPIO_Pin_9)// | GPIO_Pin_3 | GPIO_Pin_2)
#define LEDALL_ON 	GPIO_ResetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9)// | GPIO_Pin_3 | GPIO_Pin_2)

void LED_Init(void);
void LEDALL_FLASH(u8 times, u16 time);
#endif



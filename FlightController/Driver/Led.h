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
//启天科技出品
//研发团队：启天四轴团队
//联系方式：QQ群：471023785
            邮箱：qitiansizhou@163.com
            淘宝：http://shop125061094.taobao.com/
//声明：此程序只可以用于学习教学，禁止用于商业用途，否则后果必究！
//版本：V3.0
//日期：20150720
//修改说明：
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



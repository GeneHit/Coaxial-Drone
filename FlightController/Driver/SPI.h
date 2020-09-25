#ifndef _SPI_H_
#define _SPI_H_
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
//版本：V2.5
//日期：20150710
//修改说明：
//
*/
#define SPI_CE_H()   GPIO_SetBits(GPIOB, GPIO_Pin_7) 
#define SPI_CE_L()   GPIO_ResetBits(GPIOB, GPIO_Pin_7)

#define SPI_CSN_H()  GPIO_SetBits(GPIOA, GPIO_Pin_15)
#define SPI_CSN_L()  GPIO_ResetBits(GPIOA, GPIO_Pin_15)

void SPI1_INIT(void);
u8 SPI_RW(u8 dat);

#endif


#include "led.h"
#include "delay.h"
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
            淘宝：https://shop128265493.taobao.com/
//声明：此程序只可以用于学习教学，禁止用于商业用途，否则后果必究！
//版本：V3.0
//日期：20150810
//修改说明：
//
*/
#define  DELAY_MS delay_ms

void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//使能LED用的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE); //打开外设B的时钟
	//设置LED使用的管脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
//LED闪烁程序，times为闪烁的次数，time为每次闪烁的周期时间
void LEDALL_FLASH(u8 times, u16 time)
{
	while(times--)
	{
		LEDALL_ON;
		delay_Ms_Loop(time);
		LEDALL_OFF;
		delay_Ms_Loop(time);
	}
}



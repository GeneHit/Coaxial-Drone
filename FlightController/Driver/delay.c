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
static uint8_t  Multi_us=0;//us延时倍乘数
static uint16_t Multi_ms=0;//ms延时倍乘数
//初始化延迟函数
//SYSTICK的时钟固定为HCLK时钟的1/8
//SYSCLK:系统时钟
void delay_init(uint8_t SYSCLK)
{
	SysTick->CTRL&=0xfffffffb;//HCLK/8
	Multi_us=SYSCLK/8;		    
	Multi_ms=(u16)Multi_us*1000;
	SysTick->CTRL &= 0xFE;
}								    
//nms原则上不要超过1000，但是实测最好不要超过100，否则可能会不好使20150512
void delay_ms(uint16_t N_ms)
{	 		  	  
	static uint32_t temp;		   
	SysTick->LOAD=(u32)N_ms*Multi_ms;
	SysTick->VAL =0x00;          
	SysTick->CTRL |= 0x01 ;          
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01) && (!(temp&(1<<16))));//等待时间到达  
	SysTick->CTRL &= 0xFE;       //关闭计数器
	SysTick->VAL =0X00;       	//清空计数器	  	    
}   
//延时nus
//nus为要延时的us数.		    								   
void delay_us(u32 N_us)
{		
	static uint32_t temp;	    	 
	SysTick->LOAD=N_us*Multi_us;   		 
	SysTick->VAL=0x00;       
	SysTick->CTRL |= 0x01 ;       	 
	do
	{
		temp=SysTick->CTRL;	
		//SysTick->CTRL |= 0x01;
	}
	while((temp&0x01) && (!(temp&(1<<16))));
	SysTick->CTRL &= 0xFE;     
	SysTick->VAL =0X00;       
}

void delay_Ms_Loop(uint32_t ms)
{
	static uint32_t i;
	while(ms--)
	{
		i=1200;
		while(i--);
	}
}




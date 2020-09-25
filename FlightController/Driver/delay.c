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
//����Ƽ���Ʒ
//�з��Ŷӣ����������Ŷ�
//��ϵ��ʽ��QQȺ��471023785
            ���䣺qitiansizhou@163.com
            �Ա���https://shop128265493.taobao.com/
//�������˳���ֻ��������ѧϰ��ѧ����ֹ������ҵ��;���������ؾ���
//�汾��V3.0
//���ڣ�20150810
//�޸�˵����
//
*/
static uint8_t  Multi_us=0;//us��ʱ������
static uint16_t Multi_ms=0;//ms��ʱ������
//��ʼ���ӳٺ���
//SYSTICK��ʱ�ӹ̶�ΪHCLKʱ�ӵ�1/8
//SYSCLK:ϵͳʱ��
void delay_init(uint8_t SYSCLK)
{
	SysTick->CTRL&=0xfffffffb;//HCLK/8
	Multi_us=SYSCLK/8;		    
	Multi_ms=(u16)Multi_us*1000;
	SysTick->CTRL &= 0xFE;
}								    
//nmsԭ���ϲ�Ҫ����1000������ʵ����ò�Ҫ����100��������ܻ᲻��ʹ20150512
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
	while((temp&0x01) && (!(temp&(1<<16))));//�ȴ�ʱ�䵽��  
	SysTick->CTRL &= 0xFE;       //�رռ�����
	SysTick->VAL =0X00;       	//��ռ�����	  	    
}   
//��ʱnus
//nusΪҪ��ʱ��us��.		    								   
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




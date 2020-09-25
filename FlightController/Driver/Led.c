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
#define  DELAY_MS delay_ms

void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//ʹ��LED�õ�ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE); //������B��ʱ��
	//����LEDʹ�õĹܽ�
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
//LED��˸����timesΪ��˸�Ĵ�����timeΪÿ����˸������ʱ��
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



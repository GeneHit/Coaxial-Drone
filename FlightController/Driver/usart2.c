#include "usart2.h"
#include "usart.h"
#include "led.h"
#include "math.h"
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
//�汾��V4.0
//���ڣ�20151015
//�޸�˵����
//
*/
//���ڷ��ͻ����� 	
__align(8) u8 USART2_TX_BUF[64]; 	//���ͻ���,���USART2_MAX_SEND_LEN�ֽ�
//���ڽ��ջ����� 	
u8 USART2_RX_BUF[64]; 				//���ջ���,���USART2_MAX_RECV_LEN���ֽ�.
extern volatile uint16_t Speed_FR;
extern volatile float Desire_angle_roll,Desire_angle_pitch,Desire_w_yaw;
//u16 USART2_RX_BUF[32];     //���ջ���,���64���ֽ�.
///////////////////////////////////////USART2 DMA�������ò���//////////////////////////////////	   		    
//DMA1�ĸ�ͨ������
//����Ĵ�����ʽ�ǹ̶���,���Ҫ���ݲ�ͬ��������޸�
//�Ӵ洢��->����ģʽ/8λ���ݿ��/�洢������ģʽ
//DMA_CHx:DMAͨ��CHx
//cpar:�����ַ
//cmar:�洢����ַ    
void UART_DMA_Config(DMA_Channel_TypeDef*DMA_CHx,u32 cpar,u32 cmar, u32 Direction)
{
	DMA_InitTypeDef DMA_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1|RCC_APB2Periph_AFIO, ENABLE);	//ʹ��DMA����
	DMA_DeInit(DMA_CHx);   //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ
	DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;  //DMA�������ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = cmar;  //DMA�ڴ����ַ
	DMA_InitStructure.DMA_DIR = Direction;//���ݴ��䷽��DMA_DIR_PeripheralDST:���ڴ��ȡ���͵�����;DMA_DIR_PeripheralSRC:�������ȡ���͵��ڴ�
	DMA_InitStructure.DMA_BufferSize = 32;  //DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Normal;//;// ;  //������ѭ������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//DMA_Priority_Medium; //DMAͨ�� xӵ�������ȼ� 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA_CHx, &DMA_InitStructure);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USART1_Tx_DMA_Channel����ʶ�ļĴ���	
} 
//����һ��DMA����
void UART_DMA_Enable(DMA_Channel_TypeDef*DMA_CHx,u8 len)
{
	DMA_Cmd(DMA_CHx, DISABLE );  //�ر� ָʾ��ͨ��        
	DMA_SetCurrDataCounter(DMA_CHx,len);//DMAͨ����DMA����Ĵ�С	
	DMA_Cmd(DMA_CHx, ENABLE);           //����DMA����
}	   
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 		

void My_usart2_init(u32 bound)
{
	//GPIO�˿�����
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	 //USART2_TX   PA.2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//USART2_RX	  PA.3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���USART1
		
	//USART ��ʼ������		 
	USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	UART_DMA_Config(DMA1_Channel6,(u32)&USART2->DR,(u32)USART2_RX_BUF,DMA_DIR_PeripheralSRC);//DMA1ͨ��7,����Ϊ����2,�洢��ΪUSART2_RX_BUF,���赽�ڴ� 
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);  	//ʹ�ܴ���2��DMA����		
	USART_Init(USART2, &USART_InitStructure);
	USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ��� 
	DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);	//ʹ��DMA1_Channel6�����ݴ�������ж�	DMA1_IT_TC6
	DMA_Cmd(DMA1_Channel6, ENABLE);           //����DMA����
}

//����2�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���
//����״̬
//bit7��������ɱ�־
//bit6�����յ�0x0d
//bit5~0�����յ�����Ч�ֽ���Ŀ
u8 USART2_RX_STA=0;       //����״̬���

void PrintChar2(char *s)
{
	char *p;
	p=s;
	while(*p != '\0')
	{
		Usart2_Send(*p);
		p++;
	}
}

void Usart2_Send(u16 ch)
{
	static uint32_t i = 0;
	i = 0;
	USART2->DR = (ch&(u16)0x01FF);	
	//while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
  while( ((USART2->SR&0X40)==0) && (i<120000) )
	{
		i++;
	}//ѭ������,ֱ���������  
}
//static u8 state=0;
volatile int BT_Throttle=0, BT_Yaw=0, BT_Pitch=0, BT_Roll=0;
int usart_i=0, j=0;
u8 DMA_UART_complete = 0;
static u8 BT_Counter = 0;
extern volatile uint32_t  nfr_Receive_Fail_Counter;   //���մ����־��0.5sδ���ܵ���������Ϊ������ʧ��
//ǰ��ƫ���С���ֵ���ƫ
extern float roll_angel_offset;
//���Ҽ�飬�Ӵ������ֵ������ƫ
extern float pitch_angel_offset;//��ʼʱ�̶�ƫ��
//DMA1ͨ��6���ݴ�������жϣ�BufferSize = 32������32�����ݣ��ͻ�����ж�
//�������������жϺ���
void DMA1_Channel6_IRQHandler()
{
	u16 sum=0;	
	if(DMA_GetITStatus(DMA1_IT_TC6) != RESET)
	{
//		printf("over\r\r\n");
		DMA_ClearITPendingBit(DMA1_IT_TC6);
		usart_i=0;
		while((USART2_RX_BUF[usart_i] != 0xA5))
		{	
			usart_i++; 
			if(usart_i>16)	
			{
				usart_i=0;
				break;
			}
		}				
		if((USART2_RX_BUF[usart_i] == 0xA5)&&(usart_i<16))	//������Ч
		{
			sum=0;
			for(j=0;j<13;j++)
				sum += USART2_RX_BUF[usart_i+j];
			if((sum&0x00ff) == USART2_RX_BUF[usart_i+14])//+USART2_RX_BUF[i+31]*256)	//У���
			{	
				nfr_Receive_Fail_Counter = 0;
					BT_Counter ++;
				if(BT_Counter >100)
					BT_Counter = 0;
				if(BT_Counter % 2 == 0)
				GPIO_SetBits(GPIOB, GPIO_Pin_10);
				else
				GPIO_ResetBits(GPIOB, GPIO_Pin_10);
				BT_Throttle = (uint16_t)USART2_RX_BUF[usart_i+2]+((uint16_t)USART2_RX_BUF[usart_i+3]<<8);
				BT_Yaw = (uint16_t)USART2_RX_BUF[usart_i+4]+((uint16_t)USART2_RX_BUF[usart_i+5]<<8);
				BT_Roll = 1024 - ( (uint16_t)USART2_RX_BUF[usart_i+6]+((uint16_t)USART2_RX_BUF[usart_i+7]<<8) );
				BT_Pitch = 1024 - ( (uint16_t)USART2_RX_BUF[usart_i+8]+((uint16_t)USART2_RX_BUF[usart_i+9]<<8) );	
				roll_angel_offset = (USART2_RX_BUF[usart_i+12] - 120)/15.01;
				pitch_angel_offset = -(USART2_RX_BUF[usart_i+11] - 120)/15.01;
				//����һ����ֵ�����������ָ�
				if(fabs(BT_Yaw - 512)<300)
					BT_Yaw = 512;
				else if((BT_Yaw - 512) > 300)
					BT_Yaw = BT_Yaw - 100;
				else if((BT_Yaw - 512) < -300)
					BT_Yaw = BT_Yaw + 100;
				
				if(BT_Throttle != 0)
				{
					BT_Throttle = BT_Throttle/4;
					if(BT_Throttle < 90)//3150
					Speed_FR = 2000 + BT_Throttle*12.10;
					else if((BT_Throttle >= 90) && (BT_Throttle<=179))//3150-3570
					Speed_FR = 3089 + (BT_Throttle-90)*5;
					else if( (BT_Throttle >= 179) && (BT_Throttle<=255) )//3570-4500
					Speed_FR = 3534 + (BT_Throttle-179)*12.7;
				}
				//��roll����п��ƣ���������������ֵ�������ָ�����
				BT_Roll = BT_Roll/4;
				if(fabs((float)BT_Roll-128.0f)<60)
					Desire_angle_roll = ((float)BT_Roll-128.0f)*20.0f/128.0f - 0.7;		//������+-60��֮��
				else
				{
					if(((float)BT_Roll-128.0f) <= -60)
						Desire_angle_roll = -9.3 + ((float)BT_Roll-128.0f+60.0f)*20.0f/128.0f - 0.7;
					else
						Desire_angle_roll = 9.3 + ((float)BT_Roll-128.0f-60.0f)*20.0f/128.0f - 0.7;
				}
				
				//��pitch����п��ƣ���������������ֵ�������ָ�����
				BT_Pitch = BT_Pitch/4;
				if(fabs((float)BT_Pitch-128.0f)<60)
					Desire_angle_pitch = -((float)BT_Pitch-128.0f)*20.0f/128.0f - 0.7;	//������+-60��֮��
				else
				{
					if(((float)BT_Pitch-128.0f) <= -60)
						Desire_angle_pitch = 9.3 - ((float)BT_Pitch-128.0f+60.0f)*20.0f/128.0f - 0.7;
					else
						Desire_angle_pitch = -9.3 - ((float)BT_Pitch-128.0f-60.0f)*20.0f/128.0f - 0.7;	
				}
				
				//��yaw����п���
				BT_Yaw = BT_Yaw/4;
				Desire_w_yaw = -((float)BT_Yaw - 128.0f)*40.0f/128.0f;	//������0-50��ÿ��֮��			
			}
		}
			
	}
//		i++;
//		
	
}







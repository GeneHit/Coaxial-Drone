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
//启天科技出品
//研发团队：启天四轴团队
//联系方式：QQ群：471023785
            邮箱：qitiansizhou@163.com
            淘宝：https://shop128265493.taobao.com/
//声明：此程序只可以用于学习教学，禁止用于商业用途，否则后果必究！
//版本：V4.0
//日期：20151015
//修改说明：
//
*/
//串口发送缓存区 	
__align(8) u8 USART2_TX_BUF[64]; 	//发送缓冲,最大USART2_MAX_SEND_LEN字节
//串口接收缓存区 	
u8 USART2_RX_BUF[64]; 				//接收缓冲,最大USART2_MAX_RECV_LEN个字节.
extern volatile uint16_t Speed_FR;
extern volatile float Desire_angle_roll,Desire_angle_pitch,Desire_w_yaw;
//u16 USART2_RX_BUF[32];     //接收缓冲,最大64个字节.
///////////////////////////////////////USART2 DMA发送配置部分//////////////////////////////////	   		    
//DMA1的各通道配置
//这里的传输形式是固定的,这点要根据不同的情况来修改
//从存储器->外设模式/8位数据宽度/存储器增量模式
//DMA_CHx:DMA通道CHx
//cpar:外设地址
//cmar:存储器地址    
void UART_DMA_Config(DMA_Channel_TypeDef*DMA_CHx,u32 cpar,u32 cmar, u32 Direction)
{
	DMA_InitTypeDef DMA_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1|RCC_APB2Periph_AFIO, ENABLE);	//使能DMA传输
	DMA_DeInit(DMA_CHx);   //将DMA的通道1寄存器重设为缺省值
	DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;  //DMA外设基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = cmar;  //DMA内存基地址
	DMA_InitStructure.DMA_DIR = Direction;//数据传输方向，DMA_DIR_PeripheralDST:从内存读取发送到外设;DMA_DIR_PeripheralSRC:从外设读取发送到内存
	DMA_InitStructure.DMA_BufferSize = 32;  //DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Normal;//;// ;  //工作在循环缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//DMA_Priority_Medium; //DMA通道 x拥有中优先级 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA_CHx, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道USART1_Tx_DMA_Channel所标识的寄存器	
} 
//开启一次DMA传输
void UART_DMA_Enable(DMA_Channel_TypeDef*DMA_CHx,u8 len)
{
	DMA_Cmd(DMA_CHx, DISABLE );  //关闭 指示的通道        
	DMA_SetCurrDataCounter(DMA_CHx,len);//DMA通道的DMA缓存的大小	
	DMA_Cmd(DMA_CHx, ENABLE);           //开启DMA传输
}	   
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 		

void My_usart2_init(u32 bound)
{
	//GPIO端口设置
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

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART1
		
	//USART 初始化设置		 
	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	UART_DMA_Config(DMA1_Channel6,(u32)&USART2->DR,(u32)USART2_RX_BUF,DMA_DIR_PeripheralSRC);//DMA1通道7,外设为串口2,存储器为USART2_RX_BUF,外设到内存 
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);  	//使能串口2的DMA接收		
	USART_Init(USART2, &USART_InitStructure);
	USART_Cmd(USART2, ENABLE);                    //使能串口 
	DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);	//使能DMA1_Channel6的数据传输完成中断	DMA1_IT_TC6
	DMA_Cmd(DMA1_Channel6, ENABLE);           //开启DMA传输
}

//串口2中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误
//接收状态
//bit7，接收完成标志
//bit6，接收到0x0d
//bit5~0，接收到的有效字节数目
u8 USART2_RX_STA=0;       //接收状态标记

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
	//while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
  while( ((USART2->SR&0X40)==0) && (i<120000) )
	{
		i++;
	}//循环发送,直到发送完毕  
}
//static u8 state=0;
volatile int BT_Throttle=0, BT_Yaw=0, BT_Pitch=0, BT_Roll=0;
int usart_i=0, j=0;
u8 DMA_UART_complete = 0;
static u8 BT_Counter = 0;
extern volatile uint32_t  nfr_Receive_Fail_Counter;   //接收错误标志，0.5s未接受到数据则认为飞行器失控
//前后偏差，减小这个值向后偏
extern float roll_angel_offset;
//左右检查，加大下面的值会向左偏
extern float pitch_angel_offset;//初始时固定偏移
//DMA1通道6数据传输完成中断，BufferSize = 32，传输32个数据，就会进入中断
//这是蓝牙接受中断函数
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
		if((USART2_RX_BUF[usart_i] == 0xA5)&&(usart_i<16))	//数据有效
		{
			sum=0;
			for(j=0;j<13;j++)
				sum += USART2_RX_BUF[usart_i+j];
			if((sum&0x00ff) == USART2_RX_BUF[usart_i+14])//+USART2_RX_BUF[i+31]*256)	//校验和
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
				//设置一个阈值，提升操作手感
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
				//对roll轴进行控制，其中我设置了阈值，操作手感提升
				BT_Roll = BT_Roll/4;
				if(fabs((float)BT_Roll-128.0f)<60)
					Desire_angle_roll = ((float)BT_Roll-128.0f)*20.0f/128.0f - 0.7;		//限制在+-60度之内
				else
				{
					if(((float)BT_Roll-128.0f) <= -60)
						Desire_angle_roll = -9.3 + ((float)BT_Roll-128.0f+60.0f)*20.0f/128.0f - 0.7;
					else
						Desire_angle_roll = 9.3 + ((float)BT_Roll-128.0f-60.0f)*20.0f/128.0f - 0.7;
				}
				
				//对pitch轴进行控制，其中我设置了阈值，操作手感提升
				BT_Pitch = BT_Pitch/4;
				if(fabs((float)BT_Pitch-128.0f)<60)
					Desire_angle_pitch = -((float)BT_Pitch-128.0f)*20.0f/128.0f - 0.7;	//限制在+-60度之内
				else
				{
					if(((float)BT_Pitch-128.0f) <= -60)
						Desire_angle_pitch = 9.3 - ((float)BT_Pitch-128.0f+60.0f)*20.0f/128.0f - 0.7;
					else
						Desire_angle_pitch = -9.3 - ((float)BT_Pitch-128.0f-60.0f)*20.0f/128.0f - 0.7;	
				}
				
				//对yaw轴进行控制
				BT_Yaw = BT_Yaw/4;
				Desire_w_yaw = -((float)BT_Yaw - 128.0f)*40.0f/128.0f;	//限制在0-50度每秒之内			
			}
		}
			
	}
//		i++;
//		
	
}







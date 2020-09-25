#include "iic_2.h"
#include "delay.h"

/**************************实现函数********************************************
*函数原型:		void IIC_Init(void)
*功　　能:		初始化I2C对应的接口引脚。
*******************************************************************************/
void C02_IIC_Init(void)
{			
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB , ENABLE );	
//	AFIO->MAPR &= 0xf8ffffff;| RCC_APB2Periph_AFIO
//	AFIO->MAPR |= 0x02000000;
	
	//***在此修改IIC引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

//产生IIC起始信号
void C02_IIC_Start(void)
{
	C02_SDA_OUT();	//sda线输出
	C02_IIC_SDA=1;	  	  
	C02_IIC_SCL=1;
	delay_us(4);
 	C02_IIC_SDA=0;	//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	C02_IIC_SCL=0;	//钳住I2C总线，准备发送或接收数据 
}	  

//产生IIC停止信号
void C02_IIC_Stop(void)
{
	C02_SDA_OUT();	//sda线输出
	C02_IIC_SCL=0;
	C02_IIC_SDA=0;	//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	C02_IIC_SCL=1; 
	C02_IIC_SDA=1;	//发送I2C总线结束信号
	delay_us(4);							   	
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 C02_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	C02_SDA_IN();	//SDA设置为输入  
	C02_IIC_SDA=1;delay_us(1);	   
	C02_IIC_SCL=1;delay_us(1);	 
	while(C02_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			C02_IIC_Stop();
			return 1;
		}
	}
	C02_IIC_SCL=0;	//时钟输出0 	   
	return 0;  
} 

//产生ACK应答
void C02_IIC_Ack(void)
{
	C02_IIC_SCL=0;
	C02_SDA_OUT();
	C02_IIC_SDA=0;
	delay_us(2);
	C02_IIC_SCL=1;
	delay_us(2);
	C02_IIC_SCL=0;
}

//不产生ACK应答		    
void C02_IIC_NAck(void)
{
	C02_IIC_SCL=0;
	C02_SDA_OUT();
	C02_IIC_SDA=1;
	delay_us(2);
	C02_IIC_SCL=1;
	delay_us(2);
	C02_IIC_SCL=0;
}					 				     

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void C02_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	C02_SDA_OUT(); 	    
    C02_IIC_SCL=0;	//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
		C02_IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);	//对TEA5767这三个延时都是必须的
		C02_IIC_SCL=1;
		delay_us(2); 
		C02_IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 C02_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	C02_SDA_IN();	//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        C02_IIC_SCL=0; 
        delay_us(2);
		C02_IIC_SCL=1;
        receive<<=1;
        if(C02_READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        C02_IIC_NAck();	//发送nACK
    else
        C02_IIC_Ack();	//发送ACK   
    return receive;
}

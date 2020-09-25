#include "iic_2.h"
#include "delay.h"

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Init(void)
*��������:		��ʼ��I2C��Ӧ�Ľӿ����š�
*******************************************************************************/
void C02_IIC_Init(void)
{			
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB , ENABLE );	
//	AFIO->MAPR &= 0xf8ffffff;| RCC_APB2Periph_AFIO
//	AFIO->MAPR |= 0x02000000;
	
	//***�ڴ��޸�IIC����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

//����IIC��ʼ�ź�
void C02_IIC_Start(void)
{
	C02_SDA_OUT();	//sda�����
	C02_IIC_SDA=1;	  	  
	C02_IIC_SCL=1;
	delay_us(4);
 	C02_IIC_SDA=0;	//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	C02_IIC_SCL=0;	//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  

//����IICֹͣ�ź�
void C02_IIC_Stop(void)
{
	C02_SDA_OUT();	//sda�����
	C02_IIC_SCL=0;
	C02_IIC_SDA=0;	//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	C02_IIC_SCL=1; 
	C02_IIC_SDA=1;	//����I2C���߽����ź�
	delay_us(4);							   	
}

//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 C02_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	C02_SDA_IN();	//SDA����Ϊ����  
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
	C02_IIC_SCL=0;	//ʱ�����0 	   
	return 0;  
} 

//����ACKӦ��
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

//������ACKӦ��		    
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

//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void C02_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	C02_SDA_OUT(); 	    
    C02_IIC_SCL=0;	//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
		C02_IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);	//��TEA5767��������ʱ���Ǳ����
		C02_IIC_SCL=1;
		delay_us(2); 
		C02_IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    

//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 C02_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	C02_SDA_IN();	//SDA����Ϊ����
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
        C02_IIC_NAck();	//����nACK
    else
        C02_IIC_Ack();	//����ACK   
    return receive;
}

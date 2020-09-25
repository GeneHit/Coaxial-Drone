#include "myiic.h"
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
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA=1;
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;		//START:when CLK is high,DATA change form high to low
	delay_us(4);
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ��������
}
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1;
	IIC_SDA=1;//����I2C���߽����ź�
	delay_us(4);
}

//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	uint16_t ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����
	IIC_SDA=1;delay_us(1);
	IIC_SCL=1;delay_us(1);
	ucErrTime = 0;
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//ʱ�����0
	return 0;
}

//����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
//������ACKӦ��
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��
void IIC_Send_Byte(u8 txd)
{
    u8 t;
	  SDA_OUT();
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1;
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL=1;
		delay_us(2);
		IIC_SCL=0;	
		delay_us(2);
    }
}
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK
u8 IIC_Read_Byte(void)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0;
        delay_us(2);
		    IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;
		    delay_us(1);
    }
//    if (!ack)
//        IIC_NAck();//����nACK
//    else
//        IIC_Ack(); //����ACK
    return receive;
}

//��ʼ��IICʹ�õ���IO��
void IIC_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE );
//	AFIO->MAPR &= 0xf8ffffff;
//	AFIO->MAPR |= 0x02000000;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	IIC_SCL=1;
	IIC_SDA=1;
}

//addr������slave_address
//reg ����������Ҫд�����ݵ��׵�ַ
//len ��д�����ݵĳ���
//data����Ҫд���һ������
u8 IIC_Write_Buffer(u8 addr, u8 reg, u8 len, u8 * data)
{
    int i;
    IIC_Start();
    IIC_Send_Byte(addr << 1 | I2C_Direction_Transmitter);//7λ�����ӵ�ַ+��дλ
    if (IIC_Wait_Ack())
		{
        IIC_Stop();
        return false;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    for (i = 0; i < len; i++)
		{
        IIC_Send_Byte(*data);
        if (IIC_Wait_Ack())
		{
            IIC_Stop();
            return false;
        }
		data++;
    }
    IIC_Stop();
    return true;
}

//����ֵ 0��д�ɹ�
//		-1��дʧ��
int IIC_Write(u8 addr, u8 reg, u8 len, u8* data)
{
	if(IIC_Write_Buffer(addr,reg,len,data))
		return TRUE;
	else
		return FALSE;
}

//addr������slave_address
//reg ����������Ҫ�������ݵ��׵�ַ
//len ���������ݵĳ���
//buf ����Ҫ���������ݴ洢λ��
u8 IIC_Read_Buffer(u8 addr, u8 reg, u8 len, u8* buf)
{
    IIC_Start();
    IIC_Send_Byte(addr << 1 | I2C_Direction_Transmitter);
    if (IIC_Wait_Ack())
	{
        IIC_Stop();
        return false;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();

    IIC_Start();
    IIC_Send_Byte(addr << 1 | I2C_Direction_Receiver);
    IIC_Wait_Ack();
    while (len)
	{
        *buf = IIC_Read_Byte();
        if (len == 1)
            IIC_NAck();
        else
            IIC_Ack();
        buf++;
        len--;
    }
    IIC_Stop();
    return true;
}

//����ֵ 0�����ɹ�
//		-1����ʧ��
int IIC_Read(u8 addr, u8 reg, u8 len, u8 *buf)
{
	if(IIC_Read_Buffer(addr,reg,len,buf))
		return TRUE;
	else
		return FALSE;
}

//addr������slave_address
//reg ����������Ҫд�����ݵĵ�ַ
//data����Ҫд���һ������
u8 IIC_WriteOneByte(u8 addr, u8 reg, u8 data)
{
    IIC_Start();
    IIC_Send_Byte(addr << 1 | I2C_Direction_Transmitter);
    if (IIC_Wait_Ack())
	{
        IIC_Stop();
        return false;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Send_Byte(data);
    IIC_Wait_Ack();
    IIC_Stop();
    return true;
}

u16 IIC_GetErrorCounter(void)
{
    return 0;
}


//����������������HMC5883ʹ��
/*I2C��ָ���豸ָ����ַд��u8����-----------------------*/
void Single_WriteI2C(unsigned char REG_Address,unsigned char REG_data)//���ֽ�д��
{
	IIC_Start();
    IIC_Send_Byte(0x3C);   //�����豸��ַ+д�ź�//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//���ø���ʼ��ַ+������ַ
    IIC_Wait_Ack();
    IIC_Send_Byte(REG_Address );   //���õ���ʼ��ַ
    IIC_Wait_Ack();
    IIC_Send_Byte(REG_data);
    IIC_Wait_Ack();
    IIC_Stop();
	delay_us(1000);delay_us(1000);delay_us(1000);delay_us(1000);delay_us(1000);
}

/*I2C��ָ���豸ָ����ַ����u8����-----------------------*/
unsigned char Single_ReadI2C(unsigned char REG_Address)//��ȡ���ֽ�
{
	unsigned char REG_data;
	IIC_Start();
	IIC_Send_Byte(0x3C); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//���ø���ʼ��ַ+������ַ
	IIC_Wait_Ack();
	IIC_Send_Byte((u8) REG_Address);   //���õ���ʼ��ַ
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(0x3C+1);
	IIC_Wait_Ack();

	REG_data= IIC_Read_Byte();
	IIC_NAck();
	IIC_Stop();
	return REG_data;
}

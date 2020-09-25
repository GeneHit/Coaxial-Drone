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
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;		//START:when CLK is high,DATA change form high to low
	delay_us(4);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据
}
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1;
	IIC_SDA=1;//发送I2C总线结束信号
	delay_us(4);
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	uint16_t ucErrTime=0;
	SDA_IN();      //SDA设置为输入
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
	IIC_SCL=0;//时钟输出0
	return 0;
}

//产生ACK应答
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
//不产生ACK应答
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
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
void IIC_Send_Byte(u8 txd)
{
    u8 t;
	  SDA_OUT();
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1;
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		delay_us(2);
		IIC_SCL=0;	
		delay_us(2);
    }
}
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
u8 IIC_Read_Byte(void)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
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
//        IIC_NAck();//发送nACK
//    else
//        IIC_Ack(); //发送ACK
    return receive;
}

//初始化IIC使用到的IO口
void IIC_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE );
//	AFIO->MAPR &= 0xf8ffffff;
//	AFIO->MAPR |= 0x02000000;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	IIC_SCL=1;
	IIC_SDA=1;
}

//addr：器件slave_address
//reg ：从器件将要写入数据的首地址
//len ：写入数据的长度
//data：将要写入的一串数据
u8 IIC_Write_Buffer(u8 addr, u8 reg, u8 len, u8 * data)
{
    int i;
    IIC_Start();
    IIC_Send_Byte(addr << 1 | I2C_Direction_Transmitter);//7位器件从地址+读写位
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

//返回值 0：写成功
//		-1：写失败
int IIC_Write(u8 addr, u8 reg, u8 len, u8* data)
{
	if(IIC_Write_Buffer(addr,reg,len,data))
		return TRUE;
	else
		return FALSE;
}

//addr：器件slave_address
//reg ：从器件将要读的数据的首地址
//len ：读出数据的长度
//buf ：将要读出的数据存储位置
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

//返回值 0：读成功
//		-1：读失败
int IIC_Read(u8 addr, u8 reg, u8 len, u8 *buf)
{
	if(IIC_Read_Buffer(addr,reg,len,buf))
		return TRUE;
	else
		return FALSE;
}

//addr：器件slave_address
//reg ：从器件将要写入数据的地址
//data：将要写入的一个数据
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


//以下两个函数仅限HMC5883使用
/*I2C向指定设备指定地址写入u8数据-----------------------*/
void Single_WriteI2C(unsigned char REG_Address,unsigned char REG_data)//单字节写入
{
	IIC_Start();
    IIC_Send_Byte(0x3C);   //发送设备地址+写信号//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//设置高起始地址+器件地址
    IIC_Wait_Ack();
    IIC_Send_Byte(REG_Address );   //设置低起始地址
    IIC_Wait_Ack();
    IIC_Send_Byte(REG_data);
    IIC_Wait_Ack();
    IIC_Stop();
	delay_us(1000);delay_us(1000);delay_us(1000);delay_us(1000);delay_us(1000);
}

/*I2C向指定设备指定地址读出u8数据-----------------------*/
unsigned char Single_ReadI2C(unsigned char REG_Address)//读取单字节
{
	unsigned char REG_data;
	IIC_Start();
	IIC_Send_Byte(0x3C); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址
	IIC_Wait_Ack();
	IIC_Send_Byte((u8) REG_Address);   //设置低起始地址
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(0x3C+1);
	IIC_Wait_Ack();

	REG_data= IIC_Read_Byte();
	IIC_NAck();
	IIC_Stop();
	return REG_data;
}

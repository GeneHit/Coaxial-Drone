#include "AT24cxx.h"
#include "delay.h"

u8 AT24CXX_ReadOneByte(u16 ReadAddr)
{
	u8 temp=0;
  C02_IIC_Start();
	if(EE_TYPE>AT24C16)
	{
		C02_IIC_Send_Byte(0XA0);	//����д����
		C02_IIC_Wait_Ack();
		C02_IIC_Send_Byte(ReadAddr>>8);	//���͸ߵ�ַ
	}
	else
	{
		C02_IIC_Send_Byte(0XA0+((ReadAddr/256)<<1));	//����������ַ0XA0,д����
	}
	C02_IIC_Wait_Ack();
  C02_IIC_Send_Byte(ReadAddr%256);	//���͵͵�ַ
	C02_IIC_Wait_Ack();
	C02_IIC_Start();
	C02_IIC_Send_Byte(0XA1);	//�������ģʽ
	C02_IIC_Wait_Ack();
  temp=C02_IIC_Read_Byte(0);
  C02_IIC_Stop();	//����һ��ֹͣ����
	return temp;
}

//��AT24CXXָ����ַд��һ������
//WriteAddr  :д�����ݵ�Ŀ�ĵ�ַ
//DataToWrite:Ҫд�������
void AT24CXX_WriteOneByte(u16 WriteAddr,u8 DataToWrite)
{
	C02_IIC_Start();
	if(EE_TYPE>AT24C16)
	{
		C02_IIC_Send_Byte(0XA0);	//����д����
		C02_IIC_Wait_Ack();
		C02_IIC_Send_Byte(WriteAddr>>8);	//���͸ߵ�ַ
	}
	else
	{
	C02_IIC_Send_Byte(0XA0+((WriteAddr/256)<<1));	//����������ַ0XA0,д����
	}
	C02_IIC_Wait_Ack();
  C02_IIC_Send_Byte(WriteAddr%256);	//���͵͵�ַ
	C02_IIC_Wait_Ack();
	C02_IIC_Send_Byte(DataToWrite);	//�����ֽ�
	C02_IIC_Wait_Ack();
  C02_IIC_Stop();//����һ��ֹͣ����
	delay_ms(10);
}

//��AT24CXX�����ָ����ַ��ʼд�볤��ΪLen������
//�ú�������д��16bit����32bit������.
//WriteAddr  :��ʼд��ĵ�ַ
//DataToWrite:���������׵�ַ
//Len        :Ҫд�����ݵĳ���2,4
void AT24CXX_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len)
{
	u8 t;
	for(t=0;t<Len;t++)
	{
		AT24CXX_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}
}

//��AT24CXX�����ָ����ַ��ʼ��������ΪLen������
//�ú������ڶ���16bit����32bit������.
//ReadAddr   :��ʼ�����ĵ�ַ
//����ֵ     :����
//Len        :Ҫ�������ݵĳ���2,4
u32 AT24CXX_ReadLenByte(u16 ReadAddr,u8 Len)
{
	u8 t;
	u32 temp=0;
	for(t=0;t<Len;t++)
	{
		temp<<=8;
		temp+=AT24CXX_ReadOneByte(ReadAddr+Len-t-1);
	}
	return temp;
}

//���AT24CXX�Ƿ�����
//��������24XX�����һ����ַ(255)���洢��־��.
//���������24Cϵ��,�����ַҪ�޸�
//����1:���ʧ��
//����0:���ɹ�
u8 AT24CXX_Check(void)
{
	u8 temp;
	temp=AT24CXX_ReadOneByte(255);	//����ÿ�ο�����дAT24CXX
	if(temp==0X55)
	{
		return 0;
	}
	else	//�ų���һ�γ�ʼ�������
	{
		AT24CXX_WriteOneByte(255,0X55);
	  temp=AT24CXX_ReadOneByte(255);
		if(temp==0X55)
		{
			return 0;
		}
	}
	return 1;
}

//��AT24CXX�����ָ����ַ��ʼ����ָ������������
//ReadAddr :��ʼ�����ĵ�ַ ��24c02Ϊ0~255
//pBuffer  :���������׵�ַ
//NumToRead:Ҫ�������ݵĸ���
void AT24CXX_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=AT24CXX_ReadOneByte(ReadAddr++);
		NumToRead--;
	}
}

void AT24CXX_Read_Float(u16 WriteAddr,float *Data)
{
	u8 Buffer[3];
	AT24CXX_Read(WriteAddr, Buffer, 3);
	*Data = (float)Buffer[1]+ 0.01 * (float)Buffer[2];
	if(Buffer[0]==0x00)
	{
		*Data = -(*Data);
	}
}

//��AT24CXX�����ָ����ַ��ʼд��ָ������������
//WriteAddr :��ʼд��ĵ�ַ ��24c02Ϊ0~255
//pBuffer   :���������׵�ַ
//NumToWrite:Ҫд�����ݵĸ���
void AT24CXX_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite)
{
	while(NumToWrite--)
	{
		AT24CXX_WriteOneByte(WriteAddr,*pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}

void AT24CXX_Write_Float(u16 WriteAddr,float Data)
{
	u8 Buffer[3]={0,0,0};
	u8 num,point;
	if(Data>=0)
	{
		Buffer[0]=0xff;
	}
	else
	{
		Buffer[0]=0x00;
	}
	Data = fabs(Data);
	num = Data;
	point = (Data * 100 - num * 100);	//��λС��
	Buffer[1] = num;
	Buffer[2] = point;
	AT24CXX_Write(WriteAddr, Buffer, 3);
}
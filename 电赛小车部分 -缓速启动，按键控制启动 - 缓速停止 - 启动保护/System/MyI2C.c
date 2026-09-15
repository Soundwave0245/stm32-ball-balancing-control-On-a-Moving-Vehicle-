#include "stm32f10x.h"                  // Device header
#include "Delay.h"

#define My12C_APBPeriph RCC_APB2Periph_GPIOB   //软件模拟I2C用的外设，这个在开启时钟的时候要修改
#define MyI2C_Port GPIOB                       //端口GPIOB
#define MyI2C_SCL_Pin GPIO_Pin_6               //软件模拟I2C的SCL接口
#define MyI2C_SDA_Pin GPIO_Pin_7               //软件模拟I2C的SDA接口


void MyI2C_W_SCL(uint8_t BitValue)//I2C写SCL引脚电平，BitValue给1为高电平，给0为低电平
{
	GPIO_WriteBit(MyI2C_Port,MyI2C_SCL_Pin,(BitAction)BitValue);
	Delay_us(10);
}

void MyI2C_W_SDA(uint8_t BitValue)//I2C写SDA引脚电平，BitValue给1为高电平，给0为低电平
{
	GPIO_WriteBit(MyI2C_Port,MyI2C_SDA_Pin,(BitAction)BitValue);
	Delay_us(10);
}

uint8_t MyI2C_R_SDA(void)//I2C读SDA电平
{
	uint8_t SDA_Value;
	SDA_Value = GPIO_ReadInputDataBit(MyI2C_Port,MyI2C_SDA_Pin);
	Delay_us(10);
	return SDA_Value;
}

void MyI2C_Init(void)//I2C初始化
{
	RCC_APB2PeriphClockCmd(My12C_APBPeriph,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;          //开漏输出
	GPIO_InitStructure.GPIO_Pin = MyI2C_SCL_Pin | MyI2C_SDA_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MyI2C_Port,&GPIO_InitStructure);
	
	GPIO_SetBits(MyI2C_Port,MyI2C_SCL_Pin | MyI2C_SDA_Pin);
}

void MyI2C_Start(void)//I2C起始
{
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(0);
}

void MyI2C_Stop(void)//I2C终止
{
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(1);
}

void MyI2C_SendByte(uint8_t Byte)//I2C发送一个字节，Byte是那个字节的数据，范围是0x00-0xFF
{
	uint8_t i;
	for(i=0 ;i<8; i++)
	{
		//使用两次！进行逻辑取反，作用是把所有非0值统一转换为1
		MyI2C_W_SDA(!!(Byte&(0x80>>i)));//掩码读取Byte的指定一位数据并写入SDA线
		MyI2C_W_SCL(1);                
		MyI2C_W_SCL(0);					
	}
}

uint8_t MyI2C_ReceiveByte(void)//I2C接收一个字节，返回这个接收到的字节数据，范围0x00-0xFF
{
	uint8_t i,Byte = 0x00;
	MyI2C_W_SDA(1);
	for(i=0 ;i<8; i++)
	{
		MyI2C_W_SCL(0);
		MyI2C_W_SCL(1);
		if(MyI2C_R_SDA())
		{
			Byte|=0x80>>i;
		}
	    MyI2C_W_SCL(0);
	}
	return Byte;
}


void MyI2C_SendAck(uint8_t AckBit)//发送应答位，AckBit=0表示应答
{
	MyI2C_W_SDA(AckBit);
	MyI2C_W_SCL(1);
	MyI2C_W_SCL(0);
}

uint8_t MyI2C_ReceiveAck(void)//I2C接收应答位，AckBit=0表示应答，返回AckBit
{
	uint8_t AckBit;
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);
	AckBit = MyI2C_R_SDA();
	MyI2C_W_SCL(0);
	return AckBit;
}




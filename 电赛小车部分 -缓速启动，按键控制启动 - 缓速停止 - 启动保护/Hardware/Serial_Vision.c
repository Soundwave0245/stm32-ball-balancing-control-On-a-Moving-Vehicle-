#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>

#define TxPacket_Length 4       //串口发送数据包长度
#define RxPacket_Length 4       //串口接收数据包长度，修改这里同时状态机的接收长度也会修改

uint8_t Serial_Vision_TxPacket[TxPacket_Length];
uint8_t Serial_Vision_RxPacket[RxPacket_Length];
uint8_t Serial_Vision_RxFlag;

void Serial_Vision_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;                 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;             //Tx :Send out the signal
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b ;
	USART_Init(USART1,&USART_InitStructure);
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART1,ENABLE);
}



void Serial_Vision_SendByte(uint16_t Data)
{
	USART_SendData(USART1,Data);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
}

void Serial_Vision_SendArray(uint8_t *array, uint16_t Length)
{
	uint16_t i;
	for(i=0; i<Length; i++)
	{
		Serial_Vision_SendByte(array[i]);
	}
	
}
	
void Serial_Vision_SendString(char* String)
{
	uint16_t i;
	for(i=0;String[i] != '\0';i++)
	{
		Serial_Vision_SendByte(String[i]);
	}
	
}

uint32_t Serial_Vision_Pow(uint16_t X,uint16_t Y)
{
	uint32_t Result = 1;
	while(Y--) 
	{
		Result*=X;
	}
	
	return Result;
}

void Serial_Vision_SendNum(uint32_t Number,uint16_t Length)
{
	uint16_t i;
	for(i=0; i<Length; i++)
	{
		Serial_Vision_SendByte(Number/Serial_Vision_Pow(10,Length-i-1)%10 + 0x30);
	}
}

uint8_t Serial_Vision_GetRxFlag(void)
{
	if(Serial_Vision_RxFlag == 1)
	{
		Serial_Vision_RxFlag = 0;
		return 1;
	}
	else
		return 0;
}

void Serial_Vision_SendPacket(void)
{
	Serial_Vision_SendByte(0xFF);
	Serial_Vision_SendArray(Serial_Vision_TxPacket,TxPacket_Length);
	Serial_Vision_SendByte(0xFE);
}


void Serial_Vision_Printf(char *format,...)
{
	char String[100];
	va_list arg;
	va_start(arg,format);
	vsprintf(String,format,arg);
	va_end(arg);
	Serial_Vision_SendString(String);
	
}

void USART1_IRQHandler(void)                                             //状态机接收数据，0xFF ?? ?? ?? ?? 0xFE
{
	if (USART_GetITStatus(USART1,USART_IT_RXNE)==SET)
	{
		static uint8_t RxState = 0; 
		static uint8_t pRxPacket;
		
		if(RxState == 0)
		{
			if(USART_ReceiveData(USART1) == 0xFF)
			{
				RxState = 1;
				pRxPacket = 0;
			}
		}
		else if(RxState == 1)
		{
			Serial_Vision_RxPacket[pRxPacket] = USART_ReceiveData(USART1);
			pRxPacket++ ;
			if(pRxPacket >= RxPacket_Length)
			{
				RxState = 2;
			}
		}
		else if(RxState == 2)
		{
			if(USART_ReceiveData(USART1) == 0xFE)
			{
				RxState = 0;
				Serial_Vision_RxFlag = 1;
			}
		}
		
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	}
}



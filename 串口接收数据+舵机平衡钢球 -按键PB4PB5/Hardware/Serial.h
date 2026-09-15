#ifndef __SERIAL_H
#define __SERIAL_H
#include <stdio.h>

extern uint8_t Serial_TxPacket[];
extern uint8_t Serial_RxPacket[];

void Serial_Init(void);
void Serial_SendByte(uint16_t Data);
void Serial_SendArray(uint8_t *array, uint16_t Length);
void Serial_SendString(char* String);
void Serial_SendNum(uint32_t Number,uint16_t Length);
void Serial_Printf(char *format,...);
uint8_t Serial_GetRxFlag(void);
void Serial_SendPacket(void);

#endif


#ifndef __SERIAL_VISION_H
#define __SERIAL_VISION_H
#include <stdio.h>

extern uint8_t Serial_Vision_TxPacket[];
extern uint8_t Serial_Vision_RxPacket[];

void Serial_Vision_Init(void);
void Serial_Vision_SendByte(uint16_t Data);
void Serial_Vision_SendArray(uint8_t *array, uint16_t Length);
void Serial_Vision_SendString(char* String);
void Serial_Vision_SendNum(uint32_t Number,uint16_t Length);
void Serial_Vision_Printf(char *format,...);
uint8_t Serial_Vision_GetRxFlag(void);
void Serial_Vision_SendPacket(void);

#endif


#ifndef __TRACKER_H
#define __TRACKER_H

#define STOP 0;
#define RUN 1;
#define LEFT 2;
#define RIGHT 3;

extern uint16_t Num_of_Cross;

typedef struct{
	uint16_t TrackMode;
	uint16_t TrackData;      //传感器读出来的数据（按位10）
	uint16_t TrackLastData;  //上一次检测到的数据，用于应对直角和锐角向对应方向转弯
	int16_t TrackCurrentDev; //当前偏航
}TrackerStruct;

void Tracker_Init(void);
void Tracker_ReadData(TrackerStruct *Tracker);
void Signal_Handler(TrackerStruct *Tracker);
int16_t Dev_SpeedPIDUpdate(int16_t Empower);


#endif


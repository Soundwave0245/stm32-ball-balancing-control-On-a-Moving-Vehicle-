#ifndef __PID_H
#define __PID_H

typedef struct{

	int16_t Target;
	int16_t Actual;
	int16_t Out;
	
	float Kp;
	float Ki;
	float Kd;
	
	float Error0;
	float Error1;
	float ErrorInt;
	
	int16_t OutMax;
	int16_t OutMin;
}PID_t;

void PID_Update(PID_t *p);


#endif


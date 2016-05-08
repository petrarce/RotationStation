#ifndef MYTYPES_H
#define MYTYPES_H
#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

struct driverSettings
{
	uint16_t tooglePin;
	uint16_t directionPin;
	uint16_t powerPin;
	GPIO_TypeDef* Port;
	float VCur;
	float Vmin;
	float Accel;
	float Vmax;
	uint32_t 	tooglePeriod;
	float PCur;
	float Pgoal;
	float AngelPerStep;
	uint8_t DIR;
	uint8_t flagStart;
	uint8_t sleepMode;
	xTaskHandle xToogleBits;
	xTaskHandle xAccelStart;
	xTaskHandle xAccelStop;
	xTaskHandle xStopWithReturn;
	xTaskHandle xStopWithRun;	
	xSemaphoreHandle mutexPCur;
};
struct MainShedulerstruct
{
	const struct driverSettings* upDriver;
	const struct driverSettings* dounDriver;
	char* 				 							 Buffer;
};
typedef struct MainShedulerstruct sMainSheduler;
typedef struct driverSettings sDriverSettings;

#endif
#include "rotationTasks.h"
#include "alldefines.h"
#include "mytypes.h"
#include "Functions.h"
#include "stm32f4_discovery.h"

#define TOOGLE_BITS_ON 1
#define TOOGLE_BITS_OFF 0
#define turnOnToogleBits() \
	{ vTaskResume(currDriver->xToogleBits); toogleBitsFlag=TOOGLE_BITS_ON;}
#define turnOffToogleBits() \
	{ vTaskSuspend(currDriver->xToogleBits); toogleBitsFlag=TOOGLE_BITS_OFF;}

uint8_t toogleBitsFlag=0;

void vAccelStart(void* pvParameters)
{
	struct driverSettings* currDriver=(struct driverSettings*) pvParameters;
	while(1)
	{
		if(!toogleBitsFlag)
			turnOnToogleBits()
		if(currDriver->VCur<=currDriver->Vmax)
		{
			
			//-------------------------------------
			//-----------MAIN CODE-----------------
			currDriver->VCur=Start(currDriver->VCur,dt,currDriver->Accel,currDriver->Vmax);
			currDriver->tooglePeriod=SEC/(currDriver->VCur*DEG_SEC*2);
			vTaskDelay(dt*SEC);			
			//-------------------------------------
			
			//-----------TEST CODE-----------------
			//-------------------------------------
		}
		else
		{
			currDriver->VCur=currDriver->Vmax;
			vTaskSuspend(NULL);
		}
	}
}

void vAccelStop(void* pvParameters)
{
	struct driverSettings* currDriver=(struct driverSettings*) pvParameters;
	while(1)
	{
		if(currDriver->VCur>1) // >1 degree !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 0.5 WTF????
		{
			//xSemaphoreTake(currDriver->mutexPCur,PCUR_MUTEX_DELAY);
			currDriver->VCur=Stop(currDriver->VCur, dt, currDriver->Accel,currDriver->DIR);
			currDriver->tooglePeriod=SEC/(currDriver->VCur*DEG_SEC*2);
			//xSemaphoreGive(currDriver->mutexPCur);

			vTaskDelay(dt*SEC);
		}
		else
		{
			currDriver->VCur=0;
			turnOffToogleBits();
			if(currDriver->sleepMode)
				GPIO_ResetBits(currDriver->Port,currDriver->powerPin); 
			vTaskSuspend(NULL);
		}			
	}		
}

void vStopWithRun(void* pvParameters)
{
	//uint8_t toogleBitsFlag=0;
	struct driverSettings* currDriver=(struct driverSettings*) pvParameters;
	portTickType xLastWakeTime;

	while(1)
	{
		xLastWakeTime = xTaskGetTickCount();
		if(!toogleBitsFlag)
		{
			turnOnToogleBits()
		}
		if (currDriver->VCur > 0||(!currDriver->flagStart))
		{
			currDriver->flagStart=1;
			//xSemaphoreTake(currDriver->mutexPCur,PCUR_MUTEX_DELAY);
			currDriver->VCur = StopAtAngle(currDriver->VCur, 
																			dt, 
																			currDriver->Accel, 
																			currDriver->PCur,
																			currDriver->Pgoal, 
																			0, 
																			currDriver->Vmax, 
																			currDriver->Vmin,
																			currDriver->DIR); 
			currDriver->tooglePeriod=SEC/(currDriver->VCur*DEG_SEC*2);
			//xSemaphoreGive(currDriver->mutexPCur);
			vTaskDelayUntil(&xLastWakeTime,dt*SEC);
		}
		else																				
		{
			currDriver->flagStart=0;
			currDriver->VCur = 0;
			turnOffToogleBits();
			vTaskSuspend(NULL);	 
		}		
	}			
}

/*void vStopWithReturn(void* pvParameters)
{
		struct driverSettings* currDriver=(struct driverSettings*) pvParameters;
		vTaskResume(currDriver->xToogleBits);
		while(1)
		{
			if (FlagStopAndReturn == 0)
			{
				currDriver->VCur = Stop(currDriver->VCur, dt, currDriver->Accel,currDriver->DIR);
				vTaskDelay(dt*SEC);
				if (currDriver->VCur == 0)
				{
					FlagStopAndReturn = 1;
					if(GetMinPath(Compass_Term, currDriver->PCur,1)>GetMinPath(Compass_Term, currDriver->PCur,0))
					{
						GPIO_ResetBits(currDriver->Port,currDriver->directionPin);
						GPIO_ResetBits(currDriver->Port,currDriver->directionPin);
						currDriver->DIR=0;
					}
					else
					{
						GPIO_SetBits(currDriver->Port,currDriver->directionPin);
						GPIO_SetBits(currDriver->Port,currDriver->directionPin);
						currDriver->DIR=1;
					}
				}
				continue;
			}
			else
			{
				currDriver->VCur = StopAtAngle(currDriver->VCur, 
																				dt, 
																				currDriver->Accel, 
																				currDriver->PCur,
																				Compass_Term, 
																				0, 
																				currDriver->Vmax, 
																				currDriver->Vmax,
																				currDriver->DIR);
				vTaskDelay(dt*SEC);
			}	
			if(currDriver->VCur==0 && FlagStopAndReturn == 1) 
			{
				FlagStopAndReturn=0;
				vTaskSuspend(currDriver->xToogleBits);
				vTaskSuspend(NULL);
			}
		}						
}*/


uint8_t HoleOn=0;
uint8_t HolaCount=0;
float delta;
uint32_t someVar=0;
uint32_t Steps=0;

void vToogleBits(void* pvParameters)
{
	
	struct driverSettings* currDriver=(struct driverSettings*) pvParameters;
	float _CurSpeed=0;
	float delay;

	while(1)
	{
		_CurSpeed=currDriver->VCur;
		//delay=currDriver->tooglePeriod;
		if(_CurSpeed>0)//(delay > SEC/MAX_DRIVER_FREQ)   
		{
			//-------------------------------------
			//-----------MAIN CODE-----------------
			delay=SEC/(_CurSpeed*DEG_SEC*2);
			GPIO_SetBits(currDriver->Port,currDriver->tooglePin);
			vTaskDelay(delay);
			GPIO_ResetBits(currDriver->Port,currDriver->tooglePin);
			vTaskDelay(delay);//(SEC/_CurSpeed*DEG_SEC*2);//
			currDriver->PCur=(GPIO_ReadOutputDataBit(currDriver->Port,currDriver->directionPin))	//not shure if 0 or 1 is right direction
												?GetAngle(currDriver->PCur+currDriver->AngelPerStep)
												:GetAngle(currDriver->PCur-currDriver->AngelPerStep);
			Steps+=2;
			//-------------------------------------
			//-----------TEST CODE-----------------
			/*GPIO_SetBits(GPIOB,GPIO_Pin_15);
			vTaskDelay(SEC/(_CurSpeed));
			GPIO_ResetBits(GPIOB,GPIO_Pin_15);
			vTaskDelay(SEC/(_CurSpeed));*/
		}
			//-----------------------------------
	}
}
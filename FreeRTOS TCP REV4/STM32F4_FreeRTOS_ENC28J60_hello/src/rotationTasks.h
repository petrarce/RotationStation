#ifndef ROTATION_TASKS_H
#define ROTATION_TASKS_H

void vAccelStart(void* pvParameters);

void vAccelStop(void* pvParameters);

void vStopWithRun(void* pvParameters);
	
void vToogleBits(void* pvParameters);

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



#endif
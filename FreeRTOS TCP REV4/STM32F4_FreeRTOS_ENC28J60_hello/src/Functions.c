#include "Functions.h"
#include "math.h"


//#include "Constants.h"
#define STOP_ANGLE_ERROR 0.3
#define START_ENCREASING_SPEED_VALUE 10
#define ERR fabs(GetMinPath(GivPhase,CUR_ANG,DIR)-StopPhase)
//-----------------------------
//FUNCTIONS
//-----------------------------
float Start(float V_CUR, float dt, float ACC, float Vmax)
{
	if(V_CUR<Vmax)
	{
		V_CUR=GetV_CUR(V_CUR,dt,ACC);//get current V_CUR
	}
	else 
		V_CUR=Vmax;
	return V_CUR;
}

float Stop(float V_CUR, float dt, float ACC,int DIR)
{
	//*CUR_ANG=GetPhase(*CUR_ANG,dt,V_CUR,DIR);
	V_CUR = GetV_CUR(V_CUR, dt, -ACC);
	if (V_CUR > 0)
		return V_CUR;
	else 
		return 0;
}

float StopAtAngle(float V_CUR, float dt, float ACC,float CUR_ANG,float GivPhase,float Vmin,float Vmax,float Vmax_STOP,int DIR)
{
	float StopPhase=pow(V_CUR,2)/(ACC*2);;
	float Error;
	if(ERR>START_ENCREASING_SPEED_VALUE)
		return Start(V_CUR,dt,ACC,Vmax);		 
	if(V_CUR>Vmax_STOP)
	{
		//*CUR_ANG=GetPhase(*CUR_ANG,dt,V_CUR,DIR);
		return GetV_CUR(V_CUR,dt,-ACC);
	}
	else {
		//StopPhase=pow(V_CUR,2)/(ACC*2);				//calc stop path
		Error=fabs(GetMinPath(GivPhase,CUR_ANG,DIR)-StopPhase);
		if(ERR>STOP_ANGLE_ERROR||ERR<-STOP_ANGLE_ERROR)//if(Error>1||Error<-1)  //was >1 otrabotka s min degree
		{
			//*CUR_ANG=GetPhase(*CUR_ANG,dt,V_CUR,DIR);
			return Start(V_CUR,dt,ACC,Vmax_STOP);		 
		}
		else
		{
			//*CUR_ANG=GetPhase(*CUR_ANG,dt,V_CUR,DIR);
			V_CUR=GetV_CUR(V_CUR,dt,-ACC);
		}
	}
	if (V_CUR < 1 ) // 1 per sec
		V_CUR = 0;
	return V_CUR;
}

float GetV_CUR(float V_CUR,float dt, float ACC)
{
	return V_CUR+dt*ACC;
}

float GetAngle(float CUR_ANG)
{
	if(CUR_ANG>360)
		return CUR_ANG-360;
	else 
		if(CUR_ANG<0)
			return 360+CUR_ANG;	
		else 
			return CUR_ANG;
			 
}


float GetPhase(float Pprev,float dt,float V_CUR,int DIR)
{
	float temp=(DIR==1)?Pprev+V_CUR*dt:Pprev-V_CUR*dt; //DIR=1 - clockwise
	return GetAngle(temp);
}



float GetMinPath(float TERM_ANG,float CUR_ANG,int DIR)
{
	float tempCUR_ANG=(DIR==1)?TERM_ANG-CUR_ANG:360-(TERM_ANG-CUR_ANG);
	return GetAngle(tempCUR_ANG);
}
float GetNewCUR_ANG(float CUR_ANG,float STOP_PATH,int DIR)
{
	float tempANG=(DIR==1)?CUR_ANG+STOP_PATH:CUR_ANG-STOP_PATH;
	return GetAngle(tempANG);
} 

float GetReturnPath(float TERM_ANG,float CUR_ANG,float STOP_PATH,int DIR)
{
	float NewPhase=GetNewCUR_ANG(CUR_ANG,STOP_PATH,DIR);
	float S1,S2;
	S1=GetMinPath(TERM_ANG,NewPhase,1);
	S2=GetMinPath(TERM_ANG,NewPhase,0);
	if(S1>S2)
		return S2;
	else 
		return S1;
}

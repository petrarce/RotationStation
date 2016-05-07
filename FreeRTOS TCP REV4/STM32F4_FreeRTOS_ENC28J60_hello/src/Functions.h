#ifndef Functions
#define Functions
//-----------------------------
//FUNCTIONS
//-----------------------------
	//
	//Acelerates engine from Vmin to Vmax with defined acceleration 
	//
	float Start(float , float, float , float );
	//
	//Stops engine from V_CUR to 0
	//
	float Stop(float, float, float,int);

	//
	//stops engine from current V  to 0
	//	if V>Vmax_STOP then firstfully it slows down the engine winh defined acceleration 
	//	to Vmax_STOP speed, and than it slows down the engine to a 0
	//
	float StopAtAngle(float , float , float ,float,float,float,float, float,int);
	//
	//Finds new speed according to a parameters 
	//
	float GetV_CUR(float ,float , float );
	//
	//Finds new Phase according to a speed, delay, direction and current phase 
	//
	float GetPhase(float ,float ,float ,int );
	//
	//Finds new Frequency according to a parameters 
	//
	int GetFrequency(float);
	//
	//Find path between twoangles according to direction
	//
	float GetMinPath(float TERM_ANG,float CUR_ANG,int DIR);
	//
	//
	//
	float GetReturnPath(float,float,float,int );
	//
	//
	//
	float GetAngle(float CUR_ANG);

#endif // !Functions


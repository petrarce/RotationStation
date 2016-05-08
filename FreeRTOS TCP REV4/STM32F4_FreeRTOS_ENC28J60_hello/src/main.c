
//---------------------------- my includes
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_tim.h"
#include <stdlib.h>
#include "alldefines.h"
#include "math.h"
#include <stdio.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_i2c.h>
#include <string.h>
//--------------------------- end of my includes
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "stm32f4_discovery.h"
//#include "utils.h"
//#include "mems.h"
//****************************************************************
#include "hello-world.h"
#include "uip.h"
#include "uip_arp.h"
#include "enc28j60.h"
#include "Functions.h"
#include "mytypes.h"
#include "rotationTasks.h"
//****************************************************************

//--------------------------------------------------------------
#define BUF ((struct uip_eth_hdr *)&uip_buf[0])

//--------------------------------------------------------------

uint64_t u64Ticks=0;        // Counts OS ticks (default = 1000Hz).
uint64_t u64IdleTicks=0;    // Value of u64IdleTicksCnt is copied once per sec.
uint64_t u64IdleTicksCnt=0; // Counts when the OS has no task to execute.
uint16_t u16PWM1=0;
//============================================================================
xTaskHandle xHolla_ADC;
xTaskHandle xTaskSwapStartStop;
xTaskHandle xCompassAndInc;
xTaskHandle xHolaCounter;
xTaskHandle xMainSheduler;

uint8_t FlagStart=0;


sDriverSettings upDriver;
sDriverSettings dounDriver;
sMainSheduler 	mainShedulerObjects;


static __IO uint32_t TimingDelay;
void Delay_ms(__IO uint32_t nTime);

static float X_Inclinometer = 0;

static float Compass_Term =0; 
static float Incli_Term =0;        

static int Flag_Start=0;

uint8_t FlagStopAndReturn = 0;								//state if inb function stop with returndriver already stops but not returned yet

uint8_t Measuremode       		= 0x00; 				//Measure mode (normal operation mode after power on)
uint8_t ReadTemperature   		= 0x08; 				//Read temperature data register
uint8_t ActivateSelfTestX 		= 0x0E; 				//Activate Self test for X-channel
uint8_t ActivateSelfTestY 		= 0x0F; 				//Activate Self test for Y-channel
uint8_t ReadX 			  		= 0x10; 				//Read X-channel acceleration
uint8_t ReadY 			  		= 0x11; 				//Read Y-channel acceleration

uint16_t data0ch_adc =0;
uint16_t data1ch_adc =0; 
uint16_t data2ch_adc =0;
uint16_t data3ch_adc =0;

uint8_t cmdBuffer[10];

int ConvertedValue = 0; //Converted value readed from ADC

void adc_configure(){
 ADC_InitTypeDef ADC_init_structure; //Structure for adc confguration
 GPIO_InitTypeDef GPIO_initStructre; //Structure for analog input pin
 //Clock configuration
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);//The ADC1 is connected the APB2 peripheral bus thus we will use its clock source
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//Clock for the ADC port!! Do not forget about this one ;)
 //Analog pin configuration
 GPIO_initStructre.GPIO_Pin = GPIO_Pin_0;					//The channel 10 is connected to PC0
 GPIO_initStructre.GPIO_Mode = GPIO_Mode_AN; 			//The PC0 pin is configured in analog mode
 GPIO_initStructre.GPIO_PuPd = GPIO_PuPd_NOPULL; 	//We don't need any pull up or pull down
 GPIO_Init(GPIOA,&GPIO_initStructre);//Affecting the port with the initialization structure configuration
 //ADC structure configuration
 ADC_DeInit();
 ADC_init_structure.ADC_DataAlign = ADC_DataAlign_Right;//data converted will be shifted to right
 ADC_init_structure.ADC_Resolution = ADC_Resolution_12b;//Input voltage is converted into a 12bit number giving a maximum value of 4096
 ADC_init_structure.ADC_ContinuousConvMode = ENABLE; //the conversion is continuous, the input data is converted more than once
 ADC_init_structure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;// conversion is synchronous with TIM1 and CC1 (actually I'm not sure about this one :/)
 ADC_init_structure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//no trigger for conversion
 ADC_init_structure.ADC_NbrOfConversion = 1;//I think this one is clear :p
 ADC_init_structure.ADC_ScanConvMode = DISABLE;//The scan is configured in one channel
 ADC_Init(ADC1,&ADC_init_structure);//Initialize ADC with the previous configuration
 //Enable ADC conversion
 ADC_Cmd(ADC1,ENABLE);
 //Select the channel to be read from
 ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_144Cycles); //3 15 28 72
}

int adc_convert(){
 ADC_SoftwareStartConv(ADC1);//Start the conversion
 while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));//Processing the conversion
 return ADC_GetConversionValue(ADC1); //Return the converted data
}

uint8_t SPISend(uint8_t data)
{
	while (!(SPI1->SR & SPI_SR_TXE));
	SPI1->DR = data;
	while (!(SPI1->SR & SPI_SR_RXNE));
	return (SPI1->DR);
}

float ConvertDataToDegree(int data)
{
	return (asinf((data-1024.00f) /InclinometerSensitivity ) * (180.00f / M_PI));
}

void GetDataFromInclinometer()
{
	int16_t buffer[2];
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
	SPISend(0x10);
	buffer[0] = SPISend(0x00);
	buffer[1] = SPISend(0x00);
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
	X_Inclinometer = ConvertDataToDegree(((buffer[0] << 8) | buffer[1]) >> 5);
}


void float_to_string(void);

//------------------------------------------- PROTO SOCKET ------------------------------------------------------

/*
 * Declaration of the protosocket function that handles the connection
 * (defined at the end of the code).
 */
static int handle_connection(struct hello_world_state *s);
/*------------------------------------------------------------------------------------------------------------*/
/*
 * The initialization function. We must explicitly call this function
 * from the system initialization code, some time after uip_init() is
 * called.
 */
void
hello_world_init(void)
{
  /* We start to listen for connections on TCP port 1000. */
  uip_listen(HTONS(1000));
}
/*---------------------------------------------------------------------------*/
/*
 * In hello-world.h we have defined the UIP_APPCALL macro to
 * hello_world_appcall so that this funcion is uIP's application
 * function. This function is called whenever an uIP event occurs
 * (e.g. when a new connection is established, new data arrives, sent
 * data is acknowledged, data needs to be retransmitted, etc.).
 */
void hello_world_appcall(void)
{
  /*
   * The uip_conn structure has a field called "appstate" that holds
   * the application state of the connection. We make a pointer to
   * this to access it easier.
   */
  struct hello_world_state *s = &(uip_conn->appstate);

  /*
   * If a new connection was just established, we should initialize
   * the protosocket in our applications' state structure.
   */
  if(uip_connected()) {
    PSOCK_INIT(&s->p, s->inputbuffer, sizeof(s->inputbuffer));
  }

  /*
   * Finally, we run the protosocket function that actually handles
   * the communication. We pass it a pointer to the application state
   * of the current connection.
   */
  handle_connection(s);
}
/*---------------------------------------------------------------------------*/
/*
 * This is the protosocket function that handles the communication. A
 * protosocket function must always return an int, but must never
 * explicitly return - all return statements are hidden in the PSOCK
 * macros.
 */


static int handle_connection(struct hello_world_state *s)
{
  PSOCK_BEGIN(&s->p);
  PSOCK_SEND_STR(&s->p, "Inclinometer\n");
  PSOCK_READTO(&s->p, '\n');
  strncpy(s->name, s->inputbuffer, sizeof(s->name));
  strncpy(mainShedulerObjects.Buffer,s->name, sizeof(char)*10);
	vTaskResume(xMainSheduler);
	if (cmdBuffer[0] == 'Q')
	{
		PSOCK_CLOSE(&s->p);
	}
  PSOCK_SEND_STR(&s->p, s->name);
  PSOCK_END(&s->p);
}

/*----------------------------------nEND PROTO SOCKET ------------------------------------------------------*/

//void vLedBlinkOrange(void *pvParameters);
void vTask_uIP_periodic(void *pvParameters);
void vTask_uIP(void *pvParameters);


// This FreeRTOS callback function gets called once per tick (default = 1000Hz).
// ---------------------------------------------------------------------------- 
void vApplicationTickHook( void ) 
{
    ++u64Ticks;
}

// This FreeRTOS call-back function gets when no other task is ready to execute.
// On a completely unloaded system this is getting called at over 2.5MHz!
// ---------------------------------------------------------------------------- 
void vApplicationIdleHook( void ) {
    ++u64IdleTicksCnt;
}

// A required FreeRTOS function.
// ---------------------------------------------------------------------------- 
void vApplicationMallocFailedHook( void ) {
    configASSERT( 0 );  // Latch on any failure / error.
}



void vCompassAndInc(void* pvParameters)     // RECEIVE DATA FROM SENSORS AND  QUEUE FOR NEXT TASK TAHT SEND IT TO DOWN BOARD
{	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{  
		GetDataFromInclinometer();
		vTaskDelayUntil(&xLastWakeTime, INCLI_DEALY); //1000 20 samples per second AT 20000 TICK_RATE_HZ
	}
}


void vHolla_ADC(void* pvParameters)     // RECEIVE DATA FROM SENSORS AND  QUEUE FOR NEXT TASK TAHT SEND IT TO DOWN BOARD
{	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{  	
		ConvertedValue = adc_convert();  // otsuda popadaem v IRQ  
		vTaskDelayUntil(&xLastWakeTime, HOLA_DELAY); //100 samples per second
	}
}

void vTask_uIP_periodic(void *pvParameters) {
        uint32_t i;
        uint8_t delay_arp = 0;
	
        for (;;) {
                vTaskDelay(configTICK_RATE_HZ/2); 
                delay_arp++;
								
                for (i = 0; i < UIP_CONNS; i++) {
                        uip_periodic(i);
                        if (uip_len > 0) {
                                uip_arp_out();
                                enc28j60_send_packet((uint8_t *) uip_buf, uip_len);
                        }
                }

#if UIP_UDP
                for(i = 0; i < UIP_UDP_CONNS; i++) {
                        uip_udp_periodic(i);
                        if(uip_len > 0) {
                                uip_arp_out();
                                network_send();
                        }
                }
#endif /* UIP_UDP */

                if (delay_arp >= 50) { 
                        delay_arp = 0;
                        uip_arp_timer();
                }
        }
				
//				    STM_EVAL_LEDToggle(LED5);
				
}
//--------------------------------------------------------------
void vTask_uIP(void *pvParameters) {

	   // STM_EVAL_LEDToggle(LED3);
    //STM_EVAL_LEDToggle(LED6);
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
        {
                uip_len = enc28j60_recv_packet((uint8_t *) uip_buf, UIP_BUFSIZE);

                if (uip_len > 0) {
                        if (BUF->type == htons(UIP_ETHTYPE_IP)) {
                                uip_arp_ipin();
                                uip_input();
                                if (uip_len > 0) {
                                        uip_arp_out();
                                        enc28j60_send_packet((uint8_t *) uip_buf, uip_len);
                                }
                        } else if (BUF->type == htons(UIP_ETHTYPE_ARP)) {
                                uip_arp_arpin();
                                if (uip_len > 0) {
                                        enc28j60_send_packet((uint8_t *) uip_buf, uip_len);
                                }
                        }
                }
                taskYIELD();
								vTaskDelayUntil(&xLastWakeTime, UIP_DELAY);
        }
}

//--------------------------------------------------------------
//--------------------------------------------------------------
//******************************************************************************
/*-----------------------------------------------------------*/
void uip_log(char *msg) {

}
#define ASCII_9 58
#define ASCII_0 48

uint8_t myAtof(char* str,float* value)
{
	uint8_t i=0;
	uint32_t temp=0;
	uint8_t sign=1;
	while(str[i]!=NULL)
	{
		if(str[i]<=ASCII_9&&str[i]>=ASCII_0)
			temp=temp*10+str[i++]-ASCII_0;
		else
			return 0;
	}
	*value=temp;
	return 1;
}
void parceBuffer()
{
	uint8_t symbolCounter=0;
	int8_t sign=1;
	char tempBuffer[5];
	for(int i=1; i<10;i++)
	{
		switch(cmdBuffer[i])
		{
			case 13:
				tempBuffer[symbolCounter]=0;
				myAtof(&tempBuffer[0],&upDriver.Pgoal);
				upDriver.Pgoal*=sign;
				return;
			case '-':
				sign=-1;
				break;
			case 'H':
				tempBuffer[symbolCounter]=0;
				myAtof(&tempBuffer[0],&dounDriver.Pgoal);
				dounDriver.Pgoal*=sign;
				sign=1;
				symbolCounter=0;
				break;
			default:
				tempBuffer[symbolCounter++]=cmdBuffer[i];
				break;
		}	
	}
	return;
}

void setDirection(sDriverSettings* const currDriver)
{
			if(currDriver->Pgoal-currDriver->PCur>=0)
			{
				if(currDriver->Pgoal-currDriver->PCur<=180)
					currDriver->DIR=1;
				else
					currDriver->DIR=0;
			}
			else
			{
				if(currDriver->Pgoal-currDriver->PCur>=-180)
					currDriver->DIR=0;
				else
					currDriver->DIR=1;
			}		
			if(currDriver->DIR)
				GPIO_SetBits(currDriver->Port,currDriver->directionPin);
			else
				GPIO_ResetBits(currDriver->Port,currDriver->directionPin);
			return;

}
#define startDounDriver() \
			vTaskSuspend(dounDriver.xAccelStop); \
			vTaskSuspend(dounDriver.xStopWithRun); \
			vTaskResume(dounDriver.xAccelStart)
#define stopDounDriver() \
			vTaskSuspend(dounDriver.xAccelStart); \
			vTaskSuspend(dounDriver.xStopWithRun); \
			vTaskResume(dounDriver.xAccelStop)	

#define  standDounDriverOnAngle() \
			vTaskSuspend(dounDriver.xAccelStart);\
			vTaskSuspend(dounDriver.xAccelStop);\
			vTaskResume(dounDriver.xStopWithRun)		


#define standUpDriverOnAngle()\
			vTaskResume(upDriver.xStopWithRun)				
void turnTasks()
{
	
	switch(cmdBuffer[0])
	{
		case 'R':
		{
			vTaskSuspend(upDriver.xAccelStop);
			vTaskResume(upDriver.xAccelStart);
			break;
		}
		case 'S':
		{
			vTaskSuspend(upDriver.xAccelStart);
			vTaskResume(upDriver.xAccelStop);
			break;
		}
		//-----------------dounDriver functionality
		case 'A':
		{
			startDounDriver();
			break;
		}
		case 'B':
		{
			stopDounDriver();			
			break;
		}
		case 'C':
		{
			dounDriver.Pgoal=GetAngle(dounDriver.PCur+dounDriver.Pgoal);//Set realangle on which we need to stand
			
			if(dounDriver.VCur==0){
				setDirection(&dounDriver);
			}	
			standDounDriverOnAngle();
			break;
		}	
		case 'D':
		{
			if(dounDriver.VCur==0){
				setDirection(&dounDriver);
			}				
			standDounDriverOnAngle();
			break;
		}
		case 'E':
		{
			upDriver.Pgoal=upDriver.PCur+upDriver.Pgoal;//Set realangle on which we need to stand
			upDriver.Pgoal=(upDriver.Pgoal>140)?140:(upDriver.Pgoal<0)?0:upDriver.Pgoal;
			setDirection(&upDriver);
			
			standUpDriverOnAngle();
			vTaskResume(upDriver.xStopWithRun);				
			break;
		}
		//-----------------upDriver functionality
		case 'F':
		{
			upDriver.Pgoal+=70;
			setDirection(&upDriver);
			
			standUpDriverOnAngle();
			break;
		}
		case 'G':
		{
			dounDriver.Pgoal=GetAngle(dounDriver.PCur+dounDriver.Pgoal);//Set realangle on which we need to stand	
			if(dounDriver.VCur==0){
				setDirection(&dounDriver);
			}	
			
			upDriver.Pgoal=upDriver.PCur+upDriver.Pgoal;//Set realangle on which we need to stand
			upDriver.Pgoal=(upDriver.Pgoal>140)?140:(upDriver.Pgoal<0)?0:upDriver.Pgoal;
			setDirection(&upDriver);

			
			standDounDriverOnAngle();
			standUpDriverOnAngle();

			break;
		}
		//-----------------doun+upDriver functionality
		case 'I':
		{
			if(dounDriver.VCur==0){
				setDirection(&dounDriver);
			}
			
			upDriver.Pgoal+=70;
			setDirection(&upDriver);
			
			standDounDriverOnAngle();
			standUpDriverOnAngle();


			break;
		}
		case 'J':
		{
			upDriver.PCur=X_Inclinometer+70;
			break;
		}
		default:
		{
			break;
		}
	}
}
		
void vMainSheduler(void* pvParameter)
{
	//const sMainSheduler* localShedulerData=(sMainSheduler*) pvParameter;
	xTaskHandle* selfHandle=(xTaskHandle*) pvParameter;
	while(1)
	{
		
		parceBuffer();
		turnTasks();
		
		vTaskSuspend(*selfHandle);
	}
}
	
static uint16_t Counter=0;

void vCounter(void* pvParameter)
{
	while(1)
	{
		Counter++;
		vTaskDelay(20000);
	}
}
// ============================================================================
int main(void )
{
	SystemCoreClockUpdate();
	DebugLed_ini();  // DISCVOERY DEBUG COMMAND LEDS
	UpDriver_ini();    // (PC6 PU) (PC7 DIR) (PC8 MF)
	DownDriver_ini();  // (PB13 PU) (PB14 DIR) (PB15 MF)	
	init_SPI1(); // PA4-CSB PA5-SCK PA6-MISO PA7-MOSI
	GPIO_SetBits(GPIOB,GPIO_Pin_13 | GPIO_Pin_15); // DOWN DRIVER
	GPIO_SetBits(GPIOC,GPIO_Pin_6 | GPIO_Pin_8); // UP DRIVER
	adc_configure();//Start configuration
	//xTaskHandle xMainSheduler;
	//-------------------------------------- initialise downDriver -------------------------------
	dounDriver.VCur=0;
	dounDriver.PCur=0;
	dounDriver.mutexPCur=xSemaphoreCreateMutex();
	dounDriver.Pgoal=0;
	dounDriver.Vmin=3;
	dounDriver.Vmax=120;
	dounDriver.Accel=120;
	dounDriver.directionPin=GPIO_Pin_13;
	dounDriver.tooglePin=GPIO_Pin_14;
	dounDriver.powerPin=GPIO_Pin_15;
	dounDriver.Port=GPIOB;
	dounDriver.sleepMode=0;
	dounDriver.flagStart=0;
	dounDriver.DIR=1;
	dounDriver.AngelPerStep=0.0225;
	
	//-----------------------------------  initialise upDriver settings ----------------
	upDriver.VCur=0;
	upDriver.PCur=70;
	upDriver.mutexPCur=xSemaphoreCreateMutex();
	upDriver.Pgoal=60;
	upDriver.Vmin=3;
	upDriver.Vmax=10;
	upDriver.Accel=10;// dont set les then 10
	upDriver.directionPin=GPIO_Pin_6;			//DIR
	upDriver.tooglePin=GPIO_Pin_7;				//PULS	
	upDriver.powerPin=GPIO_Pin_8;					//MF
	upDriver.Port=GPIOC;
	upDriver.sleepMode=0;
	upDriver.flagStart=0;
	upDriver.DIR=1;
	upDriver.AngelPerStep=0.0225;
	//-----------------------------------  initialise mainShedulerObjects settings ----------------
	mainShedulerObjects.Buffer=&cmdBuffer[0];
	mainShedulerObjects.dounDriver=&dounDriver;
	mainShedulerObjects.upDriver=&upDriver;
	  //struct uip_eth_addr mac = { { 0x00, 0x01, 0x02, 0x03, 0x04, 0x00 } };
	  struct uip_eth_addr mac = { {0x54,0x55,0x58,0x10,0x00,0x29} };
	  //mymac[6] = {0x54,0x55,0x58,0x10,0x00,0x29};
		
	enc28j60_init(mac.addr);

	uip_init();
	uip_arp_init();

	hello_world_init();

	uip_setethaddr(mac);

	uip_ipaddr_t ipaddr;
	uip_ipaddr(ipaddr, 192, 168, 1, 65);
	uip_sethostaddr(ipaddr);
	uip_ipaddr(ipaddr, 192, 168, 1, 1);
	uip_setdraddr(ipaddr);
	uip_ipaddr(ipaddr, 255, 255, 255, 0);
	uip_setnetmask(ipaddr);
	
	///*
	xTaskCreate(vHolla_ADC,(signed char *)"vHolla_ADC", 256, NULL, tskIDLE_PRIORITY+1, &xHolla_ADC);
	xTaskCreate(vCompassAndInc,(signed char *)"vCompassAndInc", 256, NULL, tskIDLE_PRIORITY+1, &xCompassAndInc);
	//--------------------------------------- LAN -------------------------------------------
	xTaskCreate( vTask_uIP_periodic, ( signed char * ) "uIPp",configMINIMAL_STACK_SIZE*2, NULL, 1, ( xTaskHandle * ) NULL);			
	//xTaskCreate( vTask_uIP, ( signed char * ) "uIP",configMINIMAL_STACK_SIZE*2, NULL, 3, ( xTaskHandle * ) NULL);		
	xTaskCreate( vTask_uIP, ( signed char * ) "uIP",512, NULL, 1, ( xTaskHandle * ) NULL);		
	//-------------------------------------- Down Dricver Tasks -------------------------------------------
	xTaskCreate(vToogleBits,							(signed char *)"vDounToogleBits", 
							configMINIMAL_STACK_SIZE, (void*) 				&dounDriver, 
							tskIDLE_PRIORITY+1, 			&dounDriver.xToogleBits);
	vTaskSuspend(dounDriver.xToogleBits);
							
	xTaskCreate(vAccelStart,							(signed char *)"vDounAccelStart", 
							configMINIMAL_STACK_SIZE, (void*) 				&dounDriver, 
							tskIDLE_PRIORITY+1, 			&dounDriver.xAccelStart);
	vTaskSuspend(dounDriver.xAccelStart);
							
	xTaskCreate(vAccelStop,								(signed char *)"vDounAccelStop", 
							configMINIMAL_STACK_SIZE, (void*) &dounDriver, 
							tskIDLE_PRIORITY+1, 			&dounDriver.xAccelStop);	
	vTaskSuspend(dounDriver.xAccelStop);
							
	xTaskCreate(vStopWithRun,				(signed char *)"vDounStopWithRun", 
							256, (void*) 				&dounDriver, 
							tskIDLE_PRIORITY+1, &dounDriver.xStopWithRun);
	vTaskSuspend(dounDriver.xStopWithRun);
							
	/*xTaskCreate(vStopWithReturn,		(signed char *)"vDounStopWithReturn", 
							512, (void*) 				&dounDriver, 
							tskIDLE_PRIORITY+1, &dounDriver.xStopWithReturn);
	vTaskSuspend(dounDriver.xStopWithReturn);	*/
	//-------------------------------------- Up DriverTasks -------------------------------------------
	/*xTaskCreate(vAccelStart,				(signed char *)"vUpAccelStart", 
							configMINIMAL_STACK_SIZE, 								(void*) &upDriver, 
							tskIDLE_PRIORITY+1, &upDriver.xAccelStart);
	vTaskSuspend(upDriver.xAccelStart);
							
	xTaskCreate(vAccelStop,								(signed char *)"vUpAccelStop", 
							configMINIMAL_STACK_SIZE, (void*) &upDriver, 
							tskIDLE_PRIORITY+1, 		  &upDriver.xAccelStop);
	vTaskSuspend(upDriver.xAccelStop);*/
	xTaskCreate(vStopWithRun,				(signed char *)"vUpStopWithRun", 
							configMINIMAL_STACK_SIZE, (void*) 				&upDriver, 
							tskIDLE_PRIORITY+1, &upDriver.xStopWithRun);
	vTaskSuspend(upDriver.xStopWithRun);
							
	xTaskCreate(vToogleBits,						 (signed char *)"vUpToogleBits", 
							configMINIMAL_STACK_SIZE,(void*) &upDriver, 
							tskIDLE_PRIORITY+1, 		 &upDriver.xToogleBits);
	vTaskSuspend(upDriver.xToogleBits);
	//-------------------------------------- Main Sheduler Task -------------------------------------------
	xTaskCreate(vMainSheduler, 					 (signed char *) "vMainSheduler",
							configMINIMAL_STACK_SIZE,(void*) &xMainSheduler, 
							tskIDLE_PRIORITY+1, 												&xMainSheduler);		
	vTaskSuspend(xMainSheduler);						
	//-------------------------------------- Test Tasks -------------------------------------------
	//xTaskCreate(vHolaCounter,(signed char *)"vHolaCounter", 256, NULL, tskIDLE_PRIORITY+1,&xHolaCounter);	
	//vTaskSuspend(xHolaCounter);	
	/*xTaskCreate(vMainSheduler, ( signed char * ) "vMainSheduler",
							configMINIMAL_STACK_SIZE,(void*) &mainShedulerObjects, 
							1, &xMainSheduler);		
	vTaskSuspend(xMainSheduler);*/
	//
	//xTaskCreate(vCounter, ( signed char * ) "vCounter",configMINIMAL_STACK_SIZE, NULL, 1, NULL);		
   
	vTaskStartScheduler(); // This should never return.
}
// *********************************************************************************
// *********************************************************************************
// *********************************************************************************
// *********************************************************************************
//**********************************************************************************



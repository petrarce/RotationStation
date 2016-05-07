#ifndef ALLDEFINES_H
#define ALLDEFINES_H

#include "stm32f4xx.h"

#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
//#include "FreeRTOS.h"

#define M_PI 3.14159265358979323846
#define DEG_SEC 44.4 		//frequency for 1 deg/sec
#define BASE configTICK_RATE_HZ		//system frequency 200000 == 9 Pulse by 10 sec 20000 +1 pulse so 222222 10 Pulse by 10sec
#define SEC BASE
#define MAX_DRIVER_FREQ 16000
#define INCLI_DEALY BASE/100
#define UIP_DELAY BASE/100
#define HOLA_DELAY BASE/100
#define PCUR_MUTEX_DELAY BASE/100
#define dt 0.01




static float InclinometerSensitivity = 819.00;
//----------------------------------------- PROTOTYPES FOR FUNCTIONS ---------------------------------------------------------
void SendStrToPC(char* str_p); // USART1 func - otpravka simvolnuh kommand(peredaem masiv)    PROVERIT I PRIMER USA NAPISAT
//void SendDataToPC(char* str_p, uint16_t count); 	// USART1 fun -  peredaem obu4nue dannue bez IRQ   PROVERIT I PRIMER USA NAPISAT
void SendDataToPC(float);
void Send_with_IT_PC(void); // USART1 func - for IRQ handler iniciiruem pervuyu pereda4u			  PROVERIT I PRIMER USA NAPISAT
void SendSingleComand(void); // USART1 v tele Send_with_IT_PC + delay_ms(10000) 10sec					PROVERIT I PRIMER USA NAPISAT

void my_USART2_Ini(void);  // COMPASS
void USART2_IRQHandler(void);  // REC_PACK_FROM_UPBOARD_AND_SEND_INC_TERM
void SendToCompas(uint8_t* cmd, uint8_t lenght); 
void UpdatePitchRollHeadingInclinometerData(void);
void init_usart3(void);  //not need more
void init_SPI1(void); 
void SendToF4(void);  

void SysTick_Handler(void); // SYSTICK HANDLER FOR DELAY MS
void delay_ms(uint16_t); // DELAY MS OF SYSTICK HANDLER

void DebugLed_ini(void);
void UpDriver_ini(void);
void DownDriver_ini(void);

void ADC_Holla_Regular(void);	//1 channel
void ADC_Holla_Inject(void);  //4 registra scan mode odnovremenno 4 channela

void INIT_SysTick(int);
//-----------------------------------------------------------------------------------
#endif

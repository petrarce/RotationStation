#include "stm32f4xx_conf.h"
#include "alldefines.h"
#include "math.h"
char mybuff[]= "I am DMA!"; //vstavlyaem v strukturu DMA ini

//============================================= DEFINES =====================================================================

//============================================= DEFINES END =================================================================

void INIT_SysTick(int BASE)
{
	SysTick_Config(SystemCoreClock/BASE);
}

void DebugLed_ini(void)
{
	GPIO_InitTypeDef GPIO_Init_LED;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	GPIO_Init_LED.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_Init_LED.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init_LED.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init_LED.GPIO_OType = GPIO_OType_PP;
	GPIO_Init_LED.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
	GPIO_Init(GPIOD, &GPIO_Init_LED);
}
//***************************************************************************************************************************
//******************************************** DOWN DRIVER PB 13 14 15 ******************************************************
//***************************************************************************************************************************
// PB13 DIR
// PB14 PULSE
// PB15 MF
void DownDriver_ini(void)
{
	GPIO_InitTypeDef GPIO_Init_LED;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_Init_LED.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_Init_LED.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init_LED.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init_LED.GPIO_OType = GPIO_OType_PP;
	GPIO_Init_LED.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
	GPIO_Init(GPIOB, &GPIO_Init_LED);
}
//-------------------------------- NEED TO CHANGE PORTS-----------------------

//---  ADC HOLLA A0 A1 A2 A3 4095 urovnei kvantovaniya ----- AS regular mode--------
/*
void ADC_Holla_Regular(void)
{
	GPIO_InitTypeDef GPIO_ADC_Holla;
	ADC_InitTypeDef ADC_Ini_Holla; 
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	GPIO_ADC_Holla.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2 |GPIO_Pin_3;
	GPIO_ADC_Holla.GPIO_Mode = GPIO_Mode_AN;
	GPIO_ADC_Holla.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_ADC_Holla.GPIO_OType = GPIO_OType_PP;
	GPIO_ADC_Holla.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
	GPIO_Init(GPIOC, &GPIO_ADC_Holla);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC, ENABLE);
	
	ADC_Ini_Holla.ADC_Resolution = ADC_Resolution_12b; // 4096 urovnei kvantovaniya 0-4095
  ADC_Ini_Holla.ADC_ScanConvMode = DISABLE;					//  ADC skaniruet srazu neskolko kanalov po o4eredi
  ADC_Ini_Holla.ADC_ContinuousConvMode = DISABLE;		// zapusk preobrazovaniya srazu kak tolko zakon4ilos predidushie
  ADC_Ini_Holla.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; 	//po sobutiyu nabprimer trige po timeru
  //po kakomu konkretno sobutiyu zapuskaetsa ADC, esli ADC_ExternalTrigConvEdge vuklu4en to i kakoe tut sobutie ne vagno
	ADC_Ini_Holla.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;	
	//ot 0 do 4095 a esli left to 4095*16 eto 12bit zna4enie mogem peredavat po SPI
  ADC_Ini_Holla.ADC_DataAlign = ADC_DataAlign_Right; //toest right Zoom zna4eniya net a left x16 zoom zna4eniya
  //skolko kanalov mu budem skanirovat tak kak ADC_Ini_Holla.ADC_ScanConvMode = DISABLE;
	ADC_Ini_Holla.ADC_NbrOfConversion = 1; // to muskaniruem tolko 1 channel ADC

	ADC_Init(ADC1, &ADC_Ini_Holla);
	//ukazuvaem s kakovovoobshe chennela we read data, rang skanirovaniya 1 2 3 4 o4ered
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_56Cycles);
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_56Cycles);
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_56Cycles);
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_56Cycles);
	//---- razreshaem IRQ 4tob ekonomit RAM-----
	NVIC_EnableIRQ(ADC_IRQn);
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE); //EOC end of convertion flag
	ADC_Cmd(ADC1, ENABLE);
}
*/
/*
void ADC_Holla_Inject(void)
{
	GPIO_InitTypeDef GPIO_ADC_Holla_Inject;
	ADC_InitTypeDef ADC_Ini_Holla_Inject; 
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	GPIO_ADC_Holla_Inject.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2 |GPIO_Pin_3;
	GPIO_ADC_Holla_Inject.GPIO_Mode = GPIO_Mode_AN;
	GPIO_ADC_Holla_Inject.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_ADC_Holla_Inject.GPIO_OType = GPIO_OType_PP;
	GPIO_ADC_Holla_Inject.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
	GPIO_Init(GPIOC, &GPIO_ADC_Holla_Inject);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC, ENABLE);
	
	ADC_Ini_Holla_Inject.ADC_Resolution = ADC_Resolution_12b; // 4096 urovnei kvantovaniya 0-4095
  ADC_Ini_Holla_Inject.ADC_ScanConvMode = ENABLE;					//  ADC skaniruet srazu neskolko kanalov po o4eredi
  ADC_Ini_Holla_Inject.ADC_ContinuousConvMode = DISABLE;		// zapusk preobrazovaniya srazu kak tolko zakon4ilos predidushie off
  ADC_Ini_Holla_Inject.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; 	//trigger off
  //po kakomu konkretno sobutiyu zapuskaetsa ADC, esli ADC_ExternalTrigConvEdge vuklu4en to i kakoe tut sobutie ne vagno
	ADC_Ini_Holla_Inject.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;	
	//ot 0 do 4095 a esli left to 4095*16 eto 12bit zna4enie mogem peredavat po SPI
  ADC_Ini_Holla_Inject.ADC_DataAlign = ADC_DataAlign_Right; //toest right Zoom zna4eniya net a left x16 zoom zna4eniya
  //skolko kanalov mu budem skanirovat tak kak ADC_Ini_Holla.ADC_ScanConvMode = DISABLE;
	ADC_Ini_Holla_Inject.ADC_NbrOfConversion = 4; // to muskaniruem tolko 1 channel ADC

	ADC_Init(ADC1, &ADC_Ini_Holla_Inject);
	
//	ADC_InjectedSequencerLenghtConfig(ADC1,4);
	//ukazuvaem s kakovovoobshe chennela we read data, rang skanirovaniya 1 2 3 4 o4ered
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_56Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_56Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_56Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_56Cycles);
	//---- razreshaem IRQ 4tob ekonomit RAM-----
	NVIC_EnableIRQ(ADC_IRQn);
	ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE); //EOC end of convertion flag
	ADC_Cmd(ADC1, ENABLE);
}
*/


//------------------------------ UP DRIVER PC6 PC7 PC8 ---------------------------------------------------
// PC6 - PU
// PC7 - DIR
// PC8 - MF
void UpDriver_ini(void)
{
	GPIO_InitTypeDef GPIO_Init_LED;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	GPIO_Init_LED.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;
	GPIO_Init_LED.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init_LED.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init_LED.GPIO_OType = GPIO_OType_PP;
	GPIO_Init_LED.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
	GPIO_Init(GPIOC, &GPIO_Init_LED);
}
//***************************************************************************************************************************
//*************************************************** END LEDS PB 13 14 15 **************************************************
//***************************************************************************************************************************


//=============================================== USART 2 COM PORT TERMINAL =================================================
//*********************************************** AND USART 2 IRQ HANDLER ***************************************************
//**************************************************** PA 2   PA3    ********************************************************
void my_USART2_Ini(void)
{
	GPIO_InitTypeDef GPIO_Ini_USART2;
	USART_InitTypeDef USART2_MyInit;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_Ini_USART2.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_Ini_USART2.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Ini_USART2.GPIO_OType = GPIO_OType_PP;
	GPIO_Ini_USART2.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Ini_USART2.GPIO_Speed = GPIO_Speed_50MHz;
  
	GPIO_Init(GPIOA, &GPIO_Ini_USART2);
 	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	
	// nastroili vuvodu tepr sam USART nastraivaem
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	USART2_MyInit.USART_BaudRate = 9600;
	USART2_MyInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART2_MyInit.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART2_MyInit.USART_Parity = USART_Parity_No;
	USART2_MyInit.USART_StopBits = USART_StopBits_1;
	USART2_MyInit.USART_WordLength = USART_WordLength_8b;
	
	USART_Init(USART2, &USART2_MyInit);
	
	NVIC_EnableIRQ(USART2_IRQn);  //razreshit obshie prerivaniya
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	
	
	USART_Cmd(USART2, ENABLE); // vklu4aem USART2
}
//*********************************************** END USART 2 COM PORT TERMINAL **********************************************
//*********************************************** AND USART 2 IRQ HANDLER ****************************************************
//==================================================== PA 2   PA3    =========================================================


//****************************************************************************************************************************
//************************************* REC PACKEGE AND SEND INC_TERM to/from UPPER BOARD ************************************
//****************************************************************************************************************************
void init_usart3()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef gpioUsart3;
	gpioUsart3.GPIO_OType = GPIO_OType_PP;
	gpioUsart3.GPIO_PuPd = GPIO_PuPd_UP;
	gpioUsart3.GPIO_Mode = GPIO_Mode_AF;
	gpioUsart3.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	gpioUsart3.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpioUsart3); //initialize

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	//110, 150, 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600 (115200 PRACTICAL LIMIT OF RS232)
	USART_InitTypeDef usart3;
	usart3.USART_BaudRate            = 57600;
	usart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart3.USART_Mode				 = USART_Mode_Tx | USART_Mode_Rx;
	usart3.USART_Parity				 = USART_Parity_No;
	usart3.USART_StopBits			 = USART_StopBits_1;
	usart3.USART_WordLength 		 = USART_WordLength_8b;
	USART_Init(USART3, &usart3);

	NVIC_EnableIRQ(USART3_IRQn);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART3, ENABLE);
}
//----------------------------------------------END REC PACKEGE AND SEND INC_TERM to/from UPPER BOARD  -----------------------
//****************************************************************************************************************************
//****************************************************************************************************************************
void init_SPI1()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* configure pins used by SPI1
	* PA4 = Select
	* PA5 = SCK
	* PA6 = MISO
	* PA7 = MOSI*/

	/* Configure SPIy pins: SCK, MISO and MOSI ---------------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);		

	GPIO_SetBits(GPIOA, GPIO_Pin_4);

	// enable peripheral clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* configure SPI1 in Mode 0
	 * CPOL = 0 --> clock is low when idle
	 * CPHA = 0 --> data is sampled at the first edge
	 */
	SPI_InitTypeDef SPI_InitStruct;
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;     // transmit in master mode, NSS pin has to be always high
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;        // clock is low when idle
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;      // data sampled at first edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;// | SPI_NSSInternalSoft_Set; // set the NSS management to internal and pull internal NSS high
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; // SPI frequency is APB2 frequency / 4
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
	SPI_Init(SPI1, &SPI_InitStruct);

	SPI_Cmd(SPI1, ENABLE); // enable SPI1
}



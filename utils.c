//*********************** Prototipo Serbena **************************
//    Autor: Vinicius Gardin Pires da Silva
//    Pinos utilizados:
//
//    PA00 - EXTI
//    PA01 - ADC1
//    PA05 - SPI1 SCK
//    PA07 - SPI1 MOSI
//    PB00 - FSYNC 1
//    PB01 - FSYNC 2
//    PC13 - Led
//
//    Bibliotecas utilizadas:
//
//    stm32f10x.h		Biblioteca do STM32f10
//    stm32f10x_adc.h 	Biblioteca do ADC
//    stm32f10x_dma.h 	Biblioteca do DMA
//    stm32f10x_rcc.h 	Biblioteca do clock
//    stm32f10x_spi.h	Biblioteca do SPI
//    stm32f10x_it.h  	Biblioteca do interrupções
//
//********************************************************************
//    DATA      |	Descrição
//********************************************************************
// 28/06/2021   |	Conexão com github
// 29/06/2021	|	Add LED
// 08/07/2021	|	Add SPI
// 13/07/2021   |   Add utils
//--------------------------------------------------------------------

#include "utils.h"

/*
 * TODO: ADC Disc mode enable, AFIO?
 *
 * TODO: ADC external trigger
 *
 * TODO: ADC change external trigger function
 *
 * TODO: Phase correction Marlio codigo
 *
 * TODO: CPOL e CPHA do SPI estão certos?
 *
 * TODO: Calcular corretamente o tempo da função delay
 *
 * TODO: Verificar os delays nas funções do AD9833
 *
 * TODO: SPI2 para debbugar
 *
 */

/*******************************************************************/
/*                     Funções do AD9833                           */
/*******************************************************************/

/*
 * Recomendação do manual é resetar sempre ao ligar. Ao resetar ele
 * não altera os valores de frequencia e fase
 */
void reset_AD9833() {
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);
	GPIO_ResetBits(GPIOB, GPIO_Pin_1);

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SP1, RESET_CMD);
	delay(15);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SP1, NORMAL_CMD);

	GPIO_SetBits(GPIOB, GPIO_Pin_0);
	GPIO_SetBits(GPIOB, GPIO_Pin_1);
}

/*
 * Setup and apply a signal, frequency in Hz
 */
void init_AD9833(float frequency) {
	reset_AD9833();
	writeRegisterA(SINE_WAVE);
	writeRegisterB(SQUARE_WAVE);
	SetFrequency(frequency);
}

/*
 *  Set the specified frequency register with the frequency (in Hz)
 */
void setFrequency(float frequency) {
	if (frequency > 6e6)
		frequency = 6e6;
	if (frequency < 0.0) 
		frequency = 0.0;
	
	uint32_t freqWord = (uint32_t)(frequency * 10.73741824);
	uint16_t upper14 = (uint16_t)((freqWord & 0xFFFC000) >> 14), 
			lower14 = (uint16_t)(freqWord & 0x3FFF);

	lower14 |= REG0;
	upper14 |= REG0;   

  	writeRegisterA(FREQ_WRITE_CMD); 
	writeRegisterA(lower14);			//Write lower 14 bits to AD9833
	writeRegisterA(upper14);			//Write upper 14 bits to AD9833
  	writeRegisterA(PHASE_WRITE_CMD);	//Phase = 0

	freqWord = (uint32_t)(2*frequency * 10.73741824);
	upper14 = (int16_t)((freqWord & 0xFFFC000) >> 14);
	lower14 = (int16_t)(freqWord & 0x3FFF);

	lower14 |= REG0;
	upper14 |= REG0;   

  	writeRegisterB(FREQ_WRITE_CMD); 
	writeRegisterB(lower14);			//Write lower 14 bits to AD9833
	writeRegisterB(upper14);			//Write upper 14 bits to AD9833
  	writeRegisterB(PHASE_WRITE_CMD);	//Phase = 0
}

void sleep_AD9833() {
    writeRegisterA(SLEEP); 
    writeRegisterB(SLEEP); 
}

void writeRegisterA( uint16_t command ) {
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SP1, command);
	GPIO_SetBits(GPIOB, GPIO_Pin_0);
}

void writeRegisterB( uint16_t command ) {
	GPIO_ResetBits(GPIOB, GPIO_Pin_1);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SP1, command);
	GPIO_SetBits(GPIOB, GPIO_Pin_1);
}

/*******************************************************************/
/*               Funções de configuração do STM                    */
/*******************************************************************/

void init_Clock() {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  |
		      		RCC_APB2Periph_GPIOB |
		      		RCC_APB2Periph_GPIOC |
		      		RCC_APB2Periph_ADC1  |
		      		RCC_APB2Periph_SPI1  | 
				    RCC_APB2Periph_AFIO,	ENABLE);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  	RCC_ADCCLKConfig(RCC_PCLK2_Div2); 
  	RCC_PCLK2Config(RCC_HCLK_Div2); 

	/* RCC configuration STM librarie                            */
  	/* PCLK2 = HCLK/2 */
	RCC_PCLK2Config(RCC_HCLK_Div2); 
}

void init_GPIO()
{
  /* Configure ADC pin ---------------------------------------------*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure SPI1 pins: SCK e MOSI -------------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

  /* Configure SPI1 pins: FSYNC1 e FSYNC2 --------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

  /* Configure LED builtin pin -------------------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

//SPI2 para debbugar INCOMPLETO
//  /* Configure SPI2 pins: SCK e MOSI -------------------------------*/
//	GPIO_InitTypeDef GPIO_InitStructure3;
//	GPIO_InitStructure3.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
//	GPIO_InitStructure3.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure3.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_Init(GPIOA, &GPIO_InitStructure3);	
//  /* Configure SPI2 pin: SS ---------------------------------------*/
//	GPIO_InitTypeDef GPIO_InitStructure3;
//	GPIO_InitStructure3.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
//	GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStructure3.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_Init(GPIOA, &GPIO_InitStructure3);	
	
}

void init_ADC() {

	ADC_InitTypeDef ADC_InitStruct;
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_Ext_IT11_TIM8_TRGO;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStruct);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_1Cycles5);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);

	/* Regular discontinuous mode channel number configuration */
	ADC_DiscModeChannelCountConfig(ADC1, 1);
	/* Enable regular discontinuous mode */
	ADC_DiscModeCmd(ADC1, ENABLE);
	
	/* Enable ADC1 external trigger conversion */
	ADC_ExternalTrigConvCmd(ADC1, ENABLE);


	//Before starting a calibration, the ADC must have been in
	//power-on state (ADON bit = ‘1’) for at least two ADC clock
	//cycles. It is recommended to perform a calibration after
	//each power-up.
	for(long i = 0; i<3; i++){__NOP();}
  	//Processo de calibração
  	ADC_ResetCalibration(ADC1);
  	while(ADC_GetResetCalibrationStatus(ADC1));
  	ADC_StartCalibration(ADC1);
  	while(ADC_GetCalibrationStatus(ADC1));
}

/*
 * External event on PA00 RISE or FALL
 *
 */
void init_EXT( int externalTrigger ) {
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

	EXTI_InitTypeDef EXTI_InitStruct;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Event;

	if(externalTrigger == RISE)
		EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	else
		EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;

	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
}


/*
 * Configuração do DMA1 para receber informações do
 * ADC 1 e enviar para &data, com um buffer de 1 no modo
 * circular
 *
 */
void init_DMA(){
	DMA_InitTypeDef DMA_Initstruct;
	DMA_Initstruct.DMA_PeripheralBaseAddr = ADC1_DR_Address; 
	DMA_Initstruct.DMA_MemoryBaseAddr = (uint32_t)&data;    
	DMA_Initstruct.DMA_DIR = DMA_DIR_PeripheralSRC; 
	DMA_Initstruct.DMA_BufferSize = 1; 
	DMA_Initstruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_Initstruct.DMA_MemoryInc = DMA_MemoryInc_Disable; 
	DMA_Initstruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_Initstruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_Initstruct.DMA_Mode = DMA_Mode_Circular;
	DMA_Initstruct.DMA_Priority = DMA_Priority_High;
	DMA_Initstruct.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_Initstruct);

	DMA_Cmd(DMA1_Channel1,ENABLE);

	//DMA Transfer complete
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
}

/*
 * Configura o SPI1 para ser o mestre com uma comunicação onde 
 * só ele fala com dados de 16 bits e SS/FSYNC via hardware que
 * no nosso caso é por GPIO
 */
void init_SPI() {
	//Configura o SPI1 para comunicar com os AD9833.
	SPI_InitTypeDef SPI_InitStruct;
	SPI_InitStruct.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Hard;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStruct);

	SPI_Cmd(SPI1, ENABLE);

//SPI2 para Debuggar INCOMPLETO
//	SPI_InitTypeDef SPI_InitStruct2;
//
//	SPI_InitStruct2.SPI_Direction = SPI_Direction_1Line_Tx;
//	SPI_InitStruct2.SPI_Mode = SPI_Mode_Master
//	SPI_InitStruct2.SPI_DataSize = SPI_DataSize_16b;
//	SPI_InitStruct2.SPI_CPOL = SPI_CPOL_Low;
//	SPI_InitStruct2.SPI_CPHA = SPI_CPHA_1Edge;
//	SPI_InitStruct2.SPI_NSS = SPI_NSS_Hard;
//	SPI_InitStruct2.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
//	SPI_InitStruct2.SPI_FirstBit = SPI_FirstBit_MSB;
//	SPI_InitStruct2.SPI_CRCPolynomial = 7;
//	SPI_Init(SPI2, &SPI_InitStruct2);
//
//	SPI_Cmd(SPI2, ENABLE);
}

/* 
 * Configure and enable DMA interrupt 
 */
void init_NVIC() {
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void delay() {
	for(long i = 0; i<SystemCoreClock/30; i++){__NOP();}
}

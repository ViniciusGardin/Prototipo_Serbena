//########################################################
// Descrição
// 
// Utiliza o pino PA01 para fazer uma conversão pelo ADC 
// utilizando o canal 01 do ADC1 continuamente. Os valores 
// da conversão são colocados na variavel diretamente na 
// memoria pelo DMA.
//########################################################
// Pinos conectados
//
// PA01 - ADC
//########################################################
//Bibliotecas Utilizadas
//
// stm32f10x.h	   Biblioteca do STM32f10
// stm32f10x_adc.h Biblioteca do ADC
// stm32f10x_dma.h Biblioteca do DMA
// stm32f10x_rcc.h Biblioteca do clock
// stm32f10x_it.h  Biblioteca do interrupções

//PA01 ADC1 IN1		OK!
//ADC 			OK!
//Interrupt		
//DMA ADC1 Ch1 DMA1	OK!
//GPIO PA01 ADC1	OK!


#include <stm32f10x.h>

#define ADC1_DR_Address    ((uint32_t)0x4001244C)
uint16_t data;

//#define DMA1_Channel1_IT_Mask    ((uint32_t)(DMA_ISR_GIF1 | DMA_ISR_TCIF1 | DMA_ISR_HTIF1 | DMA_ISR_TEIF1))

//ITStatus ADC_GetITStatus(ADC_TypeDef* ADCx, uint16_t ADC_IT);
//void ADC_ClearITPendingBit(ADC_TypeDef* ADCx, uint16_t ADC_IT);
//void DMA_SetCurrDataCounter(DMA_Channel_TypeDef* DMAy_Channelx, uint16_t DataNumber); 
//uint16_t DMA_GetCurrDataCounter(DMA_Channel_TypeDef* DMAy_Channelx);
//FlagStatus DMA_GetFlagStatus(uint32_t DMAy_FLAG);
//void DMA_ClearFlag(uint32_t DMAy_FLAG);
//ITStatus DMA_GetITStatus(uint32_t DMAy_IT);
//void DMA_ClearITPendingBit(uint32_t DMAy_IT);

void init_Clock() {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  	RCC_ADCCLKConfig(RCC_PCLK2_Div2); 
}



void init_ADC() {
	ADC_InitTypeDef ADC_InitStruct;

	//Single-channel continuous conversionMode
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStruct);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_1Cycles5);

	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);

	//Before starting a calibration, the ADC must have been in
	//power-on state (ADON bit = ‘1’) for at least two ADC clock
	//cycles. It is recommended to perform a calibration after
	//each power-up.
	
  	/* Enable ADC1 reset calibration register */   
  	ADC_ResetCalibration(ADC1);

  	/* Check the end of ADC1 reset calibration register */
  	while(ADC_GetResetCalibrationStatus(ADC1));

  	/* Start ADC1 calibration */
  	ADC_StartCalibration(ADC1);
  	/* Check the end of ADC1 calibration */
  	while(ADC_GetCalibrationStatus(ADC1));

  	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

}

void init_DMA(){
	DMA_InitTypeDef DMA_Initstruct;
	
	//Configuração do DMA1 para receber informações do
	//ADC 1 e enviar para &data, com um buffer de 4 no modo
	//circular
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

void init_GPIO()
{
	//PA01
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void init_NVIC() {
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure and enable ADC interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

int main(void) {
	init_Clock();
	init_ADC();
	init_DMA();
	init_GPIO();
	init_NVIC();

	while(1) {
		for(long i = 0; i<SystemCoreClock/30; i++){__NOP();}
	}
}


#include <stm32f10x.h>
#include "stm32f10x_conf.h"
#include "utils.h"

uint16_t data = 0;
int DMA_flag = 0;

int main(void) {
	ADC_InitTypeDef ADC_InitStruct;
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_NbrOfChannel = 1;
	init_ADC(&ADC_InitStruct);

	init_Clock();
	init_GPIO();
	init_ADC();
	init_DMA();
	init_SPI();
	init_NVIC();
	init_AD9833(FrequenciaInicial);
	//reset_AD9833();
	//setFrequency(float frequency);
	//sleep_AD9833();

	while(1) {
		if(DMA_flag) {
			GPIO_SetBits(GPIOC, GPIO_Pin_13);
			delay();
			GPIO_ResetBits(GPIOC, GPIO_Pin_13);
			delay();
		}
	}
}

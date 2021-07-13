//*********************** Prototipo Serbena **************************
//    Autor: Vinicius Gardin Pires da Silva
//    Pinos utilizados:
//
//    PA01 - ADC
//    PA05 - SPI1 SCK
//    PA07 - SPI1 MOSI
//    PB00 - FSYNC 1
//    PB01 - FSYNC 2
//    PC13 - Led
//
//    Bibliotecas utilizadas:
//
//    stm32f10x.h	Biblioteca do STM32f10
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
//--------------------------------------------------------------------
#include <stm32f10x.h>
#include "stm32f10x_conf.h"
#include "utils.h"

uint16_t data = 0;
int DMA_flag = 0;

int main(void) {
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


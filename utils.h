#ifndef __UTILS_H
#define __UTILS_H

#include <stm32f10x.h>
#include "stm32f10x_conf.h"

//Comandos para o AD9833
#define SINE_WAVE 		 	0x2000
#define TRIANGLE_WAVE	 	0x2002
#define SQUARE_WAVE      	0x2028
#define HALF_SQUARE_WAVE 	0x2020
#define RESET_CMD			0x0100
#define NORMAL_CMD			0x000 
#define PHASE_WRITE_CMD		0xC000
#define FREQ_WRITE_CMD		0x2100
#define SLEEP	 			0x180 
#define REG0				0x4000//Registrador 0

//Endereço para configurar o ADC do STM
#define ADC1_DR_Address    	((uint32_t)0x4001244C) 	

enum {
		RISE = 0,
		FALL
};

void reset_AD9833();
void init_AD9833(float frequency);
void setFrequency(float frequency);
void sleep_AD9833();
void writeRegisterA( uint16_t command ) {
void writeRegisterB( uint16_t command ) {

void init_Clock();	//Config do clock para tudo
void init_GPIO();	//Pinos do processador
void init_ADC();	//Conversor AD
void init_DMA();	//Direct Memory Access
void init_SPI();	//Comuicação com AD9833 e debuggar
void init_NVIC();	//Interrupções
void delay();

#endif

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

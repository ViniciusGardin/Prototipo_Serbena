#include <stm32f10x.h>
#include "stm32f10x_conf.h"
#include "utils.h"

uint16_t data = 0;	//Valor de conversão do ADC
int DMA_flag = 0;	//Flag que a interrupção do DMA ativa
uint8_t i = 0;		//Estagio da onda senoidal estamos analisando
uint8_t count = 0;	//Quantas vezes foi feita as mesmas medidas

long Real = 0;
long Imag = 0;


//frequencia inicial = 1 Hz
//frequencia final   = 100k Hz
//numero de pontos   = 101
//frequence increase, power(10,(log10(100 000,1)/101))
float fincr = 1.12074;
double freq  = 1;

//É feito uma media entre os valores obtidos para cada frequencia, entretanto
//para frequencia pequenas (LFavg Low Frequency average) demora muito fazer 
//muitas medidas pelo periodo ser menor, logo ele só começa a fazer as HFavg
//(High Frequency average) depois do LFlim (Low Frequency limit)
//* O operador ">>" é igual a dividir por 2^(numero usado no operador)
const uint16_t HFavg = 128;
const uint16_t LFavg = HFavg >> 2;
const uint16_t avg = LFavg;
const uint16_t LFlim = 20;
const uint16_t HFlim = 100000;

//Funções que não sei se vou usar
//void DMA_SetCurrDataCounter(DMA_Channel_TypeDef* DMAy_Channelx, uint16_t DataNumber); 
//uint16_t DMA_GetCurrDataCounter(DMA_Channel_TypeDef* DMAy_Channelx);
//FlagStatus DMA_GetFlagStatus(uint32_t DMAy_FLAG);
//void DMA_ClearFlag(uint32_t DMAy_FLAG);
//ITStatus DMA_GetITStatus(uint32_t DMAy_IT);
//void DMA_ClearITPendingBit(uint32_t DMAy_IT);

int main(void) {
	init_Clock();
	init_GPIO();
	init_ADC();
	init_DMA();
	init_EXT(FALL);
	init_SPI();
	init_NVIC();
	init_AD9833(freq);
	//reset_AD9833();
	//setFrequency(float frequency);
	//sleep_AD9833();

	ADC_Cmd(ADC1, ENABLE);
 	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while(1) {
		if(DMA_flag) {
			ADC_Cmd(ADC1, DISABLE);
			switch(i) {
				case 1://Sample at falling slope of sinus
					Imag = (long) (-1)*data;
					init_EXT(RISE);
					ADC_Cmd(ADC1, ENABLE);
  					ADC_SoftwareStartConvCmd(ADC1, ENABLE);
					i++;
					break;
				case 2://sample at negative peak of sinus
					Real = (long) (-1)*data;
					init_EXT(FALL);
					ADC_Cmd(ADC1, ENABLE);
  					ADC_SoftwareStartConvCmd(ADC1, ENABLE);
					i++;
					break;
				case 3://Sample at falling slope of sinus
					Imag += (long) data;
					init_EXT(RISE);
					ADC_Cmd(ADC1, ENABLE);
  					ADC_SoftwareStartConvCmd(ADC1, ENABLE);
					i++;
					break;
				case 4://sample at positive peak of sinus
					Real += (long) data;
					init_EXT(FALL);
					ADC_Cmd(ADC1, ENABLE);
  					ADC_SoftwareStartConvCmd(ADC1, ENABLE);
					i++;
					break;
				case 5://End of Sinus
					i = 0;
					freq *= fincr;
					if(freq < LFlim)
						avg = LFavg;
					else
						avg = HFavg;
					if(freq > HFlim){
						sleep_AD9833();
						//Aqui os valores de Real e Imag prontos para cada 
						//frequencia
						while(1);
					}
							
					setFrequency(freq);
					reset_AD9833();
					ADC_Cmd(ADC1, ENABLE);
  					ADC_SoftwareStartConvCmd(ADC1, ENABLE);
		}

		}
	}
}

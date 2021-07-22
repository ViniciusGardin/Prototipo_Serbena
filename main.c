/*
 * TODO: Fazer a calibração
 *
 * TODO: Fazer calculo do modulo
 *
 * TODO: Valor de res
 */

#include <stm32f10x.h>
#include "stm32f10x_conf.h"
#include "utils.h"

uint16_t data = 0;		//Valor de conversão do ADC
uint8_t DMA_flag = 0;		//Flag que a interrupção do DMA ativa
uint8_t count = 0;		//Quantas vezes foi feita as mesmas medidas
uint8_t wavePoint = 1;	//Ponto da onda senoidal estamos analisando

//frequencia inicial = 1 Hz
//frequencia final   = 100k Hz

const uint8_t npts = 101;	//numero de pontos = 101
const uint8_t nptsA = 0;	//numero do ponto atual

long RealVar = 0;
long ImagVar = 0;
long Real[npts] = {0};	//Todos os valores reais da impedancia
long Imag[npts] = {0};	//Todos os valores imaginarios da impedancia
long Zimag[npts] = {0};	//Valores da impedancia no final
long Zreal[npts] = {0};	//Valores da impedancia no final
double freqA[npts] = {0};	//Valores da frequencia para cada ponto

long RealOpen[npts] = {1};
long ImagOpen[npts] = {1};
const uint16_t res = 1;

//Para calcular o passo para cada frequencia, para que ele seja feito de uma
//forma logaritma, é feito  seguinte calculo:
//10^((log10(freqF) - log10(freqI))/n° pts)
//frequence increase, power(10,(log10(100000/1)/101))
float fincr = 1.12074;
double freq  = 1;
freqA[nptsA] = freq;

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

int main(void) {
	init_Clock();
	init_GPIO();
	init_ADC();
	init_DMA();
	init_NVIC();
	init_SPI();
	init_AD9833(freq);
	init_EXT(FALL);
	//reset_AD9833();
	//setFrequency(float frequency);
	//sleep_AD9833();

	while(1) {
		while(count < avg){
			if(DMA_flag) {
				//ADC_Cmd(ADC1, DISABLE);
				switch(wavePoint) {
					case 1://Sample at falling slope of sinus
						ImagVar = -((data)>>1);
						init_EXT(RISE);
						wavePoint++;
						//ADC_Cmd(ADC1, ENABLE);
						break;
					case 2://Sample at negative peak of sinus
						RealVar = -((data)>>1);
						init_EXT(FALL);
						wavePoint++;
						//ADC_Cmd(ADC1, ENABLE);
						break;
					case 3://Sample at falling slope of sinus
						ImagVar += ((data)>>1);
						init_EXT(RISE);
						wavePoint++;
						//ADC_Cmd(ADC1, ENABLE);
						break;
					case 4://Sample at positive peak of sinus
						RealVar += ((data)>>1);
						wavePoint++;
						//ADC_Cmd(ADC1, ENABLE);
						break;
					case 5://End of Sinus
						Real[nptsA] += RealVar/npts;
						Imag[nptsA] += ImagVar/npts;
						wavePoint = 1;
						break;
				}//End os switch
				DMA_flag = 0;
			}//End of if(DMA_flag)
			count++;
		}//End of while(count < avg)
		ADC_Cmd(ADC1, DISABLE);
		count = 0;
		nptsA++;
		if(nptsA >= npts){//Aqui acabou 
				for(int i = 0; i<npts; i++) {
						long var = Res/(power((Real[i]+RealOpen[i]),2)+power((Imag[i] + ImagOpen[i]),2));
						Zimag[i] = Imag[i]*(Real[i]+RealOpen[i])-Real[i]*(Imag[i]+ImagOpen[i]);
						Zimag[i] *= var;
						Zimag[i] = Real[i]*(Real[i]+RealOpen[i])-Imag[i]*(Imag[i]+ImagOpen[i]);
						Zreal[i] *= var;
				}
			ADC_Cmd(ADC1, DISABLE);
			sleep_AD9833();
			while(1){__NOP();}
		}
		freq *= fincr;
		freA[nptsA] = freq;
		setFrequency(freq);
		if(freq < LFlim)
			avg = LFavg;
		else
			avg = HFavg;
		init_EXT(FALL);
		ADC_Cmd(ADC1, ENABLE);
		reset_AD9833();
	}//End of while(1)
}//End of main

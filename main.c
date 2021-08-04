/*
 * TODO: Fazer verificação no sysclock
 *
 * TODO: Fazer a calibração
 *
 * TODO: Valor de res
 */

#include <math.h>//Raiz e power
#include <stm32f10x.h>
#include "stm32f10x_conf.h"
#include "utils.h"

//A frequencia maxima para esse programa é 125kHz
//frequencia inicial = 1 Hz
//frequencia final   = 100k Hz

#define NPTS 101	//numero de pontos = 101

uint16_t data = 0;		//Valor de conversão do ADC
uint8_t DMA_flag = 0;		//Flag que a interrupção do DMA ativa
uint8_t count = 0;		//Quantas vezes foi feita as mesmas medidas
uint8_t wavePoint = 1;	//Ponto da onda senoidal estamos analisando
uint8_t nptsA = 0;	//numero do ponto atual


long RealVar = 0;
long ImagVar = 0;
long Real[NPTS];	//Todos os valores reais da impedancia
long Imag[NPTS];	//Todos os valores imaginarios da impedancia
long Zmodulo[NPTS];	//Valores da impedancia no final
double freqA[NPTS];	//Valores da frequencia para cada ponto


long RealOpen[101] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
long ImagOpen[101] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
const uint16_t res = 1;

//Para calcular o passo para cada frequencia, para que ele seja feito de uma
//forma logaritma, é feito  seguinte calculo:
//10^((log10(freqF) - log10(freqI))/n° pts)
//frequence increase, power(10,(log10(100000/1)/101))
float fincr = 1.12074;
double freq  = 1;//Essa já é a frequencia inicial

//É feito uma media entre os valores obtidos para cada frequencia, entretanto
//para frequencia pequenas (LFavg Low Frequency average) demora muito fazer 
//muitas medidas pelo periodo ser menor, logo ele só começa a fazer as HFavg
//(High Frequency average) depois do LFlim (Low Frequency limit)
//* O operador ">>" é igual a dividir por 2^(numero usado no operador)
const uint16_t HFavg = 128;
const uint16_t LFavg = HFavg >> 2;
uint16_t avg = LFavg;
const uint16_t LFlim = 20;
const uint32_t HFlim = 100000;

int main(void) {
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);//Desliga o led builtin
	init_Clock();
	init_GPIO();
	init_ADC(freq);
	init_DMA();
	init_NVIC();
	init_SPI();
	init_AD9833(freq);
	init_EXT(FALL);

	freqA[nptsA] = freq;
	for(int i = 0; i < NPTS; i++) {
			Real[i] = 0;
			Imag[i] = 0;
	}
	while(1) {
		while(count < avg){
			while(1) {
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
							Real[nptsA] += RealVar/NPTS;
							Imag[nptsA] += ImagVar/NPTS;
							wavePoint = 1;
							break;
					}//End os switch
					break;//Para sair do while(1)
				}//End of if(DMA_flag)
			}//End of while(1)
			DMA_flag = 0;
			count++;
		}//End of while(count < avg)
		ADC_Cmd(ADC1, DISABLE);
		count = 0;
		nptsA++;
		if(nptsA >= NPTS){//Aqui acabou 
			sleep_AD9833();
			//Calculo da impedancia para cada frequencia
			for(int i = 0; i<NPTS; i++) {
				long var = res/(pow((Real[i]+RealOpen[i]),2)+pow((Imag[i] + ImagOpen[i]),2));
				long Zimag = Imag[i]*(Real[i]+RealOpen[i])-Real[i]*(Imag[i]+ImagOpen[i]);
				Zimag *= var;
				long Zreal = Real[i]*(Real[i]+RealOpen[i])-Imag[i]*(Imag[i]+ImagOpen[i]);
				Zreal *= var;
				Zmodulo[i] = sqrt(pow(Zimag,2) + pow(Zreal,2));
			}
			GPIO_SetBits(GPIOC, GPIO_Pin_13);//Liga o Led builtin
			while(1){__NOP();}
		}
		freq *= fincr;
		freqA[nptsA] = freq;
		sampleTime(freq);
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

 /*
 * TODO: Fazer a calibração

 * TODO: Usar como base o exemplo do Apply Signal
 *
 * TODO: Calculuar os sampletimes dnv
 *
 * TODO: Valor de res
 *
 * TODO: Desligar todos os perifericos no fim
 * 
 * ADC1    <-->  PA0 
 * EXTI    <-->  PA1 
 * SCK     <-->  PA5 
 * MOSI    <-->  PA7 
 * FSYNCA  <-->  PB0 
 * FSYNCB  <-->  PB1 
 */

//A frequencia maxima para esse programa é 125kHz
//frequencia inicial = 1 Hz
//frequencia final   = 100k Hz

#include <math.h>		//Raiz e power
#include <STM32ADC.h>	//ADC
#include <SPI.h>		//Comunicação com AD9833
#include <AD9833.h>		//Gerador de sinais

/*******************************************************************/
/*                         Variaveis                               */
/*******************************************************************/
#define SPI1_FSYNCA_PIN PB0
#define SPI1_FSYNCB_PIN PB1

#define NPTS 101  //numero de pontos = 101

enum {
	RISE = 0,
	FALL
};

uint8 adc_pin = PA0;	//Para usa o endenreço de PA0
uint16 data = 0;    	//Valor de conversão do ADC
uint8_t ADC_flag = 0;	//Para usa o endenreço de PA0
uint8_t DMA_flag = 0; 	//Flag que a interrupção do DMA ativa
uint8_t count = 0;    	//Quantas vezes foi feita as mesmas medidas
uint8_t wavePoint = 1;  //Ponto da onda senoidal estamos analisando
uint8_t nptsA = 0;    	//numero do ponto atual

long RealVar = 0; 	//Variavel do valor real
long ImagVar = 0; 	//Variavel do valor imaginario
long Real[NPTS];  	//Todos os valores reais da tensão
long Imag[NPTS];  	//Todos os valores imaginarios da tensão
long Zmodulo[NPTS]; //Valores da impedancia no final
double freqA[NPTS]; //Valores da frequencia para cada ponto

//Valores da calibração que são fixos do sistema. São obtidos previamente
//em uma medição com curto-circuito
long RealOpen[101] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
long ImagOpen[101] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
const uint16_t res = 1;//Resistencia fixa

//Para calcular o passo de cada frequencia considerando que sera feito 
//de uma forma logaritimica, é feito  seguinte calculo:
//10^((log10(freqF) - log10(freqI))/n° pts)
//frequence increase =  power(10,(log10(100000/1)/101))
float fincr = 1.12074;
double freq  = 1;//Variavel da frequencia já com o seu valor inicial

//É feito uma media entre os valores obtidos para cada frequencia, entretanto
//para frequencia pequenas (LFavg Low Frequency average) demora muito fazer 
//muitas medidas pelo periodo ser maior, logo ele só começa a fazer as HFavg
//(High Frequency average) depois do LFlim (Low Frequency limit)
const uint16_t HFavg = 128;
const uint16_t LFavg = HFavg >> 2;//*
uint16_t avg = LFavg;
const uint16_t LFlim = 20;
const uint32_t HFlim = 100000;
//* O operador ">>" é igual a dividir por 2^(numero usado no operador)

/*******************************************************************/
/*                           Funções                               */
/*******************************************************************/

STM32ADC myADC(ADC1);

AD9833 geradorA(FSYNC_A_PIN);
AD9833 geradorB(FSYNC_B_PIN);

adc_smp_rate sampleTime(double freq);
void init_EXT(int externalTrigger);
void exti_Interrupt();
void dma_Interrupt();
void ad9833_reset();

/*******************************************************************/
/*                          void setup                             */
/*******************************************************************/

void setup() {
	for(int i = 0; i < NPTS; i++) {
		Real[i] = 0;
		Imag[i] = 0;
	}
	while(!Serial);delay(10);
	Serial.print("Setup...");
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);//Led OFF

	/* Configure SPI1 ------------------------------------------------------------------*/
	pinMode(SPI1_FSYNCA_PIN, OUTPUT);
	pinMode(SPI1_FSYNCB_PIN, OUTPUT);
	
	/* Configure ADC1 ------------------------------------------------------------------*/
	rcc_set_prescaler(RCC_PRESCALER_AHB, RCC_AHB_SYSCLK_DIV_1);//24MHz
	rcc_set_prescaler(RCC_PRESCALER_APB2, RCC_APB2_HCLK_DIV_1);//24Mhz
	adc_set_prescaler(ADC_PRE_PCLK2_DIV_2);//12MHz (Maximo do ADC)

	myADC.setPins(&adc_pin, 1);
  	myADC.setSampleRate(sampleTime(freq));
	myADC.setDMA(&data, 1,(DMA_TRNS_CMPLT | DMA_CIRC_MODE), dma_Interrupt);
	myADC.calibrate();
	init_EXT(FALL);
	
	/* Configure AD9833 ----------------------------------------------------------------*/
	geradorA.Begin();
	geradorA.ApplySignal(SINE_WAVE,REG0,freq);
	geradorB.Begin();
	geradorB.ApplySignal(SQUARE_WAVE,REG0,freq*2);

	geradorA.EnableOutput(true);
	geradorB.EnableOutput(true);
	
	Serial.println("done.");
}

/*******************************************************************/
/*                         void loop                               */
/*******************************************************************/
 
void loop() {
	ADC_flag = 1;
	while(1) {
		while(count < avg){
			while(1) {
				if(DMA_flag) {
					switch(wavePoint) {
						case 1://Sample at falling slope of sinus
							ImagVar = -((data)>>1);
							init_EXT(RISE);
							wavePoint++;
							break;
						case 2://Sample at negative peak of sinus
							RealVar = -((data)>>1);
							init_EXT(FALL);
							wavePoint++;
							break;
						case 3://Sample at falling slope of sinus
							ImagVar += ((data)>>1);
							init_EXT(RISE);
							wavePoint++;
							break;
						case 4://Sample at positive peak of sinus
							RealVar += ((data)>>1);
							wavePoint++;
							break;
						case 5://End of Sinus
							Real[nptsA] += RealVar/NPTS;
							Imag[nptsA] += ImagVar/NPTS;
							break;
					}//End os switch
					DMA_flag = 0;
					if(wavePoint == 5) {
						wavePoint = 1;
						break;//Para sair do segundo while(1)
					}
				}//End of if(DMA_flag)
			}//End of second while(1)
		count++;
		}//End of while(count < avg)
	ADC_flag = 0;
	Serial.print("Frequencia: ");
	Serial.print(freq);
	Serial.print("\tTensão real: ");
	Serial.print(Real[nptsA]);
	Serial.print("\tTensão Imaginaria: ");
	Serial.println(Imag[nptsA]);
	count = 0;
	freqA[nptsA] = freq;
	nptsA++;
	if(nptsA >= NPTS){//Aqui acabou 
		geradorA.SleepMode();
		geradorB.SleepMode();
		//Calculo da impedancia para cada frequencia
		for(int i = 0; i<NPTS; i++) {
			long var = res/(pow((Real[i]+RealOpen[i]),2)+pow((Imag[i] + ImagOpen[i]),2));
			long Zimag = Imag[i]*(Real[i]+RealOpen[i])-Real[i]*(Imag[i]+ImagOpen[i]);
			Zimag *= var;
			long Zreal = Real[i]*(Real[i]+RealOpen[i])-Imag[i]*(Imag[i]+ImagOpen[i]);
			Zreal *= var;
			Zmodulo[i] = sqrt(pow(Zimag,2) + pow(Zreal,2));
			
			Serial.print("Frequencia: ");
			Serial.print(freqA[i]);
			Serial.print("\tImpedancia: ");
			Serial.println(Zmodulo[i]);
		}
		while(1);
	}
	freq *= fincr;
	freqA[nptsA] = freq;
  	myADC.setSampleRate(sampleTime(freq));
	geradorA.SetFrequency (REG0,freq);
	geradorB.SetFrequency (REG0,2*freq);
	if(freq < LFlim)
		avg = LFavg;
	else
		avg = HFavg;
	init_EXT(FALL);
	//geradorA.Reset();
	//geradorB.Reset();
	ad9833_reset();
	ADC_flag = 1;
	}//End of while(1)
}//End of main

/*******************************************************************/
/*               Funções de configuração do STM                    */
/*******************************************************************/
/*
 * Sampletime altera o numero de ciclos utilizados para fazer a amostra da
 *  tensão, quanto maior, maior o tempo e a qualidade da conversão. Quanto
 *  maior a frequencia mais é diminuido os ciclos de amostra.
 */

adc_smp_rate sampleTime(double freq) {
	if(freq < 11363){
  		return ADC_SMPR_239_5;
  	}
  	else if(freq < 31250){
  		return ADC_SMPR_71_5;
  	}
  	else if(freq < 37878){
  		return ADC_SMPR_55_5;
  	}
  	else if(freq < 45454){
  		return ADC_SMPR_41_5;
  	}
  	else if(freq < 56612){
  		return ADC_SMPR_28_5;
  	}
  	else if(freq < 79113){
  		return ADC_SMPR_13_5;
  	}
  	else if(freq < 96153){
  		return ADC_SMPR_7_5;
  	}
  	else{
  		return ADC_SMPR_1_5;
  	}
}

void ad9833_reset() {
	//pinMode 
	digitalWrite(PB0,LOW);
	digitalWrite(PB1,LOW);
	SPI.transfer(highByte(0x0100))
	SPI.transfer(lowByte(0x0100))
	delay(15);
	SPI.transfer(highByte(0x000))
	SPI.transfer(lowByte(0x000))
	digitalWrite(PB0,HIGH);
	digitalWrite(PB1,HIGH);
}

/*
 * External event on PA00 RISE or FALL
 *
 */
void init_EXT( int externalTrigger ) {
	if(externalTrigger == RISE)
		exti_attach_interrupt(EXTI1, EXTI_PA, &exti_Interrupt, EXTI_RISING);
	else
		exti_attach_interrupt(EXTI1, EXTI_PA, &exti_Interrupt, EXTI_FALLING);
}

void exti_Interrupt() {
	if(ADC_flag) {
		myADC.startConversion();
	}
}
void dma_Interrupt() {
	DMA_flag = 1;
}

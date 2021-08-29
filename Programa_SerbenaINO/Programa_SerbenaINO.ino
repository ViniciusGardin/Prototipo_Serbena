 /*
 * TODO: Fazer a calibração
 *
 * TODO: Calculuar os sampletimes dnv
 *
 * TODO: Valor de res
 *
 * TODO: Desligar todos os perifericos no fim
 * 
 * ADC1    <-->  PA0 
 * EXTI    <-->  PA1 
 * FSYNCA  <-->  PB0 
 * FSYNCB  <-->  PB1 
 * SCK     <-->  PA5 
 * MISO    <-->  PA6 
 * MOSI    <-->  PA7 
 */

//A frequencia maxima para esse programa é 125kHz
//frequencia inicial = 1 Hz
//frequencia final   = 100k Hz

#include <math.h>//Raiz e power
#include <SPI.h>//Comunicação com AD
#include <STM32ADC.h>

/*******************************************************************/
/*                         Variaveis                               */
/*******************************************************************/
#define SPI1_FSYNCA_PIN PB0
#define SPI1_FSYNCB_PIN PB1

//Comandos para o AD9833
#define SINE_WAVE         0x2000
#define TRIANGLE_WAVE     0x2002
#define SQUARE_WAVE       0x2028
#define HALF_SQUARE_WAVE  0x2020
#define RESET_CMD         0x0100
#define NORMAL_CMD        0x000 
#define PHASE_WRITE_CMD   0xC000
#define FREQ_WRITE_CMD    0x2100
#define SLEEP             0x180 
#define REG0              0x4000//Registrador 0

#define NPTS 101  //numero de pontos = 101

uint8_t adc_pin = PA0;	//Para usa o endenreço de PA0
uint8_t ADC_flag = 0;	//Para usa o endenreço de PA0
uint8_t DMA_flag = 0; 	//Flag que a interrupção do DMA ativa
uint8_t count = 0;    	//Quantas vezes foi feita as mesmas medidas
uint8_t wavePoint = 1;  //Ponto da onda senoidal estamos analisando
uint8_t nptsA = 0;    	//numero do ponto atual
uint16_t data = 0;    	//Valor de conversão do ADC

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

enum {
	RISE = 0,
	FALL
};

void reset_AD9833();
void init_AD9833(float frequency);
void setFrequency(float frequency);
void sleep_AD9833();
void writeRegisterA(uint16_t command);
void writeRegisterB(uint16_t command);

adc_smp_rate sampleTime(double freq);
void init_EXT(int externalTrigger);
void exti_Interrupt();
void dma_Interrupt();

/*******************************************************************/
/*                          void setup                             */
/*******************************************************************/
STM32ADC myADC(ADC1);

void setup() {
	Serial.begin(115200); 
	Serial.print("Setup...");

	/* Configure SPI1 ------------------------------------------------------------------*/
	SPI.begin(); //Initialize the SPI_1 port.
	SPI.setBitOrder(MSBFIRST); // Set the SPI_1 bit order
	SPI.setDataMode(SPI_MODE0); //Set the  SPI_2 data mode 0
	SPI.setClockDivider(SPI_CLOCK_DIV16);      // Slow speed (72 / 16 = 4.5 MHz SPI_1 speed)
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
	
	Serial.println("done.");
}

/*******************************************************************/
/*                         void loop                               */
/*******************************************************************/
 
void loop() {
	//É iniciado os valores de cada vetor
	freqA[nptsA] = freq;
	for(int i = 0; i < NPTS; i++) {
		Real[i] = 0;
		Imag[i] = 0;
	}
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
					if(wavePoint == 5)
						break;//Para sair do segundo while(1)
				}//End of if(DMA_flag)
			}//End of second while(1)
		wavePoint = 1;
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
			
			Serial.print("Frequencia: ");
			Serial.print(freqA[i]);
			Serial.print("\tImpedancia: ");
			Serial.println(Zmodulo[i]);
		}
		while(1){}
	}
	freq *= fincr;
	freqA[nptsA] = freq;
  	myADC.setSampleRate(sampleTime(freq));
	setFrequency(freq);
	if(freq < LFlim)
		avg = LFavg;
	else
		avg = HFavg;
	init_EXT(FALL);
	reset_AD9833();
	ADC_flag = 1;
	}//End of while(1)
}//End of main

/*******************************************************************/
/*                     Funções do AD9833                           */
/*******************************************************************/

/*
 * Utilizado para resetar o AD9833 ao ligar (recomendação do manual) 
 * e ao mudar a frequencia dos dois AD9833, para que assim eles 
 * estejam sincronizados. Ao resetar ele não altera os valores de
 * frequencia e fase.
 */
void reset_AD9833() {
	digitalWrite(SPI1_FSYNCA_PIN, LOW);
  	digitalWrite(SPI1_FSYNCB_PIN, LOW);

  	SPI.transfer(RESET_CMD);
  	delay(15);
  	SPI.transfer(NORMAL_CMD);

  	digitalWrite(SPI1_FSYNCA_PIN, HIGH);
  	digitalWrite(SPI1_FSYNCB_PIN, HIGH);
}

/*
 * Inicia o AD9833. Frequency in Hz
 */
void init_AD9833(float frequency) {
	reset_AD9833();
  	writeRegisterA(SINE_WAVE);
  	writeRegisterB(SQUARE_WAVE);
  	setFrequency(frequency);
}

/*
 *  Set the specified frequency register with the frequency (in Hz)
 */
void setFrequency(float frequency) {
	if(frequency > 125e3)
  		frequency = 125e3;
  	if(frequency < 0.0) 
  		frequency = 0.0;
  	
  	uint32_t freqWord = (uint32_t)(frequency * 10.73741824);
  	uint16_t upper14 = (uint16_t)((freqWord & 0xFFFC000) >> 14), 
  	lower14 = (uint16_t)(freqWord & 0x3FFF);

  	lower14 |= REG0;
  	upper14 |= REG0;   

  	writeRegisterA(FREQ_WRITE_CMD); 
  	writeRegisterA(lower14);      //Write lower 14 bits to AD9833
  	writeRegisterA(upper14);      //Write upper 14 bits to AD9833
  	writeRegisterA(PHASE_WRITE_CMD);  //Phase = 0

  	freqWord = (uint32_t)(2*frequency * 10.73741824);
  	upper14 = (int16_t)((freqWord & 0xFFFC000) >> 14);
  	lower14 = (int16_t)(freqWord & 0x3FFF);

  	lower14 |= REG0;
  	upper14 |= REG0;   

  	//Calculo para correção da fase
  	int LFcor = 1;
  	if (frequency > LFlim)
  	  LFcor = HFavg*7;
  	uint16_t phas_corr = (uint16_t)(frequency / 256 * LFcor);   

  	writeRegisterB(FREQ_WRITE_CMD); 
  	writeRegisterB(lower14);          //Write lower 14 bits to AD9833
  	writeRegisterB(upper14);          //Write upper 14 bits to AD9833
  	writeRegisterB(PHASE_WRITE_CMD + phas_corr);//Phase correction
}

void sleep_AD9833() {
    writeRegisterA(SLEEP); 
    writeRegisterB(SLEEP); 
}

void writeRegisterA( uint16_t command ) {
	digitalWrite(SPI1_FSYNCA_PIN, LOW);
  	SPI.transfer(command);
  	digitalWrite(SPI1_FSYNCA_PIN, HIGH);
}

void writeRegisterB( uint16_t command ) {
	digitalWrite(SPI1_FSYNCB_PIN, LOW);
  	SPI.transfer(command);
  	digitalWrite(SPI1_FSYNCB_PIN, HIGH);
}

/*******************************************************************/
/*               Funções de configuração do STM                    */
/*******************************************************************/
/*
 * Sampletime altera o numero de ciclos utilizados para fazer a amostra da
 *  tensão, quanto maior, maior o tempo e a qualidade da conversão. Quanto
 *  maior a frequencia mais é diminuido os ciclos de amostra.
 */

adc_smp_rate sampleTime(double freq) {
	if(freq < 13157){
  		return ADC_SMPR_239_5;
  	}
  	else if(freq < 35714){
  		return ADC_SMPR_71_5;
  	}
  	else if(freq < 42682){
  		return ADC_SMPR_55_5;
  	}
  	else if(freq < 51470){
  		return ADC_SMPR_41_5;
  	}
  	else if(freq < 63636){
  		return ADC_SMPR_28_5;
  	}
  	else if(freq < 87500){
  		return ADC_SMPR_13_5;
  	}
  	else if(freq < 102940){
  		return ADC_SMPR_7_5;
  	}
  	else{
  		return ADC_SMPR_1_5;
  	}
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

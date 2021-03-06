/*
 *
 * ADC - PA0
 * EXT - PA1
 * CLK - PA5
 * MOSI - PA7
 * FSYNCA - PB0
 * FSYNCB - PB1
 *
 * Resistor (220)
 * Potenciometro
 * Push-button
 *
 * EXT           <-> Push-button
 * Resistor GND  <-> Push-button(na reta com o EXT)
 * 3.3V          <-> Push-button(em diagonal com o EXT)
 * Potenciometro <-> ADC
 */
#include <STM32ADC.h>
#include <AD9833.h>

/*******************************************************************/
/*                         Variaveis                               */
/*******************************************************************/
#define FSYNC_A_PIN PB0
#define FSYNC_B_PIN PB1

int i;
uint8 adc_pin = PA0;	//Para usa o endenreço de PA0
uint16 data = 0;    	//Valor de conversão do ADC
uint8_t ADC_flag = 0;	//Para usa o endenreço de PA0
uint8_t wavePoint = 1;	//Para usa o endenreço de PA0
uint8_t DMA_flag = 0; 	//Flag que a interrupção do DMA ativa
double point[] = {0,0};

enum {
	RISE = 0,
	FALL
};
/*******************************************************************/
/*                           Funções                               */
/*******************************************************************/

STM32ADC myADC(ADC1);
AD9833 geradorA(FSYNC_A_PIN);
//AD9833 geradorB(FSYNC_B_PIN);

adc_smp_rate sampleTime(double freq);
void init_EXT(int externalTrigger);
void exti_Interrupt();
void dma_Interrupt();

/*******************************************************************/
/*                          void setup                             */
/*******************************************************************/

void setup() {
	while(!Serial);delay(10);
	Serial.print("Setup...");
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);//Led OFF

	/* Configure ADC1 ------------------------------------------------------------------*/
	rcc_set_prescaler(RCC_PRESCALER_AHB, RCC_AHB_SYSCLK_DIV_1);//24MHz
	rcc_set_prescaler(RCC_PRESCALER_APB2, RCC_APB2_HCLK_DIV_1);//24Mhz
	adc_set_prescaler(ADC_PRE_PCLK2_DIV_2);//12MHz (Maximo do ADC)

	myADC.setPins(&adc_pin, 1);
  	myADC.setSampleRate(sampleTime(5000));
	myADC.setDMA(&data, 1,(DMA_TRNS_CMPLT | DMA_CIRC_MODE), dma_Interrupt);
	myADC.calibrate();
	init_EXT(FALL);
	point[0] = 0;
	point[1] = 0;

	/* Configure AD9833 ----------------------------------------------------------------*/
	geradorA.Begin();
	//geradorA.ApplySignal(SINE_WAVE,REG0,freq);
	geradorA.ApplySignal(SQUARE_WAVE,REG0,1);
	geradorA.EnableOutput(true);
//	geradorB.Begin();
//	geradorB.ApplySignal(SQUARE_WAVE,REG0,1);
//	geradorB.EnableOutput(true);
	
	Serial.println("done.");
}

/*******************************************************************/
/*                         void loop                               */
/*******************************************************************/
 
void loop() {
	ADC_flag = 1;
	while(1){
		if(DMA_flag) {
			switch(wavePoint) {
				case 1://Sample at falling slope of sinus
					point[0] = data;
					init_EXT(RISE);
					wavePoint++;
					break;
				case 2://Sample at negative peak of sinus
					point[1] = data;
					init_EXT(FALL);
					wavePoint++;
					break;
			}//End of switch
			DMA_flag = 0;
		}//End of if(DMA_flag)
		if(wavePoint == 3) {
			wavePoint = 1;
			break;
		}
	}
	ADC_flag = 0;
	Serial.print("Subida: ");
	Serial.print(point[0]);
	Serial.print("\tDescida: ");
	Serial.println(point[1]);
  	myADC.setSampleRate(sampleTime(20000));
	init_EXT(FALL);
	i++;
	if(i >= 1000)
		geradorA.SleepMode(true);
}//End of loop

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

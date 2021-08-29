#include <SPI.h>//Comunicação com AD

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

/*******************************************************************/
/*                           Funções                               */
/*******************************************************************/

void reset_AD9833();
void init_AD9833(float frequency);
void setFrequency(float frequency);
void sleep_AD9833();
void writeRegisterA(uint16_t command);
void writeRegisterB(uint16_t command);

/*******************************************************************/
/*                          void setup                             */
/*******************************************************************/

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

	init_AD9833(1);
	
	Serial.println("done.");
}

/*******************************************************************/
/*                         void loop                               */
/*******************************************************************/
 
void loop() {
	delay(10000);
	sleep_AD9833();
	delay(5000);
	reset_AD9833();
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

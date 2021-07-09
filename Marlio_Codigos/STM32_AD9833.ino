/*
AD9833 Waveform Module vwlowen.co.uk
    Using the first SPI port (SPI_1)
    SS    <-->  PA4 <-->  BOARD_SPI1_NSS_PIN
    SCK   <-->  PA5 <-->  BOARD_SPI1_SCK_PIN
    MISO  <-->  PA6 <-->  BOARD_SPI1_MISO_PIN
    MOSI  <-->  PA7 <-->  BOARD_SPI1_MOSI_PIN

    Using the second SPI port (SPI_2)
    SS    <-->  PB12 <-->  BOARD_SPI2_NSS_PIN
    SCK   <-->  PB13 <-->  BOARD_SPI2_SCK_PIN
    MISO  <-->  PB14 <-->  BOARD_SPI2_MISO_PIN
    MOSI  <-->  PB15 <-->  BOARD_SPI2_MOSI_PIN

*/

#include <SPI.h>
SPIClass SPI_2(2); //Create an instance of the SPI Class called SPI_2 that uses the 2nd SPI Port
   
const int SINE = 0x2000;                    // Define AD9833's waveform register value.
const int TRIANGLE = 0x2002;                // When we update the frequency, we need to
const int SQUARE = 0x2028;                  // define the waveform when we end writing.    
const int SQUARE2 = 0x2020;   // square/2

int wave = 0;
//int waveType = SINE;
int waveType = TRIANGLE;
int wavePin = 7;

const float refFreq = 25000000.0;           // On-board crystal reference frequency

//#define FSYNC PA15                       // Standard SPI pins for the AD9833 waveform generator.
#define FSYNC PB12
const int CLK = 13;                         // CLK and DATA pins are shared with the TFT display.
const int DATA = 11;

unsigned long freq = 1000;               // Set initial frequency.
String inputString = "";

//ADC read
const int AnalogIn = PA5;
int ADread = 0;
int count = 0;

void setup() { 

  // Setup SPI 1: SPI1 works on APB2 (72 MHz)
//  SPI.begin(); //Initialize the SPI_1 port.
//  SPI.setBitOrder(MSBFIRST); // Set the SPI_1 bit order
//    // AD9833 uses SPI MODE2 
//  SPI.setDataMode(SPI_MODE2);     
//  SPI.setClockDivider(SPI_CLOCK_DIV8);      // clock speed =72 / 8 = 9 MHz (SPI_1 speed)
//  pinMode(FSYNC, OUTPUT);

  // Setup SPI 2: SPI2 works on APB1 (36 MHz)
  SPI_2.begin(); //Initialize the SPI_2 port.
  SPI_2.setBitOrder(MSBFIRST); // Set the SPI_2 bit order
  SPI_2.setDataMode(SPI_MODE2); //Set the  SPI_2 data mode 0
  SPI_2.setClockDivider(SPI_CLOCK_DIV2);   // clock speed =36 / 2 = 18 MHz (SPI_2 speed)
  pinMode(FSYNC, OUTPUT);

  
  AD9833reset();                                   // Reset AD9833 module after power-up.
  delay(1);
  
  AD9833setFrequency(freq);                  // Set the frequency output
  AD9833setWaveform(SINE);                  // Set the Wave output
   inputString.reserve(10);

  Serial.begin(115200);  //begins serial port at 1 Mbps
  
  pinMode(PA5, INPUT);//defines pin 6 as A/D input
  pinMode(PA4, OUTPUT); //put to GND analog input (PA5) neigboring pins to minimize noise
  pinMode(PA6, OUTPUT);
  digitalWrite(PA4, LOW);
  digitalWrite(PA6, LOW);

 }


void loop() {

  Serial.write(analogRead(AnalogIn)>>2); //envia para a serial a leitura do AD em 8 bits (fs ~ 78 kS/s)
//  count += 1;
//  if (count == 1000) {
//    WriteRegister(SINE);
//  }
//  if (count == 2000) {
//    WriteRegister(SQUARE);
//    count = 0;
// }        


    if ( Serial.available() ) {

//        freq = Serial.parseInt();
//         freq = constrain(freq, 1, 100);
//         }
//         if (Serial.read() == '\n' || Serial.read() == '\r') {
//         }
    while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      freq = inputString.toInt(); 
      inputString = "";
       }
    }

    AD9833setFrequency(freq); //send freq to AD9833
    return; 
  }

}


//Functions

// AD9833 documentation advises a 'Reset' on first applying power.
void AD9833reset() {
  WriteRegister(0x100);   // Write '1' to AD9833 Control register bit D8 for Reset
  delayMicroseconds(2); 
  WriteRegister(0x000);   // Write '0' to AD9833 Control register bit D8 for normal operation
}

// Set the waveform registers in the AD9833.
void AD9833setWaveform(int Waveform) {
  WriteRegister(Waveform);             // Exit & Reset to SINE, SQUARE or TRIANGLE
}

// Set the frequency registers in the AD9833.
void AD9833setFrequency(long frequency) {

  long FreqWord = (frequency * pow(2, 28)) / refFreq;

  int MSB = (int)((FreqWord & 0xFFFC000) >> 14);    //Only lower 14 bits are used for data
  int LSB = (int)(FreqWord & 0x3FFF);
  
  //Set control bits 15 and 14 to 0 and 1, respectively, for frequency register 0
  LSB |= 0x4000;
  MSB |= 0x4000; 
  
  WriteRegister(0x2100);   
  WriteRegister(LSB);                  // Write lower 16 bits to AD9833 registers
  WriteRegister(MSB);                  // Write upper 16 bits to AD9833 registers.
  WriteRegister(0xC000);               // Phase register
  WriteRegister(SINE);             // Exit & Reset to SINE, SQUARE or TRIANGLE

}

void WriteRegister(int dat) { 
   
  digitalWrite(FSYNC, LOW);           // Set FSYNC low before writing to AD9833 registers
  SPI_2.transfer16(dat);              //transfer 16 bits
  digitalWrite(FSYNC, HIGH);          //Write done. Set FSYNC high
}

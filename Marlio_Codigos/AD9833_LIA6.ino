/*
VNA with dual AD9833 + Arduino Nano
Version 6: Low frequency (LFlim Hz) sampling 'avg' times at each phase to get the result in a single period
*/

#include <SPI.h>

//ADC clock frequency registers
#ifndef cbi
	#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
	#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//AD9833 parameters
const int SINE = 0x2000;                    // Define AD9833's waveform register value.
const int TRIANGLE = 0x2002;                // When we update the frequency, we need to
const int SQUARE = 0x2028;                  // define the waveform when we end writing.    
const int SQUARE2 = 0x2020;   // square/2
int wave = 0;
int waveType = TRIANGLE;
int wavePin = 7;
const long refFreq = 25000000;           // On-board crystal reference frequency
const int FSYNCa = 10;                       // Standard SPI pins for the AD9833a waveform generator.
const int FSYNCb = 9;                       // Standard SPI pins for the AD9833b waveform generator.
const int CLK = 13;                         // CLK and DATA pins are shared with the TFT display.
const int DATA = 11;

//Frequency parameters
float freqi = 10;                 // Set initial frequency.
float freqf = 19000;               // Set final frequency.

int npts = 101;
double frel = log10(freqf/freqi);
double fincr = pow(10,(frel/npts));
float freq = freqi;               // Set frequency equal to initial freqi
const byte LFlim = 20;
const int HFlim = 19000;

String inputString = "";
byte IntMode = 1;    //Interruption Mode
byte n = 0;
byte count = 0;
byte i = 0;
byte HFavg = 128; // sample average number /4 for IntMode = 1 (CHANGE) or /2 for IntMode = 2 or 3 (RISING or FALLING) 
byte LFavg = HFavg >> 2;
byte avg = HFavg;
unsigned int S[4]= {0};  //sampling variables 
long Real=0, Imag=0;



void setup() { 
   //cbi (TIMSK0, TOIE0);
   TIMSK0 = 0;  // Prevent further TIMER 0 interrupts    
   TIMSK2 = 0;  // Prevent further TIMER 2 interrupts
   sei();  // Enable global interrupts

   inputString.reserve(10);

   pinMode(6, INPUT);//defines pin 6 as A/D input
   pinMode(7, INPUT);//defines pin 7 as A/D input
   pinMode(9, OUTPUT);//defines pin 9 as output FSYNCb
   pinMode(8, OUTPUT);//defines pin 8 as output for debugging purposes
   digitalWrite(8, 0);

   // Initialize SPI MODE2 (CPOL=1, CPHA=0)
  SPI.begin();
  SPISettings settings(16000000, MSBFIRST, SPI_MODE2); //freq=16 MHz;
  SPI.beginTransaction(settings);  
  Serial.begin(1000000);  //begins serial port at 1 Mbps
 //ADC settings
  /* change internal ADC clock frequency*******
  sbi: set bit=1; cbi: clear bit=0;
  ADCclock=Mainclock/(16^ADPS2*4^ADPS1*2^ADPS0)
  Default: ADPS2=ADPS1=ADPS0=1 => ADCclock=Mainclock/128=9.8 kHz (for Mainclock=16 MHz)
  ADPS2=1, ADPS1=0, ADPS0=0 => ADCclock=Mainclock/(16*1*1)=76.8 kHz (for Mainclock=16 MHz)
  ADPS2=0, ADPS1=1, ADPS0=1 => ADCclock=Mainclock/(1*4*2)=153.6 kHz (for Mainclock=16 MHz)
  ADPS2=0, ADPS1=1, ADPS0=0 => ADCclock=Mainclock/(1*4*1)=307.2 kHz (for Mainclock=16 MHz)
  */
// enable ADC + ADC Auto Trigger Enable + enable ADC interrupt vector + ADCclock=307.2 kHz
  ADCSRA = (1<<ADEN) + (1<<ADATE) + (1<<ADIE) + 2; 
  ADCSRB = 0b00000010;    //Auto trigger at each external interruption "0" (pin D2)
 //ADMUX = 0b01100111; //set ADref=AVcc,result left justified,  set channel 7
  ADMUX = 0b01000111; //set ADref=AVcc,result right justified,  set channel 7

 //External Interruption Setup for AD conversion start
//0 low level of INT0 generates an interrupt request.
//1 Any logical change on INT0 generates an interrupt request. CHANGE: 
//2 falling edge of INT0 generates an interrupt request: FALLING
//3 rising edge of INT0 generates an interrupt request.: RISING

if (freq <= HFlim ){
    IntMode = 1; //Interrupt on edge change
    if (freq < LFlim){
          n = 0; //sample 'avg' times on every interruption
          avg = LFavg;
          //ADCSRA |= 1; //decrease ADC samplig frequency to half of 307.2 kHz
          ADCSRA |= (1<<ADPS2); //decrease ADC samplig frequency to 307.2 kHz/4
          ADCSRA &= ~(1 << ADPS1);
    }
    else
    	n = 3; //count 0, 1, 2 and 3; sample once every interruption
}
    
  if (freq > HFlim & n == 3){
    n = 1; //count 0 and 1
    IntMode = 2; //Interrupt on falling edge
    }
    Serial.print(IntMode); //send to serial  ADC sample
    Serial.print('\t');    //separator character
    Serial.print(i); //send to serial  ADC sample
    Serial.print('\t');    //separator character
    Serial.print(n); //send to serial  ADC sample
    Serial.print('\t'); 
    Serial.print(count); //send to serial  ADC sample
    Serial.print('\t');     
    Serial.print(avg); //send to serial  ADC sample
    Serial.print('\t'); 
    Serial.print(LFavg); //send to serial  ADC sample
    Serial.print('\t'); 
    Serial.print(freq); //send to serial  ADC sample
    Serial.print('\t');     
    Serial.println(fincr); //send to serial  ADC sample
   // Serial.print('\t'); 
     AD9833reset();                                   // Reset AD9833 module after power-up.
  AD9833asetFrequency(freq);                  // Set the frequency output
  AD9833asetWaveform(SINE);                  // Set the Wave output
  AD9833bsetFrequency(freq);                  // Set the frequency output
  AD9833bsetWaveform(SQUARE);                  // Set the Wave output
   
   count = 1;
   i = 0;
   memset(S, 0, sizeof(S)); //clear S[] vector
   AD9833reset(); // Reset AD9833 module 
   attachInterrupt(0, AD_sample, IntMode); //four interruptions per wave period
   ADCSRA |= (1 << ADSC);            //  Start the conversions

 }


//Main Loop

void loop() {

if (count >= avg) {
  ADCSRA &= ~(1 << ADIE);  // ADC interrupt Disable
  detachInterrupt(0);
  PORTB &= ~_BV(PB0);

if (n == 0){
//separator character
switch (i){
 case 1:  //sample at falling slope of sinus
  Imag = (-long(S[0])>>1);
//  Serial.print(count); //send to serial i 
//  Serial.print('\t');    

    Serial.print(freq,2); //send to serial current frequency 
    Serial.print('\t');    //separator character 
//    Serial.print(S[0]); //send to serial  ADC sample
//    Serial.print('\t');    //separator character
    memset(S, 0, sizeof(S)); //clear S[] vector
    attachInterrupt(0, AD_sample, IntMode); 
    ADCSRA |= (1 << ADIE); 
  break;
 case 2:  //sample at negative peak of sinus
  Real = (-long(S[0])>>1);
//    Serial.print(S[0]); //send to serial  ADC sample
//    Serial.print('\t');    //separator character
    memset(S, 0, sizeof(S)); //clear S[] vector
    attachInterrupt(0, AD_sample, IntMode); 
    ADCSRA |= (1 << ADIE); 
  break;
 case 3:  //sample at rising slope of sinus
  Imag += (long(S[0])>>1);
//    Serial.print(S[0]); //send to serial  ADC sample
//    Serial.print('\t');    //separator character
    memset(S, 0, sizeof(S)); //clear S[] vector
    attachInterrupt(0, AD_sample, IntMode); 
    ADCSRA |= (1 << ADIE); 
  break;
 case 4:  //sample at positive peak of sinus
  Real += (long(S[0])>>1);
//    Serial.println(S[0]); //send to serial  ADC sample
  Serial.print(Real); //send to serial Real part
  Serial.print('\t');
  Serial.println(Imag); //send to serial Imag part
   avg = 1;
   memset(S, 0, sizeof(S)); //clear S[] vector
   attachInterrupt(0, AD_sample, IntMode); 
   ADCSRA |= (1 << ADIE); 
  break;
  case 5:  //end of sinus 
    i = 0;
    freq *= fincr;
//  Serial.write(42);// ascii 42 = *
//  Serial.print(count); //send to serial i 
//  Serial.print('\t');  
 if (freq < LFlim){
      avg = LFavg;
  AD9833asetFrequency(freq);//send freq to AD9833
  AD9833bsetFrequency(freq); //send freq to AD9833
  EIFR |= (1 << INTF0); //clear Int0 flag: ESSENTIAL to start each new frequency 
  AD9833reset(); 
  attachInterrupt(0, AD_sample, IntMode);
  memset(S, 0, sizeof(S)); //clear S[] vector
  ADCSRA |= (1 << ADIE); 
 }
  break;
  }
     count = 0; 
}

if (n == 3){
    AD9833_sleep();
  Real = (long(S[0]) - long(S[2]))>>1;
  Imag = (long(S[3]) - long(S[1]))>>1;
//  Serial.print(i); //send to serial i 
//  Serial.print('\t');    //separator character
  Serial.print(freq,1); //send to serial current frequency
  Serial.print('\t');    //separator character
  Serial.print(Real); //send to serial Real part
  Serial.print('\t');
  Serial.println(Imag); //send to serial Imag part
//  Serial.print(S[0]); //send to serial 1st ADC sample
//  Serial.print('\t');
//  Serial.print(S[1]); //send to serial 2nd ADC sample
//  Serial.print('\t');
//  Serial.println(S[2]); //send to serial 3rd ADC sample
//  Serial.print('\t');
//  Serial.println(S[3]); //send to serial 4th ADC sample

  freq *= fincr;
  i = 0;
}

if (n == 1){
    AD9833_sleep();
switch (IntMode){
 case 2:  //Interrupt on falling edge
  Real = (long(S[0]) - long(S[1]))>>2;
  IntMode = 3;   //change to Interrupt on rising edge
  Serial.print(freq,1); //send to serial current frequency
  Serial.print('\t');    //separator character
  Serial.print(Real); //send to serial Real part
//  Serial.print('\t');
//  Serial.print(S[0]); //send to serial 1st ADC sample
//  Serial.print('\t');
//  Serial.print(S[1]); //send to serial 2nd ADC sample
//  Serial.print('\t');
  break;

 case 3:   //Interrupt on rising edge
   Imag = (long(S[0]) - long(S[1]))>>2;
   IntMode = 2;   //change to Interrupt on falling edge
   Serial.print('\t');
   Serial.println(Imag); //send to serial Imag part
//  Serial.print(S[0]); //send to serial 1st ADC sample
//  Serial.print('\t');
//  Serial.print(S[1]); //send to serial 2nd ADC sample
//  Serial.println('\t');   
   freq *= fincr;
   i = 0;
   break;
 }
}

  if (freq > freqf){
    freq = freqi;
    memset(S, 0, sizeof(S)); //clear S[] vector
    i = 0;
    
  if (freq <= HFlim ){
    IntMode = 1; //Interrupt on edge change
    if (freq < LFlim){
       n = 0; //sample 'avg' times on every interruption
       //ADCSRA |= 1; //decrease ADC samplig frequency to half of 307.2 kHz
       ADCSRA |= (1<<ADPS2); //decrease ADC samplig frequency to 307.2 kHz/4
       ADCSRA &= ~(1 << ADPS1);
       avg = LFavg;  
       count = 0;
       i = 0;    
       AD9833asetFrequency(freq);//send freq to AD9833
       AD9833bsetFrequency(freq); //send freq to AD9833
       EIFR |= (1 << INTF0); //clear Int0 flag: ESSENTIAL to start each new frequency 
       AD9833reset(); 
       attachInterrupt(0, AD_sample, IntMode);
       memset(S, 0, sizeof(S)); //clear S[] vector
       ADCSRA |= (1 << ADIE); 
       }
    else {n = 3;}//count 0, 1, 2 and 3; sample once every interruption
    }
    
  if (freq > HFlim & n == 3){
    n = 1; //count 0 and 1
    IntMode = 2; //Interrupt on falling edge
    }
 
 }//end of: if (freq > freqf)
 
   if (freq >= LFlim){ 
    n = 3;
    avg = HFavg; 
    }
  
  if (n > 0){
      //ADCSRA &= ~1; //increase ADC samplig frequency to 307.2 kHz
      ADCSRA |= (1<<ADPS1); //increase ADC samplig frequency to 307.2 kHz
      ADCSRA &= ~(1 << ADPS2);
      i = 0;
      AD9833asetFrequency(freq);//send freq to AD9833
      AD9833bsetFrequency(freq); //send freq to AD9833
      EIFR |= (1 << INTF0); //clear Int0 flag: ESSENTIAL to start each new frequency 
      attachInterrupt(0, AD_sample, IntMode);  
      AD9833reset(); 
      //_delay_us(10);
      count = 0;
      memset(S, 0, sizeof(S)); //clear S[] vector
      ADCSRA |= (1 << ADIE); 
          
  } //end of:if (n > 0)
 
} //end of: if (count >= avg)

} // End of main loop

//Functions

//Interrupt routine called at every external interrupt 0 request to read ADC synchronously with the square wavw from AD9833b
void AD_sample() {
  i += 1;     //i is used in ISR(ADC_vect) to define to which S[] index the sample goes (S[0], S[1], S[2] or S[3])
  if (count >= avg) {
  ADCSRA &= ~(1 << ADIE);  // ADC interrupt Disable: important here to avoid further ADC interrupts after reaching avg number
  ADCSRB = 0b00000010;
  }
  else if (n == 0){
    ADCSRB = 0b00000000;
    ADCSRA |= 0b01000000;
  }
//   Serial.write(42);// ascii 42 = *
  PORTB |= _BV(PB0);
}

//Interrupt routine called each new ADC sample ready
ISR(ADC_vect){
   S[i & n] += ADCL | ADCH<<8; //n depends on Interrupt Mode: n=3 for IntMode=1 (CHANGE); n=1 for IntMode=2 (FALLING) or IntMode=3 (RISING) 
   if((count >= (avg-1)) & (n == 0)){
    ADCSRA &= ~(1 << ADIE);  // ADC interrupt Disable: important here to avoid further ADC interrupts after reaching avg number
    ADCSRB = 0b00000010;
   }
   count += 1;//count is used to count the number of samples to accumulate
//  PORTB ^= _BV(PB0); // it takes 6 us to execute this routine with ADCclock = 307.2 kHz and variables types: S[]=int; count=i=byte 
}

// AD9833 'Reset'

void AD9833reset() {
  WriteRegister(0x100);   // Write '1' to both AD9833 Control register bit D8 for Reset and D7 for sleep
  _delay_us(1);
  WriteRegister(SQUARE);  // Write '0' to AD9833a and AD9833b Control register bit D8 for normal operation
  WriteRegistera(SINE);  // Write '0' to AD9833a Control register to set SINE wave  
}

void AD9833_sleep() {   // Write '1' to both AD9833 D7 for sleep
    WriteRegister(0x180); 
}

// Set the waveform registers in the AD9833a.
void AD9833asetWaveform(int Waveform) {
  WriteRegistera(Waveform);             // Exit & Reset to SINE, SQUARE or TRIANGLE
}

// Set the waveform registers in the AD9833b.
void AD9833bsetWaveform(int Waveform) {
  WriteRegisterb(Waveform);             // Exit & Reset to SINE, SQUARE or TRIANGLE
}

// Set the frequency and waveform registers in the AD9833a (SINE).
void AD9833asetFrequency(float frequency) {
  unsigned long FreqWord = long(frequency * 10.7374);

  int MSB = (int)((FreqWord & 0xFFFC000) >> 14);    //Only lower 14 bits are used for data
  int LSB = (int)(FreqWord & 0x3FFF);
  
  //Set control bits 15 and 14 to 0 and 1, respectively, for frequency register 0
  LSB |= 0x4000;
  MSB |= 0x4000; 
  
  WriteRegistera(0x2100);   
  WriteRegistera(LSB);                  // Write lower 16 bits to AD9833 registers
  WriteRegistera(MSB);                  // Write upper 16 bits to AD9833 registers.
  WriteRegistera(0xC000);               // Phase register=0º
}

// Set the frequency and waveform registers in the AD9833b (SQUARE).
void AD9833bsetFrequency(float frequency) {
  unsigned long FreqWord = 2* long(frequency * 10.7374);
  int LFcor = 1;
if (frequency < LFlim){LFcor = HFavg * 7;} //phase correction for Low Frequency sampling
  int MSB = (int)((FreqWord & 0xFFFC000) >> 14);    //Only lower 14 bits are used for data
  int LSB = (int)(FreqWord & 0x3FFF);
  
  //Set control bits 15 and 14 to 0 and 1, respectively, for frequency register 0
  LSB |= 0x4000;
  MSB |= 0x4000; 
  
  WriteRegisterb(0x2100);   
  WriteRegisterb(LSB);                  // Write lower 16 bits to AD9833 registers
  WriteRegisterb(MSB);                  // Write upper 16 bits to AD9833 registers.
  uint16_t phas_corr = uint16_t(frequency / 256 * LFcor);   //Phase correction for high frequencies
  WriteRegisterb(0xC000 + phas_corr); 
}

void WriteRegister(int dat) { // SPI write to both AD9833
  digitalWrite(FSYNCa, LOW);           // Set FSYNCa low before writing to AD9833 registers
  digitalWrite(FSYNCb, LOW);           // Set FSYNCb low before writing to AD9833 registers
  //_delay_us(1);                        // Give AD9833 time to get ready to receive data.
  
  SPI.transfer16(dat);   

  digitalWrite(FSYNCa, HIGH);          //Write done. Set FSYNC1 high
  digitalWrite(FSYNCb, HIGH);          //Write done. Set FSYNC1 high
}

void WriteRegistera(int dat) { // SPI write to AD9833a (sine wave)
  digitalWrite(FSYNCa, LOW);           // Set FSYNC1 low before writing to AD9833 registers
  //_delay_us(1);               // Give AD9833 time to get ready to receive data.
  SPI.transfer16(dat);   

  digitalWrite(FSYNCa, HIGH);          //Write done. Set FSYNC1 high
}

void WriteRegisterb(int dat) { // SPI write to AD9833b (square wave)
  digitalWrite(FSYNCb, LOW);           // Set FSYNC1 low before writing to AD9833 registers
  //_delay_us(1);               // Give AD9833 time to get ready to receive data.
  SPI.transfer16(dat);    

  digitalWrite(FSYNCb, HIGH);          //Write done. Set FSYNC1 high
}


//    if ( Serial.available() ) {
//      detachInterrupt(0);
//    while (Serial.available()) {
//    // get the new byte:
//    char inChar = (char)Serial.read();
//    // add it to the inputString:
//    inputString += inChar;
//    // if the incoming character is a newline, set a flag so the main loop can
//    // do something about it:
//    if (inChar == '\n') {
//      freq = inputString.toInt(); 
//      inputString = "";
//       }
//    }
//    i = 0;
//    AD9833asetFrequency(freq);//send freq to AD9833
//    AD9833bsetFrequency(freq); //send freq to AD9833
//    EIFR |= (1 << INTF0); //clear Int0 flag: ESSENTIAL to start each new frequency 
//    attachInterrupt(0, AD_sample, IntMode);   
//    AD9833reset(); 
//    EIFR |= (1 << INTF0); //clear Int0 flag: ESSENTIAL to start each new frequency 
//    memset(S, 0, sizeof(S));
//    count = 0;
//    //sbi(ADCSRA, ADSC);  //  Start the conversions
//    //attachInterrupt(0, AD_sample, CHANGE);
//    return; 
//  }

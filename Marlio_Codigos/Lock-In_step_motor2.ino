//IDE Arduino 1.8.5
//STM32F1 Boards (STM32duino.com)
//Board: "Generic STM32F103C series" 
// Variant "STM32F103C8
//CPU Speed "72 MHz"
//Upload method:  "STM32duino bootloader" or "ST-link"
//Optimizer: smallest
/*
 *LIA for Real (I) and Immaginary (Q) sampling 
 *Generate 50% PWM at PA1 (Toogle)
 *Sa taken at 25%; Sb taken at 75% (CCR2 -> PA1)
 *ADC interrupr: CCR2 
 *Obs: issue max PWM (toogle) frequency > 80 kHz
 *Accessible variables:
 *i: Real output
 *q: Imaginary output
 *nXXX: adjust central frequency
 *oXXX: adjust sampling phase off-set
 *dual coil: exciting coil: L=332 uH, Rs= 36 ohms @ 100 kHz; detecting coil: 20 turns, RDC=0.5 ohms
 *n=403
 *o=59
 *c=936
 *
 *Step motor driver: A4988 (16 micro steps)
 */
  
#define analogInPin PA5  //5

int Sa=0, Sb=0;
//int EWMAa = 0, EWMAb = 0; 
float EWMA = 0, EWMAa = 0, EWMAb = 0; 
int n = 403; //TIMER2 divider L=336uH, C1=10nF, C2=220nF
int OS = 59; //Timer2 CCR off-set
int count = 0;
int cmax = 3800;

void setup() {

  Serial.begin(115200);
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, HIGH);
  
  //PORTA Low config: A7-A6, A4:generic output; A5:analog input; A3-A0: Timer2 output (alternate); , 
  GPIOA->regs->CRL = 0x2242AAAA;
   //  Step motor with A4988 driver
   //PORTB Low config: B7-B6:generic output, B5-B0:analog input 
  GPIOB->regs->CRL = 0x22444444;

  //  Step motor with H bridge
  /*PORTB Low config: B7-B6: Timer4 output (alternate), B5-B0:analog input 
  GPIOB->regs->CRL = 0xAA444444;*/
  //PORTB High config: B15-B10:analog input; B9-B8: Timer4 output (alternate), 
  GPIOB->regs->CRH = 0x444444AA;
   //No remap (TIM4_CH1/PB6, TIM4_CH2/PB7, TIM4_CH3/PB8, TIM4_CH4/PB9)
  tim4_config();
    
  ADC1_config ();
  tim2_config();  

  delay(17000); //wait for the serial to connect

   digitalWrite(PC13, HIGH);
   Serial.print("ADC1->regs->CR1 =");
   Serial.println(ADC1->regs->CR1, BIN);
   Serial.print("ADC1->regs->CR2 =");
   Serial.println(ADC1->regs->CR2, BIN);
   Serial.print("TIMER2->regs.adv->CCMR1 =");
   Serial.println(TIMER2->regs.adv->CCMR1, BIN);
   Serial.print("TIMER2->regs.adv->DIER =");
   Serial.println(TIMER2->regs.adv->DIER, BIN);
}

void loop() {
  digitalWrite(PB6, 0);
  TIMER2->regs.adv->CCMR1 = 0x7434;// 0x24:high, 0x34:toogle, 0x44:Force inactive level, 0x64:PWM1, 0x74:PWM2 (complementary)
  TIMER2->regs.adv->CCMR2 = 0x3434;// 0x24:high, 0x34:toogle, 0x44:Force inactive level, 0x64:PWM1, 0x74:PWM2 (complementary)
  TIMER2->regs.adv->CNT = 0;  //initialize timer2 counter
  TIMER2->regs.adv->SR =0; // reset TIMER2 flags
  TIMER2->regs.adv->CR1 |= TIMER_CR1_CEN;  //Timer 2 count enable
     digitalWrite(PC13, LOW);
     //delayMicroseconds(107);

  TIMER2->regs.adv->DIER =  TIMER_DIER_CC2IE + TIMER_DIER_UIE + TIMER_DIER_TIE; //Enable compare 2 interrupt +Update interrupt enabled+Trigger interrupt enabled

     nvic_globalirq_disable(); //disable interrupts
  for (int i= 0; i < 1000; i++){

   while ((TIMER2->regs.adv->SR & TIMER_SR_CC1IF) == 0 ){} //q:TIMER_SR_CC2IF = count/2; i: TIMER_SR_CC2IF = count
   
           TIMER2->regs.adv->SR = 0; // reset TIMER2 flags
           if((i & 1) == 0) //verify even or odd number
           {    //even
                //while (!(ADC1->regs->SR & ADC_SR_EOC)){}
                Sb += ((ADC1->regs->DR & ADC_DR_DATA));
           }          
            else
            {   //odd
                //while (!(ADC1->regs->SR & ADC_SR_EOC)){}
                Sa +=  ((ADC1->regs->DR & ADC_DR_DATA));
            }
              
           //TIMER2->regs.adv->SR =0; // reset TIMER2 flags
    }
         nvic_globalirq_enable(); //enable interrupts
          TIMER2->regs.adv->CR1 &= !TIMER_CR1_CEN;  //Timer 2 count disable
          TIMER2->regs.adv->CCMR1 = 0x4444;// 0x24:high, 0x34:toogle, 0x44:Force inactive level,  0x64:PWM1, 0x74:PWM2 (complementary)
          TIMER2->regs.adv->CCMR2 = 0x4444;// 0x24:high, 0x34:toogle, 0x44:Force inactive level,  0x64:PWM1, 0x74:PWM2 (complementary)
          TIMER2->regs.adv->DIER = 0; //disable interrupt
          
           EWMAa = (EWMAa*3 + (Sa >> 5))/4;
           EWMAb = (EWMAb*3 + (Sb >> 5))/4;
           EWMA  = (EWMA*3 + (EWMAa - EWMAb))/4;
//           Serial.print(EWMAa);
//           Serial.print(' ');
//           Serial.println(Sa>>4);
//
//           Serial.println(EWMAa-EWMAb);
             Serial.println(EWMA, 1);          
//           Serial.print(Sa>>6);
//           Serial.print(' ');
//           Serial.println(Sb>>6);
           Sa = Sb = 0;
           //Serial.write((ADC1->regs->DR & ADC_DR_DATA)>>4);
            digitalWrite(PB6, 1);
            digitalWrite(PC13, HIGH);
            
            count += 1;  

          if (count >= (cmax-1)){
             // digitalWrite(PA7, !digitalRead(PA7));
              digitalWrite(PB7, !digitalRead(PB7));
              count = 0;
              }
    if (Serial.available() > 0) { 
    serialRead();
    }
}


////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////

 void serialRead(){    
   char  received = Serial.read();
//           Serial.print(received);

  switch (received) {

      case 't':
      case 'T':
//            trigger = 1;
      break;

     case 'c':
     case 'C':
            cmax = Serial.parseInt();
            cmax = constrain(cmax, 0, 3800);
            TIMER2->regs.adv->CCR3 = int(((TIMER2->regs.adv->ARR)+1)/2) +30 - OS; //Compare register 3
            TIMER2->regs.adv->CCR4 = int(((TIMER2->regs.adv->ARR)+1)/2) +30 - OS; //Compare register 4
          if (Serial.read() == '\n' || Serial.read() == '\r') {}
       break;
      
//take in-phase sample      
      case 'i':
      case 'I':
        TIMER2->regs.adv->CCR2 = int(((TIMER2->regs.adv->ARR)+1)/2)+1;//Compare register 2
        if (Serial.read() == '\n' || Serial.read() == '\r') {}
       break;
       
//take quadrature sample   
      case 'q':
      case 'Q':
        TIMER2->regs.adv->CCR2 =  1 ;//Compare register 2
        if (Serial.read() == '\n' || Serial.read() == '\r') {}
       break;

     
      case 'o':
      case 'O':
            OS = Serial.parseInt();
            OS = constrain(OS, 0, 100);
            TIMER2->regs.adv->CCR3 = int(((TIMER2->regs.adv->ARR)+1)/2) +30 - OS; //Compare register 3
            TIMER2->regs.adv->CCR4 = int(((TIMER2->regs.adv->ARR)+1)/2) +30 - OS; //Compare register 4
          if (Serial.read() == '\n' || Serial.read() == '\r') {}
       break;
       
      case 'n':
      case 'N':
            n = Serial.parseInt();
            TIMER2->regs.adv->ARR = constrain(n, 0, 65535);
            TIMER2->regs.adv->CCR1 = (TIMER2->regs.adv->ARR)-10;//Compare register 2
            TIMER2->regs.adv->CCR2 = int(((TIMER2->regs.adv->ARR)+1)/2)+1;//Compare register 2
            TIMER2->regs.adv->CCR3 = int(((TIMER2->regs.adv->ARR)+1)/2) +30 - OS; //Compare register 3
            TIMER2->regs.adv->CCR4 = int(((TIMER2->regs.adv->ARR)+1)/2) +30 - OS; //Compare register 4
          if (Serial.read() == '\n' || Serial.read() == '\r') {}
       break;


//      default :
//      if (Serial.read() == '\n' || Serial.read() == '\r') {};
    
      }
} 

void tim2_config()  //Lock-in and coil excitation
{
  //TIMER clock frequency = 72 MHz
  TIMER2->regs.adv->PSC  = 0; //0x001F; //0x0FFF; //clock Prescaler  
  TIMER2->regs.adv->ARR  = n; //359; //719;//359; //311; //0x1FFF;  //Main register max count
  TIMER2->regs.adv->CCR1 = (TIMER2->regs.adv->ARR)-10;;//Compare register 1
  TIMER2->regs.adv->CCR2 = int(((TIMER2->regs.adv->ARR)+1)/2)+1;; //Compare register 2
  TIMER2->regs.adv->CCR3 = int(((TIMER2->regs.adv->ARR)+1)/2) +30 - OS; //Compare register 3
  TIMER2->regs.adv->CCR4 = int(((TIMER2->regs.adv->ARR)+1)/2) +30 - OS; //Compare register 4
  //CCMR1: configure capture/compare mode register 1 for CCR1 anr CCR2
  TIMER2->regs.adv->CCMR1 = 0x3474;// 0x24:high, 0x34:toogle, 0x44:Force inactive level, 0x64:PWM1, 0x74:PWM2 (complementary)
  TIMER2->regs.adv->CCMR2 = 0x3434;// 0x24:high, 0x34:toogle, 0x44:Force inactive level, 0x64:PWM1, 0x74:PWM2 (complementary)
 // TIMER2->regs.adv->DIER =  TIMER_DIER_CC2IE + TIMER_DIER_UIE + TIMER_DIER_TIE; //Enable compare 2 interrupt +Update interrupt enabled+Trigger interrupt enabled
//  TIMER2->regs.adv->DIER |= TIMER_DIER_UIE; //
//  TIMER2->regs.adv->DIER |= TIMER_DIER_TIE; //Trigger interrupt enabled

  //TIMER2->regs.adv->EGR |= TIMER_EGR_TG + TIMER_EGR_CC2G; //TIF flag is set in TIMx_SR register and Related interrupt can occur
  TIMER2->regs.adv->CCER |= TIMER_CCER_CC1E + TIMER_CCER_CC2E +TIMER_CCER_CC3E + TIMER_CCER_CC4E; //capture/compare enable register:OC2 signal is output on the corresponding output pin

  //TIMER2->regs.adv->CR1 |= TIMER_CR1_CEN;  //Timer 2 count enable
}

void tim4_config() //two phase step motor (each phase driven by 2 PWM outputs)
{
  //TIMER4 clock frequency = 72 MHz
  TIMER4->regs.adv->PSC  = 224; //Clock Prescaler divider => finPWM=320 kHz
  TIMER4->regs.adv->ARR  = 31;  //Main register max count => foutPWM=10 kHz, 32 levels

//  TIMER4->regs.adv->DIER |= TIMER_DIER_CC1IE;
//  TIMER4->regs.adv->DIER |= TIMER_DIER_UIE;

    //output operation mode:
  TIMER4->regs.adv->CCMR1 = 0x6464;// 0x24:high, 0x34:toogle, 0x44:Force inactive level, 0x64:PWM1, 0x74:PWM2 (complementary)
  TIMER4->regs.adv->CCMR2 = 0x6464;// 0x24:high, 0x34:toogle, 0x44:Force inactive level, 0x64:PWM1, 0x74:PWM2 (complementary)
 
  //output registers:
  TIMER4->regs.adv->CCR1 = 0; //Compare register 1
  TIMER4->regs.adv->CCR2 = 0; //Compare register 2
  TIMER4->regs.adv->CCR3 = 10; //Compare register 3
  TIMER4->regs.adv->CCR4 = 0; //Compare register 4
  
   //outputs oenable:
  TIMER4->regs.adv->CCER |= TIMER_CCER_CC1E + TIMER_CCER_CC2E +TIMER_CCER_CC3E + TIMER_CCER_CC4E; //capture/compare enable register:OC2 signal is output on the corresponding output pin
  TIMER4->regs.adv->CR1 |= TIMER_CR1_CEN;  //Timer 4 count enable
}


void ADC1_config ()
{
  //rcc_set_prescaler(RCC_PRESCALER_ADC,RCC_ADCPRE_PCLK_DIV_6);
  //ADC clock frequency = 12 MHz
  adc_set_sample_rate(ADC1, ADC_SMPR_1_5); // ADC_SMPR_1_5, ADC_SMPR_7_5, ADC_SMPR_13_5, 
  //ADC_SMPR_7_5 => Ts = 1.667 us
  adc_set_reg_seqlen(ADC1, 1);//convert just 1 channel
  ADC1->regs->SQR3 = 5; //ADC channel 5 (PA5)
  ADC1->regs->CR1 |= ADC_CR1_EOCIE; //0x0000; // ADC_CR1_INDEPENDENT mode, enable EOC interrupt;
  ADC1->regs->CR2 = ADC_CR2_EXTTRIG + ADC_CR2_EXTSEL_TIM2_CC2;    // Set external trigger mode on Tim2 CC2
  //ADC1->regs->CR2 |= ADC_CR2_EXTSEL_SWSTART;
  ADC1->regs->CR2 |= ADC_CR2_ADON;             //Switch ADC ON
  ADC1->regs->CR2 |= ADC_CR2_CAL;             //Start ADC calibration
  while(ADC1->regs->CR2 & ADC_CR2_CAL) { }        //wait calibration to finish
}


 /*
 * TODO: Fazer verificação no sysclock
 *
 * TODO: Fazer a calibração
 *
 * TODO: Valor de res
 *
 * TODO: Desligar todos os perifericos no fim
 * 
 * FSYNCA  <-->  PB0 <-->  BOARD_SPI1_FSYNCA_PIN
 * FSYNCB  <-->  PB1 <-->  BOARD_SPI1_FSYNCB_PIN
 * SCK     <-->  PA5 <-->  BOARD_SPI1_SCK_PIN
 * MISO    <-->  PA6 <-->  BOARD_SPI1_MISO_PIN
 * MOSI    <-->  PA7 <-->  BOARD_SPI1_MOSI_PIN
 */

#include <math.h>//Raiz e power
#include <SPI.h>
#include <STM32ADC.h>

#define SPI1_FSYNCA_PIN PB0
#define SPI1_FSYNCB_PIN PB1
uint8 adc_pin = PA0; 

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

//Endereço para configurar o ADC do STM
#define ADC1_DR_Address     ((uint32_t)0x4001244C)  

#define NPTS 101  //numero de pontos = 101

enum {
  RISE = 0,
  FALL
};

void reset_AD9833();
void init_AD9833(float frequency);
void setFrequency(float frequency);
void sleep_AD9833();
void writeRegisterA( uint16_t command );
void writeRegisterB( uint16_t command );

//A frequencia maxima para esse programa é 125kHz
//frequencia inicial = 1 Hz
//frequencia final   = 100k Hz

static void DMA_Interrupt() {
  DMA_flag = 1;
}

uint16_t data = 0;    //Valor de conversão do ADC
uint8_t DMA_flag = 0; //Flag que a interrupção do DMA ativa
uint8_t count = 0;    //Quantas vezes foi feita as mesmas medidas
uint8_t wavePoint = 1;  //Ponto da onda senoidal estamos analisando
uint8_t nptsA = 0;    //numero do ponto atual


long RealVar = 0; //Variavel do valor real
long ImagVar = 0; //Variavel do valor imaginario
long Real[NPTS];  //Todos os valores reais da tensão
long Imag[NPTS];  //Todos os valores imaginarios da tensão
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


  STM32ADC myAdc(ADC1);
void setup() {
  Serial.print("Setup...");
  /* Configure SPI1 ------------------------------------------------------------------*/
  SPI.begin(); //Initialize the SPI_1 port.
  SPI.setBitOrder(MSBFIRST); // Set the SPI_1 bit order
  SPI.setDataMode(SPI_MODE0); //Set the  SPI_2 data mode 0
  SPI.setClockDivider(SPI_CLOCK_DIV16);      // Slow speed (72 / 16 = 4.5 MHz SPI_1 speed)
  pinMode(SPI1_FSYNCA_PIN, OUTPUT);
  pinMode(SPI1_FSYNCB_PIN, OUTPUT);
  
  /* Configure ADC1 ------------------------------------------------------------------*/
  sampleTime(freq);
  myADC.setPins(&adc_pin, 1);
  myADC.setTrigger(ADC_EXT_EV_EXTI11);
  
  
  //set the DMA transfer for the ADC. 
  //in this case we want to increment the memory side and run it in circular mode
  //By doing this, we can read the last value sampled from the channels by reading the dataPoints array
  myADC.setDMA(data, 1,DMA_CIRC_MODE, NULL);
  myADC.attachDMAInterrupt(&DMA_Interrupt);
  myADC.calibrate();
  myADC.startConversion();   
  
  
  Serial.begin(115200); 

  Serial.println("done.");
}

void loop() {
  //É iniciado os valores de cada vetor
  freqA[nptsA] = freq;
  for(int i = 0; i < NPTS; i++) {
      Real[i] = 0;
      Imag[i] = 0;
  }
   while(1) {
    while(count < avg){
      while(1) {
        if(DMA_flag) {
          //ADC_Cmd(ADC1, DISABLE); Se necessario
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
          break;//Para sair do segundo while(1)
        }//End of if(DMA_flag)
      }//End of second while(1)
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
  if (frequency > 125e3)
    frequency = 125e3;
  if (frequency < 0.0) 
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
    //while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    //SPI_I2S_SendData(SPI1, command);
  SPI.transfer(command);
  digitalWrite(SPI1_FSYNCA_PIN, HIGH);
}

void writeRegisterB( uint16_t command ) {
 digitalWrite(SPI1_FSYNCB_PIN, LOW);
    //while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    //SPI_I2S_SendData(SPI1, command);
  SPI.transfer(command);
  digitalWrite(SPI1_FSYNCB_PIN, HIGH);
}

/*******************************************************************/
/*               Funções de configuração do STM                    */
/*******************************************************************/
void init_Clock() {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  |
              RCC_APB2Periph_GPIOB |
              RCC_APB2Periph_GPIOC |
              RCC_APB2Periph_ADC1  |
              RCC_APB2Periph_SPI1  | 
            RCC_APB2Periph_AFIO,  ENABLE);

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  //Considerando 56MHz = HCLK = PCLK2
  RCC_ADCCLKConfig(RCC_PCLK2_Div4); 

  //#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
  //  /* ADCCLK = PCLK2/2 */
  //  RCC_ADCCLKConfig(RCC_PCLK2_Div2); 
  //#else
  //  /* ADCCLK = PCLK2/4 */
  //  RCC_ADCCLKConfig(RCC_PCLK2_Div4); 
  //#endif
}

void init_GPIO()
{
  /* Configure ADC pin ---------------------------------------------*/
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure SPI1 pins: SCK e MOSI -------------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  

  /* Configure SPI1 pins: FSYNC1 e FSYNC2 --------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  

  /* Configure LED builtin pin -------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

//SPI2 para debbugar INCOMPLETO
//  /* Configure SPI2 pins: SCK e MOSI -------------------------------*/
//  GPIO_InitTypeDef GPIO_InitStructure3;
//  GPIO_InitStructure3.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
//  GPIO_InitStructure3.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure3.GPIO_Mode = GPIO_Mode_AF_PP;
//  GPIO_Init(GPIOA, &GPIO_InitStructure3); 
//  /* Configure SPI2 pin: SS ---------------------------------------*/
//  GPIO_InitTypeDef GPIO_InitStructure3;
//  GPIO_InitStructure3.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
//  GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_Out_PP;
//  GPIO_InitStructure3.GPIO_Mode = GPIO_Mode_AF_PP;
//  GPIO_Init(GPIOA, &GPIO_InitStructure3); 
  
}

void init_ADC(double freq) {


}

/*
 * Sampletime altera o numero de ciclos utilizados para fazer a amostra da
 *  tensão, quanto maior, maior o tempo e a qualidade da conversão. Quanto
 *  maior a frequencia mais é diminuido os ciclos de amostra.
 */

void sampleTime(double freq) {

  if(freq < 13157){
    setSampleRate(ADC_SMPR_239_5);

  }
  else if(freq < 35714){
    setSampleRate(ADC_SMPR_71_5);

  }
  else if(freq < 42682){
    setSampleRate(ADC_SMPR_55_5);
  }
  else if(freq < 51470){
    setSampleRate(ADC_SMPR_41_5);
  }
  else if(freq < 63636){
    setSampleRate(ADC_SMPR_28_5);
  }
  else if(freq < 87500){
    setSampleRate(ADC_SMPR_13_5);
  }
  else if(freq < 102940){
    setSampleRate(ADC_SMPR_7_5);
  }
  else{
    setSampleRate(ADC_SMPR_1_5);
  }
}
/*
 * External event on PA00 RISE or FALL
 *
 */
void init_EXT( int externalTrigger ) {
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

  EXTI_InitTypeDef EXTI_InitStruct;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Event;

  if(externalTrigger == RISE)
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
  else
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;

  EXTI_InitStruct.EXTI_Line = EXTI_Line0;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStruct);
}


/*
 * Configuração do DMA1 para receber informações do
 * ADC 1 e enviar para &data, com um buffer de 1 no modo
 * circular
 *
 */
void init_DMA(){
  DMA_InitTypeDef DMA_Initstruct;
  DMA_Initstruct.DMA_PeripheralBaseAddr = ADC1_DR_Address; 
  DMA_Initstruct.DMA_MemoryBaseAddr = (uint32_t)&data;    
  DMA_Initstruct.DMA_DIR = DMA_DIR_PeripheralSRC; 
  DMA_Initstruct.DMA_BufferSize = 1; 
  DMA_Initstruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_Initstruct.DMA_MemoryInc = DMA_MemoryInc_Disable; 
  DMA_Initstruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_Initstruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_Initstruct.DMA_Mode = DMA_Mode_Circular;
  DMA_Initstruct.DMA_Priority = DMA_Priority_High;
  DMA_Initstruct.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_Initstruct);

  DMA_Cmd(DMA1_Channel1,ENABLE);

  //DMA Transfer complete
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
}

/*
 * Configura o SPI1 para ser o mestre com uma comunicação onde 
 * só ele fala com dados de 16 bits e SS/FSYNC via hardware que
 * no nosso caso é por GPIO
 */

/* 
 * Configure and enable DMA interrupt 
 */
void init_NVIC() {
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

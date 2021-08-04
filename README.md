# Prototipo_Serbena
Programa desenvolvido durante a IC:
"Construção de Dispositivo Eletrônico Portátil para Medida de Impedância Operando de 1 kHz a 10 kHz". Para utilizar o metodo lock-in por subtração para medir a impedancia.

## STM32F103C8T6

Este é o microcontrolador utilizado, ele necessita da biblioteca STM32F10x_StdPeriph_Lib_V3.5.0, do compilador gcc-arm-none-eabi e do programador ST-link.

stlink install (Arch)

	$ sudo pacman -S stlink

gcc-arm-none-eabi 
	
	https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads

STM32F10x_StdPeriph_Lib_V3.5.0

	https://www.st.com/en/embedded-software/stsw-stm32054.html

## Estrutura do programa

### Main
   No main é feito uma varredura logaritmica entre duas frequencias, analisando a impedancia em cada frequencia. Esse processo se repete um numero \*avg de vezes para que seja feito uma media. 
   Para a obtenção do valor da impedancia é utilizando o método lock-in por subtração, ou seja, é obtido a tensão em 4 pontos da onda que passa pelo dispositivo sobre teste. Para a sincronização é utilizado uma onda secundaria que utiliza o mesmo cristal e esta com o dobro da frequencia, dependendo de que ponto queremos utilizamos a descida ou subida dessa onda secundaria para iniciar a conversão do ADC. Este valor obtido pelo ADC é guardado na memoria pelo DMA e é utilizado juntamente com os valores de calibração para calcular o valor da impendacia final.

### Utils
   Na biblioteca utils eu coloquei todas as funções que configuram o gerador de frequencias e o microcontrolador.

### stm32f10x_conf stm32f10x_it
   Respectivamente esses dois arquivos são a inclusão de todas as bibliotecas necessarias do stm32f10x e as funções de interrupção. Mesmo que esses dois arquivos poderiam não existir, eu decidi mante-los já que é a forma padrão da utilização da biblioteca stm32f10x.

### Makefile
   É utilizado para compilar e carregar o codigo no microcontrolador utilizando gcc-arm e o stlink.

## Pinout

| STM32F103C8 Pi | Modulos | Description |
| :--------- | :---------- | :---------------------------------------- |
| **PA07** | **MOSI AD9833** | SPI Master Out Slave In data pin |
| **PA05** | **SCK AD9833B** | SPI Clock pin |
| **PB00** | **FSYNC AD9833A** | FSYNC pin |
| **PB01** | **FSYNC AD9833B** | FSYNC pin |
| **PA01** | **VOUT AD9833A** | Sinal que passou pelo eletrodo |
| **PA00** | **VOUT AD9833B** | Sinal quadrado utilizado para sincronização |
| **Pn0x** | **SCK Debug** | SPI Clock pin |
| **Pn0x** | **MOSI Debug** | SPI Master Out Slave In data pin |
| **Pn0x** | **SCK Debug** | SPI Clock pin |

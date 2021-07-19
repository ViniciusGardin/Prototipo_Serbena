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

## Main

No main é feito uma conversão ADC, cujo resultado é tranferido para memoria pelo DMA.
 - Existe uma interrupção na finalização da tranferencia do DMA que seta uma flag
## Pinout

| STM32F103C8 Pi | Modulos | Description |
| :--------- | :---------- | :---------------------------------------- |
| **Pn0x** | **SCK Debug** | SPI Clock pin |
| **Pn0x** | **MOSI Debug** | SPI Master Out Slave In data pin |
| **Pn0x** | **SCK Debug** | SPI Clock pin |
| **Pn0x** | **MOSI AD9833A** | SPI Master Out Slave In data pin |
| **Pn0x** | **FSYNC AD9833A** | FSYNC pin |
| **Pn0x** | **SCK AD9833B** | SPI Clock pin |
| **Pn0x** | **MOSI AD9833B** | SPI Master Out Slave In data pin |
| **Pn0x** | **FSYNC AD9833B** | FSYNC pin |
| **Pn0x** | **VOUT AD9833A** | Sinal que passou pelo eletrodo |
| **Pn0x** | **VOUT AD9833B** | Sinal quadrado utilizado para sincronização |

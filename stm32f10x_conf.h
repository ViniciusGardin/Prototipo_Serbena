#ifndef __STM32F10x_CONF_H
#define __STM32F10x_CONF_H

//Lembrar de incluir no MakeFile TAMBEM!
#include <stm32f10x_adc.h>
#include <stm32f10x_dma.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_spi.h>
#include <stm32f10x_exti.h>
#include <misc.h>
#include "stm32f10x_it.h"
// no asserts
#define assert_param(expr) ((void)0)
#endif

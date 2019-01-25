#pragma once

#include "stm32f4xx.h"
#include "hdr_bitband.h"

class Gpio
{
	static void HardwareInit()
	{
		//Uruchomienie wyjsc;
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

		// PC06, PC11 - Channel 1 start direction A/B
		// PD00, PD02 - Channel 2 start direction A/B
		// PE09 - Enable PWM output
		GPIOD->MODER |= GPIO_MODER_MODER2_0 | GPIO_MODER_MODER0_0  ;
		GPIOE->MODER |= GPIO_MODER_MODER9_0;
		GPIOC->MODER |= GPIO_MODER_MODER11_0 | GPIO_MODER_MODER6_0;
	}

public:

	static void Init()
	{
		HardwareInit();
	}

	static volatile unsigned long & Dir2A()
	{
		return *(volatile unsigned long*) m_BITBAND_PERIPH(&GPIOC->ODR, 11);
	}

	static volatile unsigned long & Dir2B()
	{
		return *(volatile unsigned long*) m_BITBAND_PERIPH(&GPIOC->ODR, 6);
	}

	static volatile unsigned long & Dir1A()
	{
		return *(volatile unsigned long*) m_BITBAND_PERIPH(&GPIOD->ODR, 0);
	}

	static volatile unsigned long & Dir1B()
	{
		return *(volatile unsigned long*) m_BITBAND_PERIPH(&GPIOD->ODR, 2);
	}

	static volatile unsigned long & EnableCLK()
	{
		return *(volatile unsigned long*) m_BITBAND_PERIPH(&GPIOE->ODR, 9);
	}

};

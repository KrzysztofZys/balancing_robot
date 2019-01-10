#pragma once

#include "stm32f4xx.h"
#include "hdr_bitband.h"

class Led
{
	static void HardwareInit()
	{
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

		// PD13, PD14, PD15 - LEDy - PA5, PA7
		GPIOD->MODER |= GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0 |GPIO_MODER_MODER15_0;
		GPIOA->MODER |= GPIO_MODER_MODER5_0 | GPIO_MODER_MODER7_0;
	}

public:

	static void Init()
	{
		HardwareInit();
	}
// Two new leds added PA7, PA5
	static volatile unsigned long & Red2()
	{
		return *(volatile unsigned long*) m_BITBAND_PERIPH(&GPIOA->ODR, 5);
	}

	static volatile unsigned long & Yellow2()
	{
		return *(volatile unsigned long*) m_BITBAND_PERIPH(&GPIOA->ODR, 7);
	}

	static volatile unsigned long & Yellow()
	{
		return *(volatile unsigned long*) m_BITBAND_PERIPH(&GPIOD->ODR, 13);
	}

	static volatile unsigned long & Red()
	{
		return *(volatile unsigned long*) m_BITBAND_PERIPH(&GPIOD->ODR, 14);
	}

	static volatile unsigned long & Blue()
	{
		return *(volatile unsigned long*) m_BITBAND_PERIPH(&GPIOD->ODR, 15);
	}

};

#pragma once

#include "stm32f4xx.h"
#include "hdr_bitband.h"

class Led
{
	static void HardwareInit()
	{
		//Uruchomienie wyjsc;
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

		// PD13, PD14, PD15 - LEDy - Pe9, pe11
		GPIOD->MODER |= GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1 ;
		GPIOE->MODER |= GPIO_MODER_MODER9_0 | GPIO_MODER_MODER11_0;
		GPIOC->MODER |= GPIO_MODER_MODER11_0 | GPIO_MODER_MODER6_0;
	}

public:

	static void Init()
	{
		HardwareInit();
	}
// Two new leds added PE9, PE11


	static volatile unsigned long & Yellow()
	{
	//	return *(volatile unsigned long*) m_BITBAND_PERIPH(&GPIOD->ODR, 13);
	}

	static volatile unsigned long & Red()
	{
	//	return *(volatile unsigned long*) m_BITBAND_PERIPH(&GPIOD->ODR, 14);
	}

	static volatile unsigned long & Blue()
	{
		//return *(volatile unsigned long*) m_BITBAND_PERIPH(&GPIOD->ODR, 15);
	}

	static volatile unsigned long & Dir1()
	{
		return *(volatile unsigned long*) m_BITBAND_PERIPH(&GPIOC->ODR, 6);
	}

	static volatile unsigned long & Dir2()
	{
		return *(volatile unsigned long*) m_BITBAND_PERIPH(&GPIOC->ODR, 11);
	}

	static volatile unsigned long & EnableCLK()
	{
		return *(volatile unsigned long*) m_BITBAND_PERIPH(&GPIOE->ODR, 9);
	}

	static volatile unsigned long & Red2()
	{
		return *(volatile unsigned long*) m_BITBAND_PERIPH(&GPIOE->ODR, 11);
	}
};

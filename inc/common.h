#pragma once
#include <string.h>

#include "stm32f4xx.h"
#include "uart_communication_interface.h"
#include "cmd_msgs.h"

#include "rangfinder_2y0a21.h"
#include "accelerometer_lis302.h"
#include "led_interface.h"

#include "filter2_iir.h"
#include "data_recorder.h"

#include "math.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"

#define CPU_CLK	((uint32_t)168000000)

class App
{

	DataRecorder < 2048, float > rec1;
	//DataRecorder < 512, uint16_t > rec2;


	float a1 = 0.000944691843840;
	float a2 = 0.001889383687680;
	float a3 = 0.000944691843840;
	float b1 = 1;
	float b2 = -1.911197067426073;
	float b3 = 0.914975834801434;
	float filterParams1[6] = {a1, a2, a3, b1, b2, b3};
	Filter2Iir filter1;

	// cykle w [ms]
	volatile uint32_t mainClock;
	volatile uint32_t auxClock;

	int temp;

	float samples_bufX[256];
	float samples_bufY[256];
	float samples_bufZ[256];
	float samples_bufangX[256];
	float samples_bufangY[256];
	float samples_bufangZ[256];
	int sample;
	bool stable;
	float acc_corrected;
	int value_PWM;

	float outFilter [2];
	float prev_outFilter [2];
	float derivative[2];
	float kd = 0.15;
	float der_freq=1000;
public:


	Rangfinder2Y0A21 rang;
	AccelerometerLIS302 acc;
	UartCommunicationInterface com;


	App(): filter1(filterParams1)
	{
		mainClock = 0;
		auxClock = 0;
		stable = false;
		sample = 0;
		acc_corrected=0;
		value_PWM=0;
		temp=0;
	};

	void GeneralHardwareInit()
	{
		// inicjalizacja mikrokontrolera
		SystemInit();

		// ustawienie zegara systemowego w programie
		if (SysTick_Config(CPU_CLK/1000))
		{
			while (1);
		}

		NVIC_EnableIRQ(DMA1_Stream6_IRQn);
		NVIC_EnableIRQ(USART2_IRQn);
		NVIC_EnableIRQ(SPI1_IRQn);
		NVIC_EnableIRQ(EXTI0_IRQn);
	}


	void Init()
	{
		GeneralHardwareInit();

		Led::Init();
		acc.Init();
	    rang.Init();
		com.Init();
	//	Led::Yellow2()= 0;
		//rang.Start();

	}

	void TM_GPIO_Init(void) {
	    GPIO_InitTypeDef GPIO_InitStruct;

	    /* Clock for GPIOD */
	    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	    /* Alternating functions for pins */
	    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
	    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);

	    /* Set pins */
	    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	    GPIO_Init(GPIOD, &GPIO_InitStruct);
	}

	void TM_TIMER_Init(void) {
	    TIM_TimeBaseInitTypeDef TIM_BaseStruct;

	    /* Enable clock for TIM4 */
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	/*
	    TIM4 is connected to APB1 bus, which has on F407 device 42MHz clock
	    But, timer has internal PLL, which double this frequency for timer, up to 84MHz
	    Remember: Not each timer is connected to APB1, there are also timers connected
	    on APB2, which works at 84MHz by default, and internal PLL increase
	    this to up to 168MHz

	    Set timer prescaller
	    Timer count frequency is set with

	    timer_tick_frequency = Timer_default_frequency / (prescaller_set + 1)

	    In our case, we want a max frequency for timer, so we set prescaller to 0
	    And our timer will have tick frequency

	    timer_tick_frequency = 84000000 / (0 + 1) = 84000000
	*/
	    TIM_BaseStruct.TIM_Prescaler = 0;
	    /* Count up */
	    TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	/*
	    Set timer period when it have reset
	    First you have to know max value for timer
	    In our case it is 16bit = 65535
	    To get your frequency for PWM, equation is simple

	    PWM_frequency = timer_tick_frequency / (TIM_Period + 1)

	    If you know your PWM frequency you want to have timer period set correct

	    TIM_Period = timer_tick_frequency / PWM_frequency - 1

	    In our case, for 10Khz PWM_frequency, set Period to

	    TIM_Period = 84000000 / 10000 - 1 = 8399

	    If you get TIM_Period larger than max timer value (in our case 65535),
	    you have to choose larger prescaler and slow down timer tick frequency
	*/
	    TIM_BaseStruct.TIM_Period = 8399; /* 10kHz PWM */
	    TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	    TIM_BaseStruct.TIM_RepetitionCounter = 0;
	    /* Initialize TIM4 */
	    TIM_TimeBaseInit(TIM4, &TIM_BaseStruct);
	    /* Start count on TIM4 */
	    TIM_Cmd(TIM4, ENABLE);
	}

	void TM_PWM_Init(void) {
	    TIM_OCInitTypeDef TIM_OCStruct;

	    /* Common settings */

	    /* PWM mode 2 = Clear on compare match */
	    /* PWM mode 1 = Set on compare match */
	    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
	    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
	    TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

	/*
	    To get proper duty cycle, you have simple equation

	    pulse_length = ((TIM_Period + 1) * DutyCycle) / 100 - 1

	    where DutyCycle is in percent, between 0 and 100%

	    25% duty cycle:     pulse_length = ((8399 + 1) * 25) / 100 - 1 = 2099
	    50% duty cycle:     pulse_length = ((8399 + 1) * 50) / 100 - 1 = 4199
	    75% duty cycle:     pulse_length = ((8399 + 1) * 75) / 100 - 1 = 6299
	    100% duty cycle:    pulse_length = ((8399 + 1) * 100) / 100 - 1 = 8399

	    Remember: if pulse_length is larger than TIM_Period, you will have output HIGH all the time
	    TIM_OCStruct.TIM_Pulse = 1000; // 50% duty cycle
	    TIM_OC2Init(TIM4, &TIM_OCStruct);
	    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

	    TIM_OCStruct.TIM_Pulse = 6299; // 75% duty cycle
	    TIM_OC3Init(TIM4, &TIM_OCStruct);
	    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	    TIM_OCStruct.TIM_Pulse = 8399; // 100% duty cycle
	    TIM_OC4Init(TIM4, &TIM_OCStruct);
	    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

	*/
	    TIM_OCStruct.TIM_Pulse = 4000; // 25% duty cycle
	    TIM_OC1Init(TIM4, &TIM_OCStruct);
	    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);


	}

	//Caluclates value of PWM
	int calculateValue (float val){
		// 0,63 ~ 40 degrees
		float max_temp = M_PI/2;
		float kp = 1;
		if (val > 0.63) {
			//val = 0.63;
		}
		temp = static_cast<int>(kp*(abs(val)*8399)/max_temp);
		return temp;
	}


	void PeriodicUpdate()
	{
		mainClock++;
		auxClock++;

		com.PeriodicUpdate();
		rang.PeriodicUpdate();
		acc.WriteReadStart();

		if (auxClock == 1000)
		{
			Led::Red2()^=1;

			auxClock = 0;

		}
	}

	void Run()
	{
		TIM_OCInitTypeDef TIM_OCStruct;

        /* Initialize system */
	    SystemInit();
	    /* Init GPIO */
	    TM_GPIO_Init();
	    /* Init timer */
	    TM_TIMER_Init();
	    /* Init PWM */
	    TM_PWM_Init();

		while(1)
		{

			Led::EnableCLK()^=1;
			if(com.isFrameReceived)
			{
				if(com.CheckFrame())
				{
					static CmdMaster cmdM;

					// analyze data from master
					com.GetUserData(&cmdM, sizeof(CmdMaster));

					// prepare data to send
					//static CmdSlave cmdS;
					if (cmdM.cmd == 9){
						Led::Red2()=1;
						stable = true;
					//	cmdS.data1 = acc.angle[0];
					//	cmdS.data2 = acc.angle[1];
					//	com.SendUserData(&cmdS, sizeof(CmdSlave));
					}
					else stable = false;

				}
				com.isFrameReceived = false;
			}
			stable = true;

			if(acc.isDataReady)
			{


				acc.ScaleData();
				acc.CalculateAngles();
				if (sample < 256)
				{
					samples_bufX[sample] = acc.accVal[0];
					samples_bufY[sample] = acc.accVal[1];
					samples_bufZ[sample] = acc.accVal[2];
					samples_bufangX[sample] = acc.angle[0];
					samples_bufangY[sample] = acc.angle[1];
					samples_bufangZ[sample++] = acc.angle[2];
				}

				//outFilter[0] = filter1.CalculateOutput(acc.accVal[0]);
				prev_outFilter[1] = outFilter[1];
				outFilter[1] = filter1.CalculateOutput(acc.angle[1] - M_PI/2);
				//outFilter[1] = filter1.CalculateOutput(2);
				derivative[1]= kd*der_freq*(outFilter[1]-prev_outFilter[1]);
				//rec1.RecordCyclically(out);
				acc.isDataReady = false;
			}

			//Simple Algorithm
			acc_corrected = outFilter[1] - derivative[1];
			value_PWM = 0;
			calculateValue(abs(acc_corrected));
			if (stable && acc_corrected >= 0) {

				Led::Dir1()=1;
				Led::Dir2()=0;
			}
			else if(stable && acc_corrected < 0){
				Led::Dir2()=1;
				Led::Dir1()=0;
			}
			else {
				Led::Dir2()=0;
				Led::Dir1()=0;
			}

			if (stable){
				TIM4->CCR1 = temp;
			}

			if(rang.isDataReady)
			{
				// Signal processing

			}
		}
	}

};

extern App * pApp;

#pragma once
#include <string.h>

#include "stm32f4xx.h"
#include "uart_communication_interface.h"
#include "cmd_msgs.h"

#include "rangfinder_2y0a21.h"
#include "accelerometer_lis302.h"
#include "gpio_interface.h"
#include "pwm_interface.h"
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

	//Filter parameter values
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

	int PWM1_value, PWM2_value;

	float samples_bufX[2048];
	float samples_bufY[2048];
	float samples_bufZ[2048];
	float samples_buffilterX[2048];
	float samples_bufangX[2048];
	float samples_bufangY[2048];
	float samples_buffilterY[2048];
	float samples_bufcorrected[2048];
	int sample = 0;
	bool stable;
	float acc_corrected;

	float outFilter [2];
	float prev_outFilter [2];
	float derivative[2];
	float kd = 0.1;
	float kp = 0.9;
	float der_freq=1000;
public:


	Rangfinder2Y0A21 rang;
	AccelerometerLIS302 acc;
	UartCommunicationInterface com;
	PWMInterface pwm;

	App(): filter1(filterParams1)
	{
		mainClock = 0;
		auxClock = 0;
		stable = false;
		sample = 0;
		acc_corrected=0;
		PWM1_value=0;
		PWM2_value=0;
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
		Gpio::Init();
		acc.Init();
	    rang.Init();
		com.Init();
		pwm.Init();
		//rang.Start();

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

		while(1)
		{

			Gpio::EnableCLK()^=1;
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
				//outFilter[0] = filter1.CalculateOutput(acc.accVal[0]);
				prev_outFilter[1] = outFilter[1];
				//outFilter[1] = filter1.CalculateOutput(acc.angle[1] - M_PI/2);
				outFilter[1] = filter1.CalculateOutput(acc.angle[1]);
				//outFilter[1] = filter1.CalculateOutput(2);
				derivative[1]= kd*der_freq*(outFilter[1]-prev_outFilter[1]);

				acc.ScaleData();
				acc.CalculateAngles();

				if (sample < 2048)
				{
					samples_bufX[sample] = acc.accVal[0];
					samples_bufY[sample] = acc.accVal[1];
					samples_bufZ[sample] = acc.accVal[2];

					samples_bufangX[sample] = acc.angle[0];
					samples_bufangY[sample] = acc.angle[1];

					samples_buffilterY[sample] = outFilter[1];

					samples_bufcorrected[sample] = derivative[1];

					sample = sample +1;

				}
				else
					sample = 2048;



				//rec1.RecordCyclically(out);
				acc.isDataReady = false;
			}

			//Simple Algorithm
			acc_corrected = outFilter[1] + derivative[1];
			PWM1_value = pwm.CalculateValue((abs(acc_corrected)),kp);
			PWM2_value = pwm.CalculateValue((abs(acc_corrected)),kp);
			if (stable && acc_corrected > 0) {

				Gpio::Dir2B()=1;
				Gpio::Dir2A()=0;
			}
			else if(stable && acc_corrected < 0){
				Gpio::Dir2A()=1;
				Gpio::Dir2B()=0;
			}
			else {

			}

			if (stable){
				pwm.WriteValue(1, PWM1_value);
				pwm.WriteValue(3, PWM2_value);
			}

			if(rang.isDataReady)
			{
				// Signal processing

			}
		}
	}

};

extern App * pApp;

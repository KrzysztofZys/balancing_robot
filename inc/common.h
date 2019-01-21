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


#define CPU_CLK	((uint32_t)168000000)

class App
{

	DataRecorder < 2048, float > rec1;
	//DataRecorder < 512, uint16_t > rec2;

	float filterParams1[6] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

	Filter2Iir filter1;

	// cykle w [ms]
	volatile uint32_t mainClock;
	volatile uint32_t auxClock;

	float samples_bufX[256];
	float samples_bufY[256];
	float samples_bufZ[256];
	float samples_bufangX[256];
	float samples_bufangY[256];
	float samples_bufangZ[256];
	int sample;
public:


	Rangfinder2Y0A21 rang;
	AccelerometerLIS302 acc;
	UartCommunicationInterface com;


	App(): filter1(filterParams1)
	{
		mainClock = 0;
		auxClock = 0;

		sample = 0;
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
		//rang.Start();

	}

	void PeriodicUpdate()
	{
		mainClock++;
		auxClock++;

		com.PeriodicUpdate();
		rang.PeriodicUpdate();
		acc.WriteReadStart();

		if (auxClock == 100)
		{
		  Led::Yellow2()^= 1;
		 // Led::EnableCLK()^= 1;
		  auxClock = 0;
		}
	}

	void Run()
	{
		while(1)
		{
			if(com.isFrameReceived)
			{
				if(com.CheckFrame())
				{
					static CmdMaster cmdM;

					// analyze data from master
					com.GetUserData(&cmdM, sizeof(CmdMaster));

					// prepare data to send
					static CmdSlave cmdS;
					if (cmdM.cmd == 9){
						//Led::Red2()=1;
						//Led::DIR1()=1;
						//Led::PWM1()=1;
						cmdS.data1 = acc.angle[0];
						cmdS.data2 = acc.angle[1];
						com.SendUserData(&cmdS, sizeof(CmdSlave));
					}



				}
				com.isFrameReceived = false;
			}

			if(acc.isDataReady)
			{



				float out;
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

				//out = filter1.CalculateOutput(acc.accVal[0]);
				out = filter1.CalculateOutput(acc.accVal[0]);
				//rec1.RecordCyclically(out);
				acc.isDataReady = false;
			}

			if(rang.isDataReady)
			{
				// Signal processing

			}
		}
	}

};

extern App * pApp;

/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "common.h"
			

App * pApp;

int main(void)
{
	App app;
	pApp = &app;
	app.Init();

	__enable_irq();

	app.Run();
}


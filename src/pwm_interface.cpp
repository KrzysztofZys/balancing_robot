#include "pwm_interface.h"

void PWMInterface::HardwareInit() {
	TM_GPIO_Init();
	TM_TIMER_Init();
	TM_PWM_Init();

}


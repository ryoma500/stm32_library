/*
 * gpio.c
 *
 *  Created on: 2026/11/16
 *      Author: nakay
 */


#include "gpio.h"

static void (*GpioCallback)(uint16_t GPIO_Pin) = NULL;

void roboB_gpio_Callback(void (*callback)(uint16_t GPIO_Pin)) {
	GpioCallback = callback;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GpioCallback != NULL) GpioCallback(GPIO_Pin);
}

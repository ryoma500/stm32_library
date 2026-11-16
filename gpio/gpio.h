/*
 * gpio.h
 *
 *  Created on: 2026/11/13
 *      Author: nakay
 */

/*
 * CubeMXが生成するポートとピンの名前は~_GPIO_Portと~_Pinのため、このコードで(name)の部分にピンのユーザーラベルを入れることでピンを読み書きできる。
 */
#ifndef GPIO_H
#define GPIO_H

#include "main.h"

// GPIO ON
#define roboB_GPIO_SET(name) \
    HAL_GPIO_WritePin(name##_GPIO_Port, name##_Pin, GPIO_PIN_SET)

// GPIO OFF
#define roboB_GPIO_RESET(name) \
    HAL_GPIO_WritePin(name##_GPIO_Port, name##_Pin, GPIO_PIN_RESET)

// GPIO Toggle
#define roboB_GPIO_TogglePin(name) \
    HAL_GPIO_TogglePin(name##_GPIO_Port, name##_Pin)

// GPIO INPUT
#define roboB_GPIO_ReadPin(name) \
    HAL_GPIO_ReadPin(name##_GPIO_Port, name##_Pin)

void roboB_gpio_Callback(void (*callback)(uint16_t GPIO_Pin));

#endif

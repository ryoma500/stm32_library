/*
 * uart.h
 *
 *  Created on: 2026/11/16
 *      Author: ryoma
 */

#ifndef UART_UART_H_
#define UART_UART_H_

#include "main.h"

// UART Interrupt Transmit
HAL_StatusTypeDef roboB_uart_transmit_IT(
    UART_HandleTypeDef *huart,
    uint8_t *data,
    uint16_t size
);

// UART Interrupt Receive
HAL_StatusTypeDef roboB_uart_receive_IT(
    UART_HandleTypeDef *huart,
    uint8_t *data,
    uint16_t size
);

// UART Callback Registration
void roboB_uart_setTxCallback(
    void (*callback)(UART_HandleTypeDef *huart)
);

void roboB_uart_setRxCallback(
    void (*callback)(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
);

#endif /* UART_UART_H_ */

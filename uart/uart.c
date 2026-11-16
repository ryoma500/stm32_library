/*
 * uart.c
 *
 *  Created on: 2026/11/16
 *      Author: ryoma
 */


#include "uart.h"

// Callback function pointers
static void (*UartTxCallback)(UART_HandleTypeDef *huart) = NULL;

static void (*UartRxCallback)(
    UART_HandleTypeDef *huart,
    uint8_t *data,
    uint16_t size
) = NULL;


// UART Interrupt Transmit
HAL_StatusTypeDef roboB_uart_transmit_IT(
    UART_HandleTypeDef *huart,
    uint8_t *data,
    uint16_t size
) {
    return HAL_UART_Transmit_IT(huart, data, size);
}


// UART Interrupt Receive
HAL_StatusTypeDef roboB_uart_receive_IT(
    UART_HandleTypeDef *huart,
    uint8_t *data,
    uint16_t size
) {
    return HAL_UART_Receive_IT(huart, data, size);
}


// TX Callback Registration
void roboB_uart_setTxCallback(
    void (*callback)(UART_HandleTypeDef *huart)
) {
    UartTxCallback = callback;
}


// RX Callback Registration
void roboB_uart_setRxCallback(
    void (*callback)(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
) {
    UartRxCallback = callback;
}


// HAL TX Callback
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

    if (UartTxCallback != NULL) {
        UartTxCallback(huart);
    }

}


// HAL RX Callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

    if (UartRxCallback != NULL) {
        UartRxCallback(
            huart,
            NULL,
            0
        );
    }

}

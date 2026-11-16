/*
 * can.h
 *
 *  Created on: 2026/11/14
 *      Author: nakay
 */

#ifndef CAN_CAN_H_
#define CAN_CAN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// STM32の型番によってCANの名前を判定する。これ以外のSTM32の場合は追加してください。
#if defined(STM32F446xx)
    #define CAN1_HANDLE hcan1
	#define CAN2_HANDLE hcan2
#elif defined(STM32F303x8)
    #define CAN1_HANDLE hcan
#endif

#ifdef CAN1_HANDLE
extern CAN_HandleTypeDef CAN1_HANDLE;

extern const CAN_FilterTypeDef CAN1_FILTER_DEFAULT;

void roboB_can1_start(const CAN_FilterTypeDef *filter);

uint32_t roboB_can1_txAvailable();

void roboB_can1_transmit(uint32_t id, const uint8_t *data, uint8_t dlc, uint8_t ide, uint8_t rtr);

void roboB_can1_setReceivedCallback(void (*callback)(uint32_t id, uint8_t *data, uint8_t dlc, uint8_t ide, uint8_t rtr));

void roboB_can1_stop();
#endif

#ifdef CAN2_HANDLE
extern CAN_HandleTypeDef CAN2_HANDLE;

extern const CAN_FilterTypeDef CAN2_FILTER_DEFAULT;

void roboB_can2_start(const CAN_FilterTypeDef *filter);

uint32_t roboB_can2_txAvailable();

void roboB_can2_transmit(uint32_t id, const uint8_t *data, uint8_t dlc, uint8_t ide, uint8_t rtr);

void roboB_can2_setReceivedCallback(void (*callback)(uint32_t id, uint8_t *data, uint8_t dlc, uint8_t ide, uint8_t rtr));

void roboB_can2_stop();
#endif

#ifdef __cplusplus
}
#endif

#endif // CAN_CAN_H_

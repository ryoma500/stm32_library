/*
 * can.c
 *
 *  Created on: 2026/11/14
 *      Author: nakay
 */


#include "can/can.h"
#include <string.h>

#ifndef MIN
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#endif /* MIN */

#ifndef MAX
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))
#endif /* MAX */

#ifdef CAN1_HANDLE
const CAN_FilterTypeDef CAN1_FILTER_DEFAULT = {
	.FilterIdHigh = 0,
	.FilterIdLow = 0,
	.FilterMaskIdHigh = 0,
	.FilterMaskIdLow = 0,
	.FilterFIFOAssignment = CAN_FILTER_FIFO0,
	.FilterBank = 0,
	.FilterMode = CAN_FILTERMODE_IDMASK,
	.FilterScale = CAN_FILTERSCALE_32BIT,
	.FilterActivation = CAN_FILTER_ENABLE,
	.SlaveStartFilterBank = 14
};

static void (*can1_receivedCallback)(uint32_t id, uint8_t *data, uint8_t dlc, uint8_t ide, uint8_t rtr) = NULL;


static uint8_t can1_buffer[8];
static CAN_RxHeaderTypeDef can1_rxHeader;

static void HAL_passOK(HAL_StatusTypeDef status) {
	if(status != HAL_OK) {
		Error_Handler();
	}
}

void roboB_can1_start(const CAN_FilterTypeDef *filter) {
	HAL_passOK(HAL_CAN_ConfigFilter(&CAN1_HANDLE, filter));
	HAL_passOK(HAL_CAN_ActivateNotification(&CAN1_HANDLE, CAN_IT_RX_FIFO0_MSG_PENDING));
	HAL_passOK(HAL_CAN_Start(&CAN1_HANDLE));
}

uint32_t roboB_can1_txAvailable() {
	return HAL_CAN_GetTxMailboxesFreeLevel(&CAN1_HANDLE);
}

void roboB_can1_transmit(uint32_t id, const uint8_t *data, uint8_t dlc, uint8_t ide, uint8_t rtr) {
	CAN_TxHeaderTypeDef TxHeader = {0};
	uint32_t TxMailbox = 0;
	uint8_t txBuffer[8] = {0};

	dlc = MIN(dlc, 8);

	if(ide == CAN_ID_STD){
		TxHeader.StdId = id;
	}
	else{
		TxHeader.ExtId = id;
	}

	if (data != NULL && rtr == CAN_RTR_DATA) {
		memcpy(txBuffer, data, dlc);
	}

	TxHeader.DLC = dlc;
	TxHeader.IDE = ide;
	TxHeader.RTR = rtr;
	TxHeader.TransmitGlobalTime = DISABLE;

	while(roboB_can1_txAvailable() == 0);
	HAL_CAN_AddTxMessage(&CAN1_HANDLE, &TxHeader, txBuffer, &TxMailbox);
}

void roboB_can1_stop() {
	HAL_passOK(HAL_CAN_Stop(&CAN1_HANDLE));
}

void roboB_can1_setReceivedCallback(void (*callback)(uint32_t id, uint8_t *data, uint8_t dlc, uint8_t ide, uint8_t rtr)) {
	can1_receivedCallback = callback;
}
#endif

#ifdef CAN2_HANDLE
const CAN_FilterTypeDef CAN2_FILTER_DEFAULT = {
	.FilterIdHigh = 0,
	.FilterIdLow = 0,
	.FilterMaskIdHigh = 0,
	.FilterMaskIdLow = 0,
	.FilterFIFOAssignment = CAN_FILTER_FIFO0,
	.FilterBank = 14,
	.FilterMode = CAN_FILTERMODE_IDMASK,
	.FilterScale = CAN_FILTERSCALE_32BIT,
	.FilterActivation = CAN_FILTER_ENABLE,
	.SlaveStartFilterBank = 14
};

static void (*can2_receivedCallback)(uint32_t id, uint8_t *data, uint8_t dlc, uint8_t ide, uint8_t rtr) = NULL;

static uint8_t can2_buffer[8];

static CAN_RxHeaderTypeDef can2_rxHeader;

void roboB_can2_start(const CAN_FilterTypeDef *filter) {
	HAL_passOK(HAL_CAN_ConfigFilter(&CAN2_HANDLE, filter));
	HAL_passOK(HAL_CAN_ActivateNotification(&CAN2_HANDLE, CAN_IT_RX_FIFO0_MSG_PENDING));
	HAL_passOK(HAL_CAN_Start(&CAN2_HANDLE));
}

uint32_t roboB_can2_txAvailable() {
	return HAL_CAN_GetTxMailboxesFreeLevel(&CAN2_HANDLE);
}

void roboB_can2_transmit(uint32_t id, const uint8_t *data, uint8_t dlc, uint8_t ide, uint8_t rtr) {
	CAN_TxHeaderTypeDef TxHeader = {0};
	uint32_t TxMailbox = 0;
	uint8_t txBuffer[8] = {0};

	dlc = MIN(dlc, 8);

	if(ide == CAN_ID_STD){
		TxHeader.StdId = id;
	}
	else{
		TxHeader.ExtId = id;
	}

	if (data != NULL && rtr == CAN_RTR_DATA) {
		memcpy(txBuffer, data, dlc);
	}

	TxHeader.DLC = dlc;
	TxHeader.IDE = ide;
	TxHeader.RTR = rtr;
	TxHeader.TransmitGlobalTime = DISABLE;

	while(roboB_can2_txAvailable() == 0);
	HAL_CAN_AddTxMessage(&CAN2_HANDLE, &TxHeader, txBuffer, &TxMailbox);
}

void roboB_can2_stop() {
	HAL_passOK(HAL_CAN_Stop(&CAN2_HANDLE));
}

void roboB_can2_setReceivedCallback(void (*callback)(uint32_t id, uint8_t *data, uint8_t dlc, uint8_t ide, uint8_t rtr)) {
	can2_receivedCallback = callback;
}
#endif

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *callback_hcan)
{
#ifdef CAN1_HANDLE
	if(callback_hcan == &CAN1_HANDLE) {
		HAL_passOK(HAL_CAN_GetRxMessage(&CAN1_HANDLE, CAN_RX_FIFO0, &can1_rxHeader, can1_buffer));
		uint8_t ide = can1_rxHeader.IDE;
		uint32_t id = (ide == CAN_ID_STD) ? can1_rxHeader.StdId : can1_rxHeader.ExtId;
		uint8_t *data = can1_buffer;
		uint8_t dlc = can1_rxHeader.DLC;
		uint8_t rtr = can1_rxHeader.RTR;
		if(can1_receivedCallback != NULL) can1_receivedCallback(id, data, dlc, ide, rtr);
	}
#endif

#ifdef CAN2_HANDLE
	if(callback_hcan == &CAN2_HANDLE) {
		HAL_passOK(HAL_CAN_GetRxMessage(&CAN2_HANDLE, CAN_RX_FIFO0, &can2_rxHeader, can2_buffer));
		uint8_t ide = can2_rxHeader.IDE;
		uint32_t id = (ide == CAN_ID_STD) ? can2_rxHeader.StdId : can2_rxHeader.ExtId;
		uint8_t *data = can2_buffer;
		uint8_t dlc = can2_rxHeader.DLC;
		uint8_t rtr = can2_rxHeader.RTR;
		if(can2_receivedCallback != NULL) can2_receivedCallback(id, data, dlc, ide, rtr);
	}
#endif
}

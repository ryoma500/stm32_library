/*
 * ics.h
 *
 *  Created on: 2026/11/14
 *      Author: nakay
 */

#ifndef ICS_ICS_H_
#define ICS_ICS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define ICS_TIMEOUT 10
#define ICS_MAX_POSITION 16383
#define ICS_MAX_ID 31

extern UART_HandleTypeDef huart1;

void ics_init();
void ics_setPosition(uint8_t id, uint16_t position);
uint8_t ics_available();
void ics_setId(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* STM32_LIBRARY_ICS_ICS_H_ */

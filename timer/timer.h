/*
 * timer.h
 *
 *  Created on: 2026/11/18
 *      Author: ryoma
 */

#ifndef STM32_LIBRARY_TIMER_TIMER_H_
#define STM32_LIBRARY_TIMER_TIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

// タイマー（DWTサイクルカウンタ）の初期化
void timer_init(void);

// 時間取得関数
uint32_t timer_getUs(void); // マイクロ秒単位の経過時間を取得
uint32_t timer_getMs(void); // ミリ秒単位の経過時間を取得

// ディレイ関数
void timer_delayUs(uint32_t us); // マイクロ秒待機
void timer_delayMs(uint32_t ms); // ミリ秒待機

#ifdef __cplusplus
}
#endif

#endif /* STM32_LIBRARY_TIMER_TIMER_H_ */

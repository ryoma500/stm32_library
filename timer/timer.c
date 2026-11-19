/*
 * timer.c
 *
 *  Created on: 2026/11/18
 *      Author: ryoma
 */

#include "timer.h"

// DWT (Data Watchpoint and Trace) サイクルカウンタを使用したマイクロ秒タイマー

void timer_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t timer_getUs(void) {
    // 初期化されていない場合は自動初期化
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        timer_init();
    }

    uint32_t us_ticks = SystemCoreClock / 1000000U;
    if (us_ticks == 0) {
        us_ticks = 1;
    }
    return DWT->CYCCNT / us_ticks;
}

uint32_t timer_getMs(void) {
    return HAL_GetTick();
}

void timer_delayUs(uint32_t us) {
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        timer_init();
    }

    uint32_t us_ticks = SystemCoreClock / 1000000U;
    if (us_ticks == 0) {
        us_ticks = 1;
    }

    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * us_ticks;
    while ((DWT->CYCCNT - start) < ticks);
}

void timer_delayMs(uint32_t ms) {
    HAL_Delay(ms);
}

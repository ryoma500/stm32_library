#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include <stdint.h>

/*
 * エンコーダの状態を保存する構造体
 */
typedef struct
{
    /*
     * A/B相をエンコーダモードでカウントしているTIM
     */
    TIM_HandleTypeDef *htim;

    /*
     * TIMカウンタの周期
     *
     * htim->Init.Period + 1 から
     * ライブラリが自動取得するので気にしなくていい
     */
    uint32_t counter_period;

    /*
     * エンコーダ1回転あたりのカウント数
     */
    uint32_t counts_per_rev;

    /*
     * Encoder_Update()で使用する
     * 前回のTIMカウンタ値
     */
    uint16_t last_counter;

    /*
     * 累積エンコーダ位置
     *
     * A/B相による移動量を累積する。
     * Z相検出時には、この値を補正する。
     */
    int32_t position;

    /*
     * 最初に検出したZ相の絶対基準位置
     *
     * 初回Z相で設定した後は変更しない。
     */
    int32_t index_position;

    /*
     * Z相を一度でも検出したか
     *
     * 0: 未検出
     * 1: 検出済み
     */
    volatile uint8_t index_detected;

} Encoder_HandleTypeDef;


/*
 * エンコーダを初期化する(最初に構造体を宣言し、そのアドレスをこの関数に渡す)
 *
 * htim:
 *     A/B相をエンコーダモードに設定したTIM
 *
 * counts_per_rev:
 *     エンコーダ1回転あたりのカウント数
 *
 * counter_period:
 *     htim->Init.Period + 1
 *     から自動取得する。
 */
void Encoder_Init(Encoder_HandleTypeDef *encoder,
                  TIM_HandleTypeDef *htim,
                  uint32_t counts_per_rev);


/*
 * A/B相によるエンコーダ位置を更新する
 *
 * TIMのCNTの前回値との差分から
 * 移動量を計算し、positionを更新する。
 */
void Encoder_Update(Encoder_HandleTypeDef *encoder);


/*
 * Z相（インデックス相）が入力されたときに呼び出す。
 *
 * 初回のZ相:
 *     その時点のpositionを
 *     index_positionとして保存する。
 *
 * 2回目以降:
 *     index_positionからの距離を確認する。
 *
 *     半周未満:
 *         positionをindex_positionへ補正する。
 *         1回転とは判定しない。
 *
 *     半周以上、正方向:
 *         positionを
 *         index_position + counts_per_rev
 *         に補正する。
 *
 *     半周以上、逆方向:
 *         positionを
 *         index_position - counts_per_rev
 *         に補正する。
 *
 * index_positionは初回Z相以降変更しない。
 *
 * TIMのCNTは変更しない。
 */
void Encoder_Z_Callback(Encoder_HandleTypeDef *encoder);


/*
 * 現在の累積エンコーダ位置を取得する
 */
int32_t Encoder_GetPosition(Encoder_HandleTypeDef *encoder);


/*
 * 現在の回転数を取得する
 */
int32_t Encoder_GetRevolution(Encoder_HandleTypeDef *encoder);


/*
 * 最初に検出したZ相の絶対基準位置を取得する
 *
 * 初回Z相で設定された後は変更されない。
 */
int32_t Encoder_GetIndexPosition(Encoder_HandleTypeDef *encoder);


/*
 * 最初のZ相位置を基準とした現在位置を取得する
 *
 * position - index_position
 * を返す。
 */
int32_t Encoder_GetPositionFromIndex(
    Encoder_HandleTypeDef *encoder);


/*
 * Z相が検出されたか確認する
 *
 * 戻り値:
 *     0: 未検出
 *     1: 検出済み
 */
uint8_t Encoder_IsIndexDetected(
    Encoder_HandleTypeDef *encoder);


/*
 * エンコーダ位置をソフトウェア上でリセットする。
 */
void Encoder_ResetPosition(
    Encoder_HandleTypeDef *encoder);

#endif /* ENCODER_H */

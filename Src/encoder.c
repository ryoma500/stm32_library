#include "encoder.h"


/*
 * エンコーダを初期化する
 */
void Encoder_Init(Encoder_HandleTypeDef *encoder,
                  TIM_HandleTypeDef *htim,
                  uint32_t counts_per_rev)
{
    if (encoder == NULL || htim == NULL)
    {
        return;
    }

    encoder->htim = htim; // 使用するTIMを保存
    encoder->counter_period = (uint32_t)htim->Init.Period + 1U; // TIMカウンタの周期を取得
    encoder->counts_per_rev = counts_per_rev; // 1回転あたりのカウント数
    encoder->last_counter = (uint16_t)__HAL_TIM_GET_COUNTER(htim); // 現在のTIM CNTを取得し、次回のEncoder_Update()の比較基準にする。
    encoder->position = 0; // ソフトウェア位置を初期化
    encoder->index_position = 0; // Z相の位置を初期化
    encoder->index_detected = 0U; // Z相の発見を初期化
}


/*
 * A/B相によるエンコーダ位置を更新する
 */
void Encoder_Update(Encoder_HandleTypeDef *encoder)
{
    uint16_t current_counter;
    uint32_t raw_diff;
    int32_t diff;

    if (encoder == NULL || encoder->htim == NULL)
    {
        return;
    }

    if (encoder->counter_period == 0U)
    {
        return;
    }

    current_counter = (uint16_t)__HAL_TIM_GET_COUNTER(encoder->htim); // 現在のTIM CNTを取得

    /*
     * 前回のCNTから今回のCNTまでの差分を計算する。
     *
     * まず正方向に進んだ場合の差分として計算し、
     * その後、半周期を境界として正負を判定する。
     */
    if (current_counter >= encoder->last_counter)
    {
        raw_diff =
            (uint32_t)current_counter -
            (uint32_t)encoder->last_counter;
    }
    else
    {
        /*
         * CNTが0をまたいだ場合
         */
        raw_diff =
            encoder->counter_period -
            (uint32_t)encoder->last_counter +
            (uint32_t)current_counter;
    }

    /*
     * カウンタ周期の半分以上の差分は、
     * 逆方向に移動したものとして扱う。
     *
     * 前回Encoder_Update()から今回までの移動量が
     * カウンタ周期の半分未満であることを前提とする。
     */
    if (raw_diff >= (encoder->counter_period / 2U))
    {
        diff =
            (int32_t)raw_diff -
            (int32_t)encoder->counter_period;
    }
    else
    {
        diff = (int32_t)raw_diff;
    }

    encoder->last_counter = current_counter; // 次回の比較用に現在値を保存
    encoder->position += diff; // 累積位置を更新
}


/*
 * Z相が入力されたときに呼び出す
 */
void Encoder_Z_Callback(Encoder_HandleTypeDef *encoder)
{
    int32_t distance_from_index;
    int32_t half_counts;
    int32_t revolution_from_index;
    int32_t remainder;

    if (encoder == NULL)
    {
        return;
    }

    if (encoder->counts_per_rev == 0U)
    {
        return;
    }

    /* 初回のZ相で基準位置を保存 */
    if (encoder->index_detected == 0U)
    {
        encoder->index_position = encoder->position;
        encoder->index_detected = 1U;
        return;
    }

    /* 基準位置からの距離 */
    distance_from_index =
        encoder->position - encoder->index_position;

    half_counts =
        (int32_t)(encoder->counts_per_rev / 2U);

    /* 基準から何回転分進んだかを計算 */
    revolution_from_index =
        distance_from_index /
        (int32_t)encoder->counts_per_rev;

    /* 割り切れなかった端数 */
    remainder =
        distance_from_index %
        (int32_t)encoder->counts_per_rev;

    /*
     * 端数が半周以上なら、
     * 最も近い回転位置へ繰り上げ・繰り下げる。
     */
    if (remainder >= half_counts)
    {
        revolution_from_index++;
    }
    else if (remainder <= -half_counts)
    {
        revolution_from_index--;
    }

    /* 該当する回転位置へ補正 */
    encoder->position =
        encoder->index_position +
        revolution_from_index *
        (int32_t)encoder->counts_per_rev;
}


/*
 * 現在の累積エンコーダ位置を取得する
 */
int32_t Encoder_GetPosition(Encoder_HandleTypeDef *encoder)
{
    if (encoder == NULL)
    {
        return 0;
    }

    return encoder->position;
}


/*
 * 現在の回転数を取得する
 *
 * position / counts_per_rev
 * で計算する。
 */
int32_t Encoder_GetRevolution(Encoder_HandleTypeDef *encoder)
{
    if (encoder == NULL)
    {
        return 0;
    }

    if (encoder->counts_per_rev == 0U)
    {
        return 0;
    }

    return encoder->position /
           (int32_t)encoder->counts_per_rev;
}


/*
 * 最初に検出したZ相の絶対基準位置を取得する
 */
int32_t Encoder_GetIndexPosition(Encoder_HandleTypeDef *encoder)
{
    if (encoder == NULL)
    {
        return 0;
    }

    return encoder->index_position;
}


/*
 * 最初のZ相位置を基準とした現在位置を取得する
 */
int32_t Encoder_GetPositionFromIndex(
    Encoder_HandleTypeDef *encoder)
{
    if (encoder == NULL)
    {
        return 0;
    }

    return encoder->position -
           encoder->index_position;
}


/*
 * Z相が検出されたか確認する
 */
uint8_t Encoder_IsIndexDetected(
    Encoder_HandleTypeDef *encoder)
{
    if (encoder == NULL)
    {
        return 0U;
    }

    return encoder->index_detected;
}


/*
 * エンコーダ位置をソフトウェア上でリセットする
 *
 * 次回のZ相が新しい絶対基準になる。
 */
void Encoder_ResetPosition(
    Encoder_HandleTypeDef *encoder)
{
    if (encoder == NULL)
    {
        return;
    }

    encoder->position = 0; // 現在位置をリセット
    encoder->index_position = 0; // Z相基準をリセット
    encoder->index_detected = 0U; // 次回のZ相を初回Z相として扱う
}

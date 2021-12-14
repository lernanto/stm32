/**
 * @brief 
 * @author 黄艺华 (lernanto@foxmail.com)
 */

#include <string.h>

#include "dsp.h"


int dsp_lwlr_init(
    size_t feature_num,
    size_t label_num,
    float32_t *var,
    float32_t *xwx,
    float32_t *xwy
)
{
    *var = 1.0f;
    memset(xwx, 0, sizeof(float32_t) * feature_num * feature_num);
    memset(xwy, 0, sizeof(float32_t) * feature_num * label_num);
    return 1;
}

int dsp_lwlr_add_sample(
    size_t feature_num,
    size_t label_num,
    const float32_t *x,
    const float32_t *y,
    float32_t decay,
    float32_t *var,
    float32_t *xwx,
    float32_t *xwy
)
{
    float32_t w = 0.0f;

    *var = *var * decay + 1;
    w = 1.0f / *var;

    /* TODO: 使用 CMSIS DSP 库加快矩阵运算 */
    for (size_t i = 0; i < feature_num; ++i)
    {
        for (size_t j = 0; j < feature_num; ++j)
        {
            xwx[i * feature_num + j] = (1 - w) * xwx[i * feature_num + j]
                + w * x[i] * x[j];
        }

        for (size_t j = 0; j < label_num; ++j)
        {
            xwy[i * label_num + j] = (1 - w) * xwy[i * label_num + j]
                + w * x[i] * y[j];
        }
    }

    return 1;
}

int dsp_lwlr_estimate(
    size_t feature_num,
    size_t label_num,
    const float32_t *xwx,
    const float32_t *wxy,
    float32_t l2,
    float32_t *theta
)
{
    /* TODO: 解线性方程组 (XWX + l2 * I) * theta = XWy */
    return 1;
}

int dsp_lwlr_predict(
    size_t feature_num,
    size_t label_num,
    const float32_t *theta,
    const float32_t *x,
    float32_t *y
)
{
    /* TODO: 使用 CMSIS DSP 库加快矩阵运算 */
    memset(y, 0, sizeof(float32_t) * label_num);

    for (size_t i = 0; i < feature_num; ++i)
    {
        for (size_t j = 0; j < label_num; ++j)
        {
            y[j] += x[i] * theta[i * label_num + j];
        }
    }

    return 1;
}

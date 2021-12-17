/**
 * @brief 信号处理功能函数
 * @author 黄艺华 (lernanto@foxmail.com)
 */

#ifndef _DSP_H
#define _DSP_H

#include "arm_math.h"


extern int dsp_linear_regression_init(
    size_t feature_num,
    size_t target_num,
    float32_t *xwx,
    float32_t *ywx
);

extern int dsp_linear_regression_add_sample(
    size_t feature_num,
    size_t target_num,
    const float32_t *x,
    const float32_t *y,
    float32_t weight,
    float32_t *xwx,
    float32_t *ywx
);

extern int dsp_linear_regression_solve(
    size_t feature_num,
    size_t target_num,
    float32_t *xwx,
    const float32_t *ywx,
    float32_t l2,
    float32_t *param_buf,
    float32_t *param_inv,
    float32_t *coef
);

extern int dsp_linear_regression_predict(
    size_t feature_num,
    size_t target_num,
    const float32_t *coef,
    const float32_t *x,
    float32_t *y
);

extern int dsp_linear_regression1_init(
    float32_t *ex,
    float32_t *ey,
    float32_t *ex2,
    float32_t *exy
);

extern int dsp_linear_regression1_add_sample(
    float32_t x,
    float32_t y,
    float32_t weight,
    float32_t *ex,
    float32_t *ey,
    float32_t *ex2,
    float32_t *exy
);

extern int dsp_linear_regression1_solve(
    float32_t ex,
    float32_t ey,
    float32_t ex2,
    float32_t exy,
    float32_t l2,
    float32_t *coef,
    float32_t *bias
);

extern int dsp_linear_regression1_predict(
    float32_t coef,
    float32_t bias,
    float32_t x,
    float32_t *y
);

extern float32_t dsp_linear_regression_decay(float32_t var, float32_t decay);

#endif  /* _DSP_H */

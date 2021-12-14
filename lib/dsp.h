/**
 * @brief 信号处理功能函数
 * @author 黄艺华 (lernanto@foxmail.com)
 */

#ifndef _DSP_H
#define _DSP_H


typedef float float32_t;


extern int dsp_lwlr_init(
    size_t feature_num,
    size_t label_num,
    float32_t *var,
    float32_t *xwx,
    float32_t *xwy
);

extern int dsp_lwlr_add_sample(
    size_t feature_num,
    size_t label_num,
    const float32_t *x,
    const float32_t *y,
    float32_t decay,
    float32_t *var,
    float32_t *xwx,
    float32_t *xwy
);

extern int dsp_lwlr_estimate(
    size_t feature_num,
    size_t label_num,
    const float32_t *xwx,
    const float32_t *wxy,
    float32_t l2,
    float32_t *theta
);

extern int dsp_lwlr_predict(
    size_t feature_num,
    size_t label_num,
    const float32_t *theta,
    const float32_t *x,
    float32_t *y
);

#endif  /* _DSP_H */

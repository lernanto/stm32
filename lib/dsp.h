/**
 * @brief 数字信号处理功能函数
 * @author 黄艺华 (lernanto@foxmail.com)
 *
 * 当前只包含增量线性回归功能
 * 增量是指每次读入一条样本，在读入任意条样本之后，随时可以求解线性回归模型，
 * 解出的模型考虑了迄今为止读入的所有样本。继续读入新的样本，重复求解过程可以得到更新的模型，
 * 这个过程可以一直持续下去。这种求解过程适合样本随着时间采集，采集的同时又要即时求解的情况。
 *
 * 读入样本的时候可以为先前样本指定衰减系数，为当前样本指定权重
 *  - 如指定衰减系数 decay = 1，样本权重 weight = 1，则相当于普通线性回归
 *  - 指定 decay = 1，weight != 1，相当于加权线性回归
 *  - 指定 decay < 1，weight = 1，相当于局部加权线性回归
 */

#ifndef _DSP_H
#define _DSP_H

#include "arm_math.h"


/**
 * @brief 初始化线性回归模型的先验协方差矩阵为对角阵
 *
 * @param feature_num 样本特征数
 * @param target_num 预测的目标数
 * @param precision 增量求解线性回归过程中保存的精度矩阵，为协方差矩阵的逆
 * 维度为 (feature_num + 1) * (feature_num + 1)
 * @param pre_mean 增量求解线性回归过程中保存的精度矩阵乘以期望矩阵
 * 维度为 (feature_num + 1) * target_num
 * @param mean 指定模型先验分布的期望，为空时指定期望为全0
 * @param diag 指定模型先验精度矩阵的对角线元素
 * @return 成功返回非0，失败返回0
 */
extern int dsp_linear_regression_init_diag(
    size_t feature_num,
    size_t target_num,
    float32_t *precision,
    float32_t *pre_mean,
    const float32_t *mean,
    const float32_t *diag
);

/**
 * @brief 初始化线性回归模型的先验协方差矩阵为对角阵
 *
 * @param feature_num 样本特征数
 * @param target_num 预测的目标数
 * @param precision 增量求解线性回归过程中保存的精度矩阵，为协方差矩阵的逆
 * 维度为 (feature_num + 1) * (feature_num + 1)
 * @param pre_mean 增量求解线性回归过程中保存的精度矩阵乘以期望矩阵
 * 维度为 (feature_num + 1) * target_num
 * @param mean 指定模型先验分布的期望，为空时指定期望为全0
 * @param scale 指定模型先验京都矩阵的对角线元素为相等的值
 * @return 成功返回非0，失败返回0
 */
extern int dsp_linear_regression_init_scale(
    size_t feature_num,
    size_t target_num,
    float32_t *precision,
    float32_t *pre_mean,
    const float32_t *mean,
    float32_t scale
);

/**
 * @brief 初始化线性回归模型的先验分布为标准高斯分布
 *
 * @param feature_num 样本特征数
 * @param target_num 预测的目标数
 * @param precision 增量求解线性回归过程中保存的精度矩阵，为协方差矩阵的逆
 * 维度为 (feature_num + 1) * (feature_num + 1)
 * @param pre_mean 增量求解线性回归过程中保存的精度矩阵乘以期望矩阵
 * 维度为 (feature_num + 1) * target_num
 * @return 成功返回非0，失败返回0
 */
static inline int dsp_linear_regression_init(
    size_t feature_num,
    size_t target_num,
    float32_t *precision,
    float32_t *pre_mean
)
{
    return dsp_linear_regression_init_scale(
        feature_num,
        target_num,
        precision,
        pre_mean,
        NULL,
        1.0f
    );
}

static inline void dsp_linear_regression_uni_init(
    float32_t precision[4],
    float32_t pre_mean[2]
)
{
    dsp_linear_regression_init(1, 1, precision, pre_mean);
}

static inline void dsp_linear_regression_uni_init_diag(
    float32_t precision[4],
    float32_t pre_mean[2],
    const float32_t mean[2],
    const float32_t diag[2]
)
{
    dsp_linear_regression_init_diag(1, 1, precision, pre_mean, mean, diag);
}

static inline void dsp_linear_regression_uni_init_scale(
    float32_t precision[4],
    float32_t pre_mean[2],
    const float32_t mean[2],
    float32_t scale
)
{
    dsp_linear_regression_init_scale(1, 1, precision, pre_mean, mean, scale);
}

extern void dsp_linear_regression_uni_update(
    float32_t x,
    float32_t y,
    float32_t decay,
    float32_t weight,
    float32_t precision[4],
    float32_t pre_mean[2]
);

extern int dsp_linear_regression_uni_solve(
    float32_t precision[4],
    const float32_t pre_mean[2],
    float32_t mean[2],
    float32_t cov[4]
);

extern float32_t dsp_linear_regression_uni_predict(
    const float32_t mean[2],
    float32_t x
);

extern float32_t dsp_linear_regression_uni_predict_prec(
    const float32_t precision[4],
    float32_t x
);

static inline void dsp_linear_regression_mul_init(
    size_t feature_num,
    float32_t *precision,
    float32_t *pre_mean
)
{
    dsp_linear_regression_init(feature_num, 1, precision, pre_mean);
}

extern void dsp_linear_regression_mul_update(
    size_t feature_num,
    const float32_t *x,
    float32_t y,
    float32_t decay,
    float32_t weight,
    float32_t *precision,
    float32_t *pre_mean
);

extern int dsp_linear_regression_mul_solve(
    size_t feature_num,
    float32_t *precision,
    const float32_t *pre_mean,
    float32_t *mean,
    float32_t *cov
);

extern float32_t dsp_linear_regression_mul_predict(
    size_t feature_num,
    const float32_t *mean,
    const float32_t *x
);

extern float32_t dsp_linear_regression_mul_predict_prec(
    size_t feature_num,
    const float32_t *precision,
    const float32_t *x
);

static inline void dsp_linear_regression_mt_init(
    size_t feature_num,
    size_t target_num,
    float32_t *precision,
    float32_t *pre_mean
)
{
    dsp_linear_regression_init(feature_num, target_num, precision, pre_mean);
}

extern void dsp_linear_regression_mt_update(
    size_t feature_num,
    size_t target_num,
    const float32_t *x,
    const float32_t *y,
    float32_t decay,
    float32_t weight,
    float32_t *precision,
    float32_t *pre_mean
);

extern int dsp_linear_regression_mt_solve(
    size_t feature_num,
    size_t target_num,
    float32_t *precision,
    const float32_t *pre_mean,
    float32_t *mean,
    float32_t *cov
);

extern void dsp_linear_regression_mt_predict(
    size_t feature_num,
    size_t target_num,
    const float32_t *mean,
    const float32_t *x,
    float32_t *y
);

/**
 * @brief 向线性回归读入一个样本
 *
 * @param feature_num 样本特征向量维数
 * @param target_num 预测的目标向量维数
 * @param x 样本特征
 * @param y 样本目标
 * @param decay 精度矩阵衰减系数
 * @param weight 样本权重
 * 先前样本的权重会由于加入更多样本而衰减，所以这个权重并不等于加权线性回归的权重
 * @param precision 先前保存的统计量矩阵，读入样本后会更新
 * @param pre_mean 先前保存的统计量矩阵，读入样本后会更新
 * @return 成功返回非0，失败返回0
 */
static inline int dsp_linear_regression_update(
    size_t feature_num,
    size_t target_num,
    const float32_t *x,
    const float32_t *y,
    float32_t decay,
    float32_t weight,
    float32_t *precision,
    float32_t *pre_mean
)
{
    if (1 == target_num)
    {
        if (1 == feature_num)
        {
            dsp_linear_regression_uni_update(*x, *y, decay, weight, precision, pre_mean);
        }
        else
        {
            dsp_linear_regression_mul_update(
                feature_num,
                x,
                *y,
                decay,
                weight,
                precision,
                pre_mean
            );
        }
    }
    else
    {
        dsp_linear_regression_mt_update(
            feature_num,
            target_num,
            x,
            y,
            decay,
            weight,
            precision,
            pre_mean
        );
    }

    return 1;
}

/**
 * @brief 根据迄今为止读入的样本，求解线性回归模型
 *
 * @param feature_num 样本特征向量维数
 * @param target_num 预测的目标向量维数
 * @param precision 先前保存的统计量矩阵
 * @param pre_mean 先前保存的统计量矩阵
 * @param mean 保存求解得到的模型期望
 * @param cov 保存求解得到的模型协方差矩阵
 * @return 成功返回非0，失败返回0
 */
static inline int dsp_linear_regression_solve(
    size_t feature_num,
    size_t target_num,
    float32_t *precision,
    const float32_t *pre_mean,
    float32_t *mean,
    float32_t *cov
)
{
    if (1 == target_num)
    {
        if (1 == feature_num)
        {
            return dsp_linear_regression_uni_solve(precision, pre_mean, mean, cov);
        }
        else
        {
            return dsp_linear_regression_mul_solve(
                feature_num,
                precision,
                pre_mean,
                mean,
                cov
            );
        }
    }
    else
    {
        return dsp_linear_regression_mt_solve(
            feature_num,
            target_num,
            precision,
            pre_mean,
            mean,
            cov
        );
    }
}

/**
 * @brief 使用解得的线性回归模型预测目标
 *
 * @param feature_num 样本特征向量维数
 * @param target_num 预测的目标向量维数
 * @param mean 线性回归模型期望
 * @param x 输入特征向量
 * @param y 保存预测得到的目标向量
 * @return 成功返回非0，失败返回0
 */
static inline int dsp_linear_regression_predict(
    size_t feature_num,
    size_t target_num,
    const float32_t *mean,
    const float32_t *x,
    float32_t *y
)
{
    if (1 == target_num)
    {
        if (1 == feature_num)
        {
            *y = dsp_linear_regression_uni_predict(mean, *x);
        }
        else
        {
            *y = dsp_linear_regression_mul_predict(feature_num, mean, x);
        }
    }
    else
    {
        dsp_linear_regression_mt_predict(feature_num, target_num, mean, x, y);
    }

    return 1;
}

/**
 * @brief 使用解得的线性回归模型预测目标的精度
 *
 * @param feature_num 样本特征向量维数
 * @param precision 线性回归模型的精度矩阵
 * @param x 输入特征向量
 * @return 预测目标的精度
 */
static inline float32_t dsp_linear_regression_predict_prec(
    size_t feature_num,
    const float32_t *precision,
    const float32_t *x
)
{
    if (1 == feature_num)
    {
        return dsp_linear_regression_uni_predict_prec(precision, *x);
    }
    else
    {
        return dsp_linear_regression_mul_predict_prec(feature_num, precision, x);
    }
}

#endif  /* _DSP_H */

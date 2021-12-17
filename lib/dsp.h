/**
 * @brief 数字信号处理功能函数
 * @author 黄艺华 (lernanto@foxmail.com)
 *
 * 当前只包含增量线性回归功能
 * 增量是指每次读入一条样本，读入任意数量的样本后，都可以求解线性回归模型，
 * 解出的模型考虑了迄今为止读入的所有样本。继续读入新的样本，重复求解过程可以得到更新的模型，
 * 这个过程可以一直持续下去。这种求解过程适合样本随着时间采集，采集的同时又要即时求解的情况。
 *
 * 读入样本的时候可以为样本指定权重
 *  - 如指定权重 w = 1 / n，其中 n 为迄今为止读入的样本数量（包括当前样本），则相当于普通线性回归
 *  - 指定 w = 1 / (1 / w * d + 1)，其中 d 为 (0, 1] 的衰减系数，相当于局部加权线性回归
 *  - 指定任意 w 属于 (0, 1)，相当于普通加权线性回归，w 需要根据样本权重预先计算
 */

#ifndef _DSP_H
#define _DSP_H

#include "arm_math.h"


/**
 * @brief 初始化线性回归模型
 *
 * @param feature_dim 样本特征向量维数
 * @param target_dim 预测的目标向量维数
 * @param xwx 增量求解线性回归需要保存的统计量矩阵，X^T * W * X
 * @param ywx 增量求解线性回归需要保存的统计量矩阵，Y^T * W * X
 * @return 成功返回非0，失败返回0
 */
extern int dsp_linear_regression_init(
    size_t feature_dim,
    size_t target_dim,
    float32_t *xwx,
    float32_t *ywx
);

/**
 * @brief 向线性回归读入一个样本
 *
 * @param feature_dim 样本特征向量维数
 * @param target_dim 预测的目标向量维数
 * @param x 样本特征
 * @param y 样本目标
 * @param weight 样本权重，(0, 1)
 * 先前样本的权重会由于加入更多样本而衰减，所以这个权重并不等于加权线性回归的权重
 * @param xwx 先前保存的统计量矩阵，读入样本后会更新
 * @param ywx 先前保存的统计量矩阵，读入样本后会更新
 * @return 成功返回非0，失败返回0
 */
extern int dsp_linear_regression_add_sample(
    size_t feature_dim,
    size_t target_dim,
    const float32_t *x,
    const float32_t *y,
    float32_t weight,
    float32_t *xwx,
    float32_t *ywx
);

/**
 * @brief 根据迄今为止读入的样本，求解线性回归模型
 *
 * @param feature_dim 样本特征向量维数
 * @param target_dim 预测的目标向量维数
 * @param xwx 先前保存的统计量矩阵
 * @param ywx 先前保存的统计量矩阵
 * @param l2 L2 正则化系数，[0, +inf)
 * @param param_buf 由调用者提供，供临时存储中间结果矩阵的空间，当 l2 = 0 时可以为 NULL
 * @param param_inv 保存 (X^T * W * X)^-1 的计算结果
 * @param coef 保存求解得到的模型系数
 * @return 成功返回非0，失败返回0
 */
extern int dsp_linear_regression_solve(
    size_t feature_dim,
    size_t target_dim,
    float32_t *xwx,
    const float32_t *ywx,
    float32_t l2,
    float32_t *param_buf,
    float32_t *param_inv,
    float32_t *coef
);

/**
 * @brief 使用解得的线性回归模型预测目标
 *
 * @param feature_dim 样本特征向量维数
 * @param target_dim 预测的目标向量维数
 * @param coef 线性回归模型系数
 * @param x 输入特征向量
 * @param y 保存预测得到的目标向量
 * @return 成功返回非0，失败返回0
 */
extern int dsp_linear_regression_predict(
    size_t feature_dim,
    size_t target_dim,
    const float32_t *coef,
    const float32_t *x,
    float32_t *y
);

/**
 * @brief 初始化一元线性回归模型
 *
 * @param ex 增量求解过程中用来保存特征的期望
 * @param ey 增量求解过程中用来保存目标的期望
 * @param ex2 增量求解过程中用来保存 x^2 的期望
 * @param exy 增量求解过程中用来保存 x * y 的期望
 * @return 成功返回非0，失败返回0
 *
 * @note 是线性回归模型在输入和输出都是一元的情况下的简便函数
 */
extern int dsp_linear_regression1_init(
    float32_t *ex,
    float32_t *ey,
    float32_t *ex2,
    float32_t *exy
);

/**
 * @brief 一元线性回归模型读入一个样本
 *
 * @param x 样本特征
 * @param y 样本目标
 * @param weight 样本权重
 * @param ex 保存迄今为止 x 的期望
 * @param ey 保存迄今为止 y 的期望
 * @param ex2 保存迄今为止 x^2 的期望
 * @param exy 保存迄今为止 x * y 的期望
 * @return 成功返回非0，失败返回0
 */
extern int dsp_linear_regression1_add_sample(
    float32_t x,
    float32_t y,
    float32_t weight,
    float32_t *ex,
    float32_t *ey,
    float32_t *ex2,
    float32_t *exy
);

/**
 * @brief 求解一元线性回归模型
 *
 * @param ex x 的期望
 * @param ey y 的期望
 * @param ex2 x^2 的期望
 * @param exy x * y 的期望
 * @param l2 L2 正则化系数，[0, +inf)
 * @param coef 求解得到的特征系数
 * @param bias 求解得到的常数偏置
 * @return 成功返回非0，失败返回0
 */
extern int dsp_linear_regression1_solve(
    float32_t ex,
    float32_t ey,
    float32_t ex2,
    float32_t exy,
    float32_t l2,
    float32_t *coef,
    float32_t *bias
);

/**
 * @brief 使用解得的一元线性回归模型预测目标
 *
 * @param coef 模型特征系数
 * @param bias 模型常数偏置
 * @param x 输入特征
 * @param y 保存预测得到的目标
 * @return 成功返回非0，失败返回0
 */
extern int dsp_linear_regression1_predict(
    float32_t coef,
    float32_t bias,
    float32_t x,
    float32_t *y
);

/**
 * @brief 为局部加权线性回归更新样本权重的简便函数
 *
 * @return 权重倒数的初始值，和先验方差成正比
 */
static inline float32_t dsp_linear_regression_init_weight(void)
{
    return 0.0f;
}

/**
 * @brief 根据衰减系数更新样本权重
 *
 * @param var 保存权重的导数，和先验方差成正比
 * @param decay 衰减系数，(0, 1]，越大表示旧样本的权重越高，特殊取值下
 *  - 当取0时，相当于只使用最新的一条样本
 *  - 当取1时，相当于新旧样本同等权重
 *  - 当取 > 1 时，相当于旧样本比新样本权重更高
 * @return 下一条样本权重
 */
static inline float32_t dsp_linear_regression_update_weight(
    float32_t *var,
    float32_t decay
)
{
    *var = *var * decay + 1.0f;
    return 1.0f / *var;
}

#endif  /* _DSP_H */

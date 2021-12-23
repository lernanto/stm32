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
 * @param feature_num 样本特征数
 * @param target_num 预测的目标数
 * @param xwx 增量求解线性回归需要保存的统计量矩阵，X^T * W * X，
 * 其中 X 为增加常数项1维度后的特征矩阵，XWX 维度为 (feature_num + 1) * (feature_num + 1)
 * @param xwy 增量求解线性回归需要保存的统计量矩阵，Y^T * W * X
 * 维度为 (feature_num + 1) * target_num
 * @return 成功返回非0，失败返回0
 */
extern int dsp_linear_regression_init(
    size_t feature_num,
    size_t target_num,
    float32_t *xwx,
    float32_t *xwy
);

static inline void dsp_linear_regression_uni_init(
    float32_t xwx[4],
    float32_t xwy[2]
)
{
    dsp_linear_regression_init(1, 1, xwx, xwy);
}

extern void dsp_linear_regression_uni_update(
    float32_t x,
    float32_t y,
    float32_t weight,
    float32_t xwx[4],
    float32_t xwy[2]
);

extern int dsp_linear_regression_uni_solve(
    const float32_t xwx[4],
    const float32_t xwy[2],
    float32_t l2,
    float32_t coef[2]
);

extern float32_t dsp_linear_regression_uni_predict(
    const float32_t *coef,
    float32_t x
);

static inline void dsp_linear_regression_mul_init(
    size_t feature_num,
    float32_t *xwx,
    float32_t *xwy
)
{
    dsp_linear_regression_init(feature_num, 1, xwx, xwy);
}

extern void dsp_linear_regression_mul_update(
    size_t feature_num,
    const float32_t *x,
    float32_t y,
    float32_t weight,
    float32_t *xwx,
    float32_t *xwy
);

extern int dsp_linear_regression_mul_solve(
    size_t feature_num,
    const float32_t *xwx,
    const float32_t *xwy,
    float32_t l2,
    float32_t *coef
);

extern float32_t dsp_linear_regression_mul_predict(
    size_t feature_num,
    const float32_t *coef,
    const float32_t *x
);

static inline void dsp_linear_regression_mt_init(
    size_t feature_num,
    size_t target_num,
    float32_t *xwx,
    float32_t *xwy
)
{
    dsp_linear_regression_init(feature_num, target_num, xwx, xwy);
}

extern void dsp_linear_regression_mt_update(
    size_t feature_num,
    size_t target_num,
    const float32_t *x,
    const float32_t *y,
    float32_t weight,
    float32_t *xwx,
    float32_t *xwy
);

extern int dsp_linear_regression_mt_solve(
    size_t feature_num,
    size_t target_num,
    const float32_t *xwx,
    const float32_t *xwy,
    float32_t l2,
    float32_t *coef
);

extern void dsp_linear_regression_mt_predict(
    size_t feature_num,
    size_t target_num,
    const float32_t *coef,
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
 * @param weight 样本权重，(0, 1)
 * 先前样本的权重会由于加入更多样本而衰减，所以这个权重并不等于加权线性回归的权重
 * @param xwx 先前保存的统计量矩阵，读入样本后会更新
 * @param xwy 先前保存的统计量矩阵，读入样本后会更新
 * @return 成功返回非0，失败返回0
 */
static inline int dsp_linear_regression_update(
    size_t feature_num,
    size_t target_num,
    const float32_t *x,
    const float32_t *y,
    float32_t weight,
    float32_t *xwx,
    float32_t *xwy
)
{
    if (1 == target_num)
    {
        if (1 == feature_num)
        {
            dsp_linear_regression_uni_update(*x, *y, weight, xwx, xwy);
        }
        else
        {
            dsp_linear_regression_mul_update(feature_num, x, *y, weight, xwx, xwy);
        }
    }
    else
    {
        dsp_linear_regression_mt_update(
            feature_num,
            target_num,
            x,
            y,
            weight,
            xwx,
            xwy
        );
    }

    return 1;
}

/**
 * @brief 根据迄今为止读入的样本，求解线性回归模型
 *
 * @param feature_num 样本特征向量维数
 * @param target_num 预测的目标向量维数
 * @param xwx 先前保存的统计量矩阵
 * @param xwy 先前保存的统计量矩阵
 * @param l2 L2 正则化系数，[0, +inf)
 * @param coef 保存求解得到的模型系数
 * @return 成功返回非0，失败返回0
 */
static inline int dsp_linear_regression_solve(
    size_t feature_num,
    size_t target_num,
    const float32_t *xwx,
    const float32_t *xwy,
    float32_t l2,
    float32_t *coef
)
{
    if (1 == target_num)
    {
        if (1 == feature_num)
        {
            return dsp_linear_regression_uni_solve(xwx, xwy, l2, coef);
        }
        else
        {
            return dsp_linear_regression_mul_solve(feature_num, xwx, xwy, l2, coef);
        }
    }
    else
    {
        return dsp_linear_regression_mt_solve(
            feature_num,
            target_num,
            xwx,
            xwy,
            l2,
            coef
        );
    }
}

/**
 * @brief 使用解得的线性回归模型预测目标
 *
 * @param feature_num 样本特征向量维数
 * @param target_num 预测的目标向量维数
 * @param coef 线性回归模型系数
 * @param x 输入特征向量
 * @param y 保存预测得到的目标向量
 * @return 成功返回非0，失败返回0
 */
static inline int dsp_linear_regression_predict(
    size_t feature_num,
    size_t target_num,
    const float32_t *coef,
    const float32_t *x,
    float32_t *y
)
{
    if (1 == target_num)
    {
        if (1 == feature_num)
        {
            *y = dsp_linear_regression_uni_predict(coef, *x);
        }
        else
        {
            *y = dsp_linear_regression_mul_predict(feature_num, coef, x);
        }
    }
    else
    {
        dsp_linear_regression_mt_predict(feature_num, target_num, coef, x, y);
    }

    return 1;
}

/**
 * @brief 初始化一元线性回归模型
 *
 * @param w 用来保存累加权重
 * @param ex 增量求解过程中用来保存特征的期望
 * @param ey 增量求解过程中用来保存目标的期望
 * @param ex2 增量求解过程中用来保存 x^2 的期望
 * @param exy 增量求解过程中用来保存 x * y 的期望
 * @return 成功返回非0，失败返回0
 *
 * @note 是线性回归模型在输入和输出都是一元的情况下的简便函数
 */
extern int dsp_linear_regression1_init(
    float32_t *w,
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
 * @param xwx XWX
 * @param xwy XWy
 * @return 成功返回非0，失败返回0
 */
extern int dsp_linear_regression1_add_sample(
    float32_t x,
    float32_t y,
    float32_t weight,
    float32_t xwx[2][2],
    float32_t xwy[2]
);

/**
 * @brief 求解一元线性回归模型
 *
 * @param w 累加权重
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
    float32_t w,
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

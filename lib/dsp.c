/**
 * @brief 数字信号处理功能实现
 * @author 黄艺华 (lernanto@foxmail.com)
 */

#include <stdlib.h>

#include "arm_math.h"

#include "dsp.h"
#include "log.h"


int dsp_linear_regression_init(
    size_t feature_num,
    size_t target_num,
    float32_t *xwx,
    float32_t *xwy
)
{
    ++feature_num;
    arm_fill_f32(0.0f, xwx, feature_num * feature_num);
    arm_fill_f32(0.0f, xwy, feature_num * target_num);
    return 1;
}

static void update_xwx(
    size_t feature_num,
    const float32_t *x,
    float32_t weight,
    float32_t *xwx
)
{
    float32_t decay = 1.0f - weight;
    float32_t *pwx = xwx + feature_num * (feature_num + 1);
#ifdef __GNUC__
    float32_t wx[feature_num + 1];
#else
    float32_t *wx = (float32_t *)malloc(sizeof(float32_t) * (feature_num + 1));
#endif  /* __GNUC__ */

    /* TODO: 使用 DSP 指令优化向量运算速度 */
    for (size_t i = 0, base = 0; i < feature_num; ++i, base += feature_num + 1)
    {
        /* XWX 是对称阵，只计算其中下三角的一半 */
        for (size_t j = 0; j <= i; ++j)
        {
            xwx[base + j] = decay * xwx[base + j]
                + weight * x[i] * x[j];
        }
    }

    /* 计算 XWX 中的常数项部分，即特征 x 的加权平均 */
    arm_scale_f32(pwx, decay, pwx, feature_num + 1);
    arm_scale_f32(x, weight, wx, feature_num);
    wx[feature_num] = weight;
    arm_add_f32(pwx, wx, pwx, feature_num + 1);

#ifndef __GNUC__
    free(wx);
#endif  /* __GNUC__ */
}

static void update_xwy(
    size_t feature_num,
    const float32_t *x,
    float32_t y,
    float32_t weight,
    float32_t *xwy
)
{
#ifdef __GNUC__
    float32_t wyx[feature_num + 1];
#else
    float32_t *wyx = (float32_t *)malloc(sizeof(float32_t) * (feature_num + 1));
#endif  /* __GNUC__ */

    arm_scale_f32(xwy, 1.0f - weight, xwy, feature_num + 1);
    arm_scale_f32(x, weight * y, wyx, feature_num);
    wyx[feature_num] = weight * y;
    arm_add_f32(xwy, wyx, xwy, feature_num + 1);

#ifndef __GNUC__
    free(wyx);
#endif  /* __GNUC__ */
}

static void update_xwy_mt(
    size_t feature_num,
    size_t target_num,
    const float32_t *x,
    const float32_t *y,
    float32_t weight,
    float32_t *xwy
)
{
#ifdef __GNUC__
    float32_t wy[target_num];
#else
    float32_t *wy = (float32_t *)malloc(sizeof(float32_t) * target_num);
#endif  /* __GNUC__ */

    arm_scale_f32(xwy, 1.0f - weight, xwy, (feature_num + 1) * target_num);

    for (size_t i = 0, base = 0; i < feature_num; ++i, base += target_num)
    {
        for (size_t j = 0; j < target_num; ++j)
        {
            xwy[base + j] += weight * x[i] * y[j];
        }
    }

    arm_scale_f32(y, weight, wy, target_num);
    arm_add_f32(
        xwy + feature_num * target_num,
        wy,
        xwy + feature_num * target_num,
        target_num
    );

#ifndef __GNUC__
    free(wy);
#endif  /* __GNUC__ */
}

static arm_status inv(
    size_t dim,
    const float32_t *xwx,
    float32_t l2,
    arm_matrix_instance_f32 *result
)
{
    arm_status status;
    arm_matrix_instance_f32 a_mat;
#ifdef __GNUC__
    float32_t a[dim * dim];
#else
    float32_t *a = (float32_t *)malloc(sizeof(float32_t) * dim * dim);
#endif  /* __GNUC__ */

    arm_mat_init_f32(&a_mat, dim, dim, a);
    arm_copy_f32(xwx, a, dim * dim);

    /* 对称阵 XWX 只计算了下三角的一半，先复制补全对称的另一半 */
    for (size_t i = 0; i < dim; ++i)
    {
        for (size_t j = 0; j < i; ++j)
        {
            a[j * dim + i] = a[i * dim + j];
        }
    }

    /* 计算 XWX + l2 * I */
    if (l2 > 0.0f)
    {
        for (size_t i = 0; i < dim * dim; i += dim + 1)
        {
            a[i] += l2;
        }
    }

    /* 求 XWX + l2 * I 的逆 */
    status = arm_mat_inverse_f32(&a_mat, result);
#ifndef __GNUC__
    free(a);
#endif  /* __GNUC__ */
    return status;
}

void dsp_linear_regression_uni_update(
    float32_t x,
    float32_t y,
    float32_t weight,
    float32_t xwx[4],
    float32_t xwy[2]
)
{
    arm_scale_f32(xwx, 1.0f - weight, xwx, 4);
    xwx[0] += weight * x * x;
    xwx[2] += weight * x;
    xwx[3] += weight;

    arm_scale_f32(xwy, 1.0f - weight, xwy, 2);
    xwy[0] += weight * x * y;
    xwy[1] += weight * y;
}

int dsp_linear_regression_uni_solve(
    const float32_t xwx[4],
    const float32_t xwy[2],
    float32_t l2,
    float32_t coef[2]
)
{
    /* 直接计算二元一次非齐次方程组 */
    float32_t det = (xwx[0] + l2) * (xwx[3] + l2) - xwx[2] * xwx[2];

    if (det != 0.0f)
    {
        coef[0] = (xwy[0] * (xwx[3] + l2) - xwx[2] * xwy[1]) / det;
        coef[1] = xwy[1] - coef[0] * xwx[2];
        return 1;
    }
    else
    {
        /* 行列式为0，参数矩阵不满秩，方程组的解不唯一 */
        return 0;
    }
}

float32_t dsp_linear_regression_uni_predict(
    const float32_t *coef,
    float32_t x
)
{
    return coef[0] * x + coef[1];
}

void dsp_linear_regression_mul_update(
    size_t feature_num,
    const float32_t *x,
    float32_t y,
    float32_t weight,
    float32_t *xwx,
    float32_t *xwy
)
{
    update_xwx(feature_num, x, weight, xwx);
    update_xwy(feature_num, x, y, weight, xwy);
}

/**
 * 解线性方程组 (XWX + l2 * I) * coef = XWY
 */
int dsp_linear_regression_mul_solve(
    size_t feature_num,
    const float32_t *xwx,
    const float32_t *xwy,
    float32_t l2,
    float32_t *coef
)
{
    arm_status status;
    size_t dim = feature_num + 1;
#ifdef __GNUC__
    float32_t a_inv[dim * dim];
#else
    float32_t *a_inv = (float32_t *)malloc(sizeof(float32_t) * dim * dim);
#endif  /* __GNUC__ */
    arm_matrix_instance_f32 a_inv_mat;

    arm_mat_init_f32(&a_inv_mat, dim, dim, a_inv);
    status = inv(dim, xwx, l2, &a_inv_mat);
    if (ARM_MATH_SUCCESS == status)
    {
        /* (XWX + l2 * I) ^ -1 * XWY 为方程的解，当 XWY 为向量时使用矩阵乘向量方式 */
        arm_mat_vec_mult_f32(&a_inv_mat, xwy, coef);
    }

#ifndef __GNUC__
    free(a_inv);
#endif  /* __GNUC__ */
    return ARM_MATH_SUCCESS == status;
}

float32_t dsp_linear_regression_mul_predict(
    size_t feature_num,
    const float32_t *coef,
    const float32_t *x
)
{
    float32_t y;

    arm_dot_prod_f32(coef, x, feature_num, &y);
    y += coef[feature_num];
    return y;
}

void dsp_linear_regression_mt_update(
    size_t feature_num,
    size_t target_num,
    const float32_t *x,
    const float32_t *y,
    float32_t weight,
    float32_t *xwx,
    float32_t *xwy
)
{
    update_xwx(feature_num, x, weight, xwx);
    update_xwy_mt(feature_num, target_num, x, y, weight, xwy);
}

int dsp_linear_regression_mt_solve(
    size_t feature_num,
    size_t target_num,
    const float32_t *xwx,
    const float32_t *xwy,
    float32_t l2,
    float32_t *coef
)
{
    arm_status status;
    size_t dim = feature_num + 1;
#ifdef __GNUC__
    float32_t a_inv[dim * dim];
#else
    float32_t *a_inv = (float32_t *)malloc(sizeof(float32_t) * dim * dim);
#endif  /* __GNUC__ */
    arm_matrix_instance_f32 a_inv_mat;

    arm_mat_init_f32(&a_inv_mat, dim, dim, a_inv);
    status = inv(dim, xwx, l2, &a_inv_mat);
    if (ARM_MATH_SUCCESS == status)
    {
        /* (XWX + l2 * I) ^ -1 * XWY 即为方程的解 */
        arm_matrix_instance_f32 xwy_mat;
        arm_matrix_instance_f32 coef_mat;

        arm_mat_init_f32(&xwy_mat, dim, target_num, (float32_t *)xwy);
        arm_mat_init_f32(&coef_mat, dim, target_num, coef);
        arm_mat_mult_f32(&a_inv_mat, &xwy_mat, &coef_mat);
    }

#ifndef __GNUC__
    free(a_inv);
#endif  /* __GNUC__ */
    return ARM_MATH_SUCCESS == status;
}

void dsp_linear_regression_mt_predict(
    size_t feature_num,
    size_t target_num,
    const float32_t *coef,
    const float32_t *x,
    float32_t *y
)
{
    arm_matrix_instance_f32 coef_mat;
    arm_matrix_instance_f32 x_mat;
    arm_matrix_instance_f32 y_mat;

    arm_mat_init_f32(&coef_mat, feature_num, target_num, (float32_t *)coef);
    arm_mat_init_f32(&x_mat, 1, feature_num, (float32_t *)x);
    arm_mat_init_f32(&y_mat, 1, target_num, y);
    arm_mat_mult_f32(&x_mat, &coef_mat, &y_mat);
    arm_add_f32(y, coef + feature_num * target_num, y, target_num);
}

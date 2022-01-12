/**
 * @brief 数字信号处理功能实现
 * @author 黄艺华 (lernanto@foxmail.com)
 */

#include <stdlib.h>

#include "arm_math.h"

#include "dsp.h"
#include "log.h"


int dsp_linear_regression_init_diag(
    size_t feature_num,
    size_t target_num,
    float32_t *precision,
    float32_t *pre_mean,
    const float32_t *mean,
    const float32_t *diag
)
{
    ++feature_num;

    /* 初始化精度矩阵 */
    arm_fill_f32(0.0f, precision, feature_num * feature_num);
    /* 设置对角线元素 */
    for (size_t i = 0; i < feature_num; ++i)
    {
        precision[i * feature_num + i] = diag[i];
    }

    if (mean != NULL)
    {
        /* P * M */
        arm_matrix_instance_f32 pre_mat;
        arm_matrix_instance_f32 mean_mat;
        arm_matrix_instance_f32 pre_mean_mat;

        arm_mat_init_f32(&pre_mat, feature_num, feature_num, precision);
        arm_mat_init_f32(&mean_mat, feature_num, target_num, (float32_t *)mean);
        arm_mat_init_f32(&pre_mean_mat, feature_num, target_num, pre_mean);
        arm_mat_mult_f32(&pre_mat, &mean_mat, &pre_mean_mat);
    }
    else
    {
        /* 初始化先验期望为0 */
        arm_fill_f32(0.0f, pre_mean, feature_num * target_num);
    }

    return 1;
}

int dsp_linear_regression_init_scale(
    size_t feature_num,
    size_t target_num,
    float32_t *precision,
    float32_t *pre_mean,
    const float32_t *mean,
    float32_t scale
)
{
    ++feature_num;

    /* 初始化精度矩阵 */
    arm_fill_f32(0.0f, precision, feature_num * feature_num);
    /* 初始化对角线元素为固定值 */
    for (size_t i = 0; i < feature_num; ++i)
    {
        precision[i * feature_num + i] = scale;
    }

    if (mean != NULL)
    {
        /* P * M */
        arm_matrix_instance_f32 pre_mat;
        arm_matrix_instance_f32 mean_mat;
        arm_matrix_instance_f32 pre_mean_mat;

        arm_mat_init_f32(&pre_mat, feature_num, feature_num, precision);
        arm_mat_init_f32(&mean_mat, feature_num, target_num, (float32_t *)mean);
        arm_mat_init_f32(&pre_mean_mat, feature_num, target_num, pre_mean);
        arm_mat_mult_f32(&pre_mat, &mean_mat, &pre_mean_mat);
    }
    else
    {
        /* 初始化先验期望为0 */
        arm_fill_f32(0.0f, pre_mean, feature_num * target_num);
    }

    return 1;
}

static void update_precision(
    size_t feature_num,
    const float32_t *x,
    float32_t decay,
    float32_t weight,
    float32_t *precision
)
{
    float32_t *pwx = precision + feature_num * (feature_num + 1);

    /* TODO: 使用 DSP 指令优化向量运算速度 */
    for (size_t i = 0, base = 0; i < feature_num; ++i, base += feature_num + 1)
    {
        /* XWX 是对称阵，只计算其中下三角的一半 */
        for (size_t j = 0; j <= i; ++j)
        {
            precision[base + j] = decay * precision[base + j]
                + weight * x[i] * x[j];
        }
    }

    /* 计算 XWX 中的常数项部分，即特征 x 的加权平均 */
    arm_scale_f32(pwx, decay, pwx, feature_num + 1);
    if (1.0f == weight)
    {
        arm_add_f32(pwx, x, pwx, feature_num);
        pwx[feature_num] += 1.0f;
    }
    else
    {
#ifdef __GNUC__
        float32_t wx[feature_num + 1];
#else
        float32_t *wx = (float32_t *)malloc(sizeof(float32_t) * (feature_num + 1));
#endif  /* __GNUC__ */

        arm_scale_f32(x, weight, wx, feature_num);
        wx[feature_num] = weight;
        arm_add_f32(pwx, wx, pwx, feature_num + 1);

#ifndef __GNUC__
        free(wx);
#endif  /* __GNUC__ */
    }
}

static void update_pre_mean(
    size_t feature_num,
    const float32_t *x,
    float32_t y,
    float32_t decay,
    float32_t weight,
    float32_t *pre_mean
)
{
#ifdef __GNUC__
    float32_t wyx[feature_num + 1];
#else
    float32_t *wyx = (float32_t *)malloc(sizeof(float32_t) * (feature_num + 1));
#endif  /* __GNUC__ */

    arm_scale_f32(pre_mean, decay, pre_mean, feature_num + 1);
    arm_scale_f32(x, weight * y, wyx, feature_num);
    wyx[feature_num] = weight * y;
    arm_add_f32(pre_mean, wyx, pre_mean, feature_num + 1);

#ifndef __GNUC__
    free(wyx);
#endif  /* __GNUC__ */
}

static void update_pre_mean_mt(
    size_t feature_num,
    size_t target_num,
    const float32_t *x,
    const float32_t *y,
    float32_t decay,
    float32_t weight,
    float32_t *pre_mean
)
{
#ifdef __GNUC__
    float32_t wy[target_num];
#else
    float32_t *wy = (float32_t *)malloc(sizeof(float32_t) * target_num);
#endif  /* __GNUC__ */

    arm_scale_f32(pre_mean, decay, pre_mean, (feature_num + 1) * target_num);

    for (size_t i = 0, base = 0; i < feature_num; ++i, base += target_num)
    {
        for (size_t j = 0; j < target_num; ++j)
        {
            pre_mean[base + j] += weight * x[i] * y[j];
        }
    }

    arm_scale_f32(y, weight, wy, target_num);
    arm_add_f32(
        pre_mean + feature_num * target_num,
        wy,
        pre_mean + feature_num * target_num,
        target_num
    );

#ifndef __GNUC__
    free(wy);
#endif  /* __GNUC__ */
}

static arm_status compute_cov(
    size_t dim,
    float32_t *precision,
    arm_matrix_instance_f32 *cov
)
{
    arm_status status;
    arm_matrix_instance_f32 a_mat;
#ifdef __GNUC__
    float32_t a[dim * dim];
#else
    float32_t *a = (float32_t *)malloc(sizeof(float32_t) * dim * dim);
#endif  /* __GNUC__ */

    /* 精度矩阵只计算了下三角的一半，先复制补全对称的另一半 */
    for (size_t i = 0; i < dim; ++i)
    {
        for (size_t j = 0; j < i; ++j)
        {
            precision[j * dim + i] = precision[i * dim + j];
        }
    }

    /* 求协方差矩阵，为精度矩阵的逆 */
    arm_mat_init_f32(&a_mat, dim, dim, a);
    arm_copy_f32(precision, a, dim * dim);
    status = arm_mat_inverse_f32(&a_mat, cov);
#ifndef __GNUC__
    free(a);
#endif  /* __GNUC__ */
    return status;
}

void dsp_linear_regression_uni_update(
    float32_t x,
    float32_t y,
    float32_t decay,
    float32_t weight,
    float32_t precision[4],
    float32_t pre_mean[2]
)
{
    arm_scale_f32(precision, decay, precision, 4);
    precision[0] += weight * x * x;
    precision[2] += weight * x;
    precision[3] += weight;

    arm_scale_f32(pre_mean, decay, pre_mean, 2);
    pre_mean[0] += weight * x * y;
    pre_mean[1] += weight * y;
}

int dsp_linear_regression_uni_solve(
    float32_t precision[4],
    const float32_t pre_mean[2],
    float32_t mean[2],
    float32_t cov[4]
)
{
    precision[1] = precision[2];

    /* 直接计算二元一次非齐次方程组 */
    float32_t det = precision[0] * precision[3] - precision[1] * precision[2];

    if (det != 0.0f)
    {
        cov[0] = precision[3] / det;
        cov[1] = cov[2] = -precision[1] / det;
        cov[3] = precision[0] / det;

        mean[0] = cov[0] * pre_mean[0] + cov[1] * pre_mean[1];
        mean[1] = cov[2] * pre_mean[0] + cov[3] * pre_mean[1];
        return 1;
    }
    else
    {
        /* 行列式为0，参数矩阵不满秩，方程组的解不唯一 */
        return 0;
    }
}

float32_t dsp_linear_regression_uni_predict(
    const float32_t mean[2],
    float32_t x
)
{
    return mean[0] * x + mean[1];
}

float32_t dsp_linear_regression_uni_predict_prec(
    const float32_t precision[4],
    float32_t x
)
{
    float32_t x2 = x * x + 1.0f;
    return (precision[0] * x * x + precision[1] * x + precision[2] * x + precision[3])
        / x2 / x2;
}

void dsp_linear_regression_mul_update(
    size_t feature_num,
    const float32_t *x,
    float32_t y,
    float32_t decay,
    float32_t weight,
    float32_t *precision,
    float32_t *pre_mean
)
{
    update_precision(feature_num, x, decay, weight, precision);
    update_pre_mean(feature_num, x, y, decay, weight, pre_mean);
}

/**
 * 解线性方程组 P * mean = P0 * M0 + X * W * Y
 */
int dsp_linear_regression_mul_solve(
    size_t feature_num,
    float32_t *precision,
    const float32_t *pre_mean,
    float32_t *mean,
    float32_t *cov
)
{
    arm_status status;
    size_t dim = feature_num + 1;
    arm_matrix_instance_f32 cov_mat;

    /* 求协方差矩阵，为精度矩阵的逆 */
    arm_mat_init_f32(&cov_mat, dim, dim, cov);
    status = compute_cov(dim, precision, &cov_mat);
    if (ARM_MATH_SUCCESS == status)
    {
        /* Cov * (P0 * M0 + X * W * Y) 为方程的解，
           当 P0 * M0 + X * W * Y 为向量时使用矩阵乘向量方式 */
        arm_mat_vec_mult_f32(&cov_mat, pre_mean, mean);
    }

    return ARM_MATH_SUCCESS == status;
}

float32_t dsp_linear_regression_mul_predict(
    size_t feature_num,
    const float32_t *mean,
    const float32_t *x
)
{
    float32_t y;

    arm_dot_prod_f32(mean, x, feature_num, &y);
    y += mean[feature_num];
    return y;
}

float32_t dsp_linear_regression_mul_predict_prec(
    size_t feature_num,
    const float32_t *precision,
    const float32_t *x
)
{
    size_t dim = feature_num + 1;
    arm_matrix_instance_f32 prec_mat;
    float32_t x2;
    float32_t prec;
#ifdef __GNUC__
    float32_t x_ext[dim * dim];
    float32_t prec_x[dim * dim];
#else
    float32_t *x_ext = (float32_t *)malloc(sizeof(float32_t) * dim);
    float32_t *prec_x = (float32_t *)malloc(sizeof(float32_t) * dim);
#endif  /* __GNUC__ */

    arm_copy_f32(x, x_ext, feature_num);
    x_ext[feature_num] = 1.0f;

    arm_dot_prod_f32(x_ext, x_ext, dim, &x2);
    arm_scale_f32(x_ext, 1.0f / x2, x_ext, dim);

    arm_mat_init_f32(&prec_mat, dim, dim, (float32_t *)precision);
    arm_mat_vec_mult_f32(&prec_mat, x_ext, prec_x);
    arm_dot_prod_f32(x_ext, prec_x, dim, &prec);

#ifndef __GNUC__
    free(x_ext);
    free(prec_x);
#endif  /* __GNUC__ */
    return prec;
}

void dsp_linear_regression_mt_update(
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
    update_precision(feature_num, x, decay, weight, precision);
    update_pre_mean_mt(feature_num, target_num, x, y, decay, weight, pre_mean);
}

int dsp_linear_regression_mt_solve(
    size_t feature_num,
    size_t target_num,
    float32_t *precision,
    const float32_t *pre_mean,
    float32_t *mean,
    float32_t *cov
)
{
    arm_status status;
    size_t dim = feature_num + 1;
    arm_matrix_instance_f32 cov_mat;

    arm_mat_init_f32(&cov_mat, dim, dim, cov);
    status = compute_cov(dim, precision, &cov_mat);
    if (ARM_MATH_SUCCESS == status)
    {
        /* Cov * (P0 * M0 + X * W * Y) 即为方程的解 */
        arm_matrix_instance_f32 pre_mean_mat;
        arm_matrix_instance_f32 mean_mat;

        arm_mat_init_f32(&pre_mean_mat, dim, target_num, (float32_t *)pre_mean);
        arm_mat_init_f32(&mean_mat, dim, target_num, mean);
        arm_mat_mult_f32(&cov_mat, &pre_mean_mat, &mean_mat);
    }

    return ARM_MATH_SUCCESS == status;
}

void dsp_linear_regression_mt_predict(
    size_t feature_num,
    size_t target_num,
    const float32_t *mean,
    const float32_t *x,
    float32_t *y
)
{
    arm_matrix_instance_f32 mean_mat;
    arm_matrix_instance_f32 x_mat;
    arm_matrix_instance_f32 y_mat;

    arm_mat_init_f32(&mean_mat, feature_num, target_num, (float32_t *)mean);
    arm_mat_init_f32(&x_mat, 1, feature_num, (float32_t *)x);
    arm_mat_init_f32(&y_mat, 1, target_num, y);
    arm_mat_mult_f32(&x_mat, &mean_mat, &y_mat);
    arm_add_f32(y, mean + feature_num * target_num, y, target_num);
}

/**
 * @brief 数字信号处理功能实现
 * @author 黄艺华 (lernanto@foxmail.com)
 */

#include <string.h>

#include "arm_math.h"

#include "dsp.h"
#include "log.h"


int dsp_linear_regression_init(
    size_t feature_dim,
    size_t target_dim,
    float32_t *xwx,
    float32_t *ywx
)
{
    arm_fill_f32(0.0f, xwx, feature_dim * feature_dim);
    arm_fill_f32(0.0f, ywx, target_dim * feature_dim);
    return 1;
}

int dsp_linear_regression_add_sample(
    size_t feature_dim,
    size_t target_dim,
    const float32_t *x,
    const float32_t *y,
    float32_t weight,
    float32_t *xwx,
    float32_t *ywx
)
{
    float32_t var = 1.0f / weight - 1.0f;

    /* TODO: 使用 DSP 指令优化向量运算速度 */
    for (size_t i = 0, base = 0; i < feature_dim; ++i, base += feature_dim)
    {
        /* XWX 是对称阵，只计算其中一半 */
        for (size_t j = i; j < feature_dim; ++j)
        {
            xwx[base + j] = (1 - weight) * xwx[base + j]
                + weight * x[i] * x[j];
        }
    }

    if (1 == target_dim)
    {
        /* 当 y 为一维时，使用向量运算加快速度，注意 1 - w = w * y * ((1 / w - 1) / y) */
        arm_scale_f32(ywx, var / *y, ywx, feature_dim);
        arm_add_f32(ywx, x, ywx, feature_dim);
        arm_scale_f32(ywx, weight * *y, ywx, feature_dim);
    }
    else
    {
        /* 注意 1 - w = w * (1 / w - 1) */
        arm_scale_f32(ywx, var, ywx, target_dim * feature_dim);

        for (size_t j = 0, base = 0; j < target_dim; ++j, base += feature_dim)
        {
            for (size_t i = 0; i < feature_dim; ++i)
            {
                ywx[base + i] += x[i] * y[j];
            }
        }

        arm_scale_f32(ywx, weight, ywx, target_dim * feature_dim);
    }

    return 1;
}

int dsp_linear_regression_solve(
    size_t feature_dim,
    size_t target_dim,
    float32_t *xwx,
    const float32_t *ywx,
    float32_t l2,
    float32_t *param_buf,
    float32_t *param_inv,
    float32_t *coef
)
{
    arm_matrix_instance_f32 a;
    arm_matrix_instance_f32 a_inv;
    arm_status status = ARM_MATH_SUCCESS;

    /* 解线性方程组 (XWX + l2 * I) * coef = yWX */
    /* 对称阵 XWX 只计算了一半，先复制补全对称的另一半 */
    for (size_t i = 0, base = 0; i < feature_dim; ++i, base += feature_dim)
    {
        for (size_t j = base, k = i; j < base + i; ++j, k += feature_dim)
        {
            xwx[j] = xwx[k];
        }
    }

    /* 计算 XWX + l2 * I */
    if (l2 > 0.0f)
    {
        arm_copy_f32(xwx, param_buf, feature_dim * feature_dim);
        for (size_t i = 0; i < feature_dim * feature_dim; i += feature_dim + 1)
        {
            param_buf[i] += l2;
        }
        arm_mat_init_f32(&a, feature_dim, feature_dim, param_buf);
    }
    else
    {
        arm_mat_init_f32(&a, feature_dim, feature_dim, (float32_t *)xwx);
    }

    /* 求 XWX + l2 * I 的逆 */
    arm_mat_init_f32(&a_inv, feature_dim, feature_dim, param_inv);
    if ((status = arm_mat_inverse_f32(&a, &a_inv)) != ARM_MATH_SUCCESS)
    {
        log_error("cannot inverse XWX");
        return 0;
    }

    /* yWX * (XWX + l2 * I) ^ -1 即为方程的解 */
    if (1 == target_dim)
    {
        /* 当 yWX 为向量时使用矩阵乘向量方式 */
        arm_mat_vec_mult_f32(&a_inv, ywx, coef);
    }
    else
    {
        /* 当 yWX 为矩阵时使用矩阵乘 */
        arm_matrix_instance_f32 b;
        arm_matrix_instance_f32 x;

        arm_mat_init_f32(&b, target_dim, feature_dim, (float32_t *)ywx);
        arm_mat_init_f32(&x, target_dim, feature_dim, coef);
        arm_mat_mult_f32(&b, &a_inv, &x);
    }

#if 0
    /* 对 XWX + l2 * I 进行 Cholesky 分解 */
    if ((status = arm_mat_cholesky_f32(&a, &cholesky)) != ARM_MATH_SUCCESS)
    {
        log_error("cannot perform Cholesky decomposition on XWX");
        return 0;
    }

    /* Cholesky 分解把系数矩阵分解为上三角阵及其转置的乘积，
       求解原方程只需要求解两次系数为三角阵的方程即可 */
    if ((status = arm_mat_solve_upper_triangular_f32(&cholesky, &b, &x)) != ARM_MATH_SUCCESS)
    {
        log_error("cannot solve upper triangular formula");
        return 0;
    }

    if ((status = arm_mat_solve_lower_triangular_f32(cholesky, x, coef)) != ARM_MATH_SUCCESS)
    {
        log_error("cannot solve lower triangular formula");
        return 0;
    }
#endif

    return 1;
}

int dsp_linear_regression_predict(
    size_t feature_dim,
    size_t target_dim,
    const float32_t *coef,
    const float32_t *x,
    float32_t *y
)
{
    if (1 == target_dim)
    {
        arm_dot_prod_f32(coef, x, feature_dim, y);
    }
    else
    {
        arm_matrix_instance_f32 coef_mat;

        arm_mat_init_f32(&coef_mat, target_dim, feature_dim, (float32_t *)coef);
        arm_mat_vec_mult_f32(&coef_mat, x, y);
    }

    return 1;
}

int dsp_linear_regression1_init(
    float32_t *ex,
    float32_t *ey,
    float32_t *ex2,
    float32_t *exy
)
{
    *ex = 0.0f;
    *ey = 0.0f;
    *ex2 = 0.0f;
    *exy = 0.0f;

    return 1;
}

int dsp_linear_regression1_add_sample(
    float32_t x,
    float32_t y,
    float32_t weight,
    float32_t *ex,
    float32_t *ey,
    float32_t *ex2,
    float32_t *exy
)
{
    float32_t decay = 1.0f - weight;

    *ex = decay * *ex + weight * x;
    *ey = decay * *ey + weight * y;
    *ex2 = decay * *ex2 + weight * x * x;
    *exy = decay * *exy + weight * x * y;

    return 1;
}

int dsp_linear_regression1_solve(
    float32_t ex,
    float32_t ey,
    float32_t ex2,
    float32_t exy,
    float32_t l2,
    float32_t *coef,
    float32_t *bias
)
{
    float32_t det = (ex2 + l2) * (1.0f + l2) - ex * ex;

    if ((det < -1e-6f) || (det > 1e-6f))
    {
        *coef = (exy * (1.0f + l2) - ex * ey) / det;
        *bias = ey - *coef * ex;
    }
    else
    {
        log_debug("matrix is singular, det = %f", det);
        *coef = 0.0f;
        *bias = ey;
    }

    return 1;
}

int dsp_linear_regression1_predict(
    float32_t coef,
    float32_t bias,
    float32_t x,
    float32_t *y
)
{
    *y = coef * x + bias;
    return 1;
}

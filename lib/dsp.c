/**
 * @brief 
 * @author 黄艺华 (lernanto@foxmail.com)
 */

#include <string.h>

#include "arm_math.h"

#include "dsp.h"
#include "log.h"


int dsp_lwlr_init(
    size_t feature_num,
    size_t label_num,
    float32_t *var,
    float32_t *xwx,
    float32_t *ywx
)
{
    *var = 0.0f;
    arm_fill_f32(0.0f, xwx, feature_num * feature_num);
    arm_fill_f32(0.0f, ywx, label_num * feature_num);
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
    float32_t *ywx
)
{
    float32_t w;

    *var = *var * decay + 1;
    w = 1.0f / *var;

    /* TODO: 使用 DSP 指令优化向量运算速度 */
    for (size_t i = 0, base = 0; i < feature_num; ++i, base += feature_num)
    {
        /* XWX 是对称阵，只计算其中一半 */
        for (size_t j = i; j < feature_num; ++j)
        {
            xwx[base + j] = (1 - w) * xwx[base + j]
                + w * x[i] * x[j];
        }
    }

    if (1 == label_num)
    {
        /* 当 y 为一维时，使用向量运算加快速度，注意 1 - w = w * y * ((var - 1) / y) */
        arm_scale_f32(ywx, (*var - 1.0f) / *y, ywx, feature_num);
        arm_add_f32(ywx, x, ywx, feature_num);
        arm_scale_f32(ywx, w * *y, ywx, feature_num);
    }
    else
    {
        /* 注意 1 - w = w * (var - 1) */
        arm_scale_f32(ywx, (*var - 1.0f), ywx, label_num * feature_num);

        for (size_t j = 0, base = 0; j < label_num; ++j, base += feature_num)
        {
            for (size_t i = 0; i < feature_num; ++i)
            {
                ywx[base + i] += x[i] * y[j];
            }
        }

        arm_scale_f32(ywx, w, ywx, label_num * feature_num);
    }

    return 1;
}

int dsp_lwlr_solve(
    size_t feature_num,
    size_t label_num,
    float32_t *xwx,
    const float32_t *ywx,
    float32_t l2,
    float32_t *coef
)
{
    static float32_t a_data[100];
    static float32_t a_inv_data[100];

    arm_matrix_instance_f32 a;
    arm_matrix_instance_f32 a_inv;
    arm_status status = ARM_MATH_SUCCESS;

    /* 解线性方程组 (XWX + l2 * I) * coef = yWX */
    /* 对称阵 XWX 只计算了一半，先复制补全对称的另一半 */
    for (size_t i = 0, base = 0; i < feature_num; ++i, base += feature_num)
    {
        for (size_t j = base, k = i; j < base + i; ++j, k += feature_num)
        {
            xwx[j] = xwx[k];
        }
    }

    /* 计算 XWX + l2 * I */
    if (l2 > 0.0f)
    {
        arm_copy_f32(xwx, a_data, feature_num * feature_num);
        for (size_t i = 0; i < feature_num * feature_num; i += feature_num + 1)
        {
            a_data[i] += l2;
        }
        arm_mat_init_f32(&a, feature_num, feature_num, a_data);
    }
    else
    {
        arm_mat_init_f32(&a, feature_num, feature_num, (float32_t *)xwx);
    }

    /* 求 XWX + l2 * I 的逆 */
    arm_mat_init_f32(&a_inv, feature_num, feature_num, a_inv_data);
    if ((status = arm_mat_inverse_f32(&a, &a_inv)) != ARM_MATH_SUCCESS)
    {
        log_error("cannot inverse XWX");
        return 0;
    }

    /* yWX * (XWX + l2 * I) ^ -1 即为方程的解 */
    if (1 == label_num)
    {
        /* 当 yWX 为向量时使用矩阵乘向量方式 */
        arm_mat_vec_mult_f32(&a_inv, ywx, coef);
    }
    else
    {
        /* 当 yWX 为矩阵时使用矩阵乘 */
        arm_matrix_instance_f32 b;
        arm_matrix_instance_f32 x;

        arm_mat_init_f32(&b, label_num, feature_num, (float32_t *)ywx);
        arm_mat_init_f32(&x, label_num, feature_num, coef);
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

int dsp_lwlr_predict(
    size_t feature_num,
    size_t label_num,
    const float32_t *coef,
    const float32_t *x,
    float32_t *y
)
{
    if (1 == label_num)
    {
        arm_dot_prod_f32(coef, x, feature_num, y);
    }
    else
    {
        arm_matrix_instance_f32 coef_mat;

        arm_mat_init_f32(&coef_mat, label_num, feature_num, (float32_t *)coef);
        arm_mat_vec_mult_f32(&coef_mat, x, y);
    }

    return 1;
}

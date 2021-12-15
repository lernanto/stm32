/**
 * @brief 
 * @author 黄艺华 (lernanto@foxmail.com)
 */

#include <string.h>

#include "arm_math.h"
#include "dsp/matrix_functions.h"

#include "dsp.h"
#include "log.h"


int dsp_lwlr_init(
    size_t feature_num,
    size_t label_num,
    float32_t *var,
    float32_t *xwx,
    float32_t *xwy
)
{
    *var = 1.0f;
    arm_fill_f32(0.0f, xwx, feature_num * feature_num);
    arm_fill_f32(0.0f, xwy, feature_num * label_num);
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
    float32_t w;

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

int dsp_lwlr_solve(
    size_t feature_num,
    size_t label_num,
    const float32_t *xwx,
    const float32_t *xwy,
    float32_t l2,
    float32_t *coef
)
{
    static float32_t a_data[100];
    static float32_t a_inv_data[100];

    arm_matrix_instance_f32 a;
    arm_matrix_instance_f32 a_inv;
    arm_matrix_instance_f32 b;
    arm_matrix_instance_f32 x;
    arm_status status = ARM_MATH_SUCCESS;

    /* 解线性方程组 (XWX + l2 * I) * coef = XWy */
    /* 计算 XWX + l2 * I */
    if (l2 > 0.0f)
    {
        arm_copy_f32(xwx, a_data, feature_num * feature_num);
        for (size_t i = 0; i < feature_num; ++i)
        {
            a_data[i * feature_num + i] += l2;
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

    /* (XWX + l2 * I) ^ -1 * XWy 即为方程的解 */
    arm_mat_init_f32(&b, feature_num, label_num, (float32_t *)xwy);
    arm_mat_init_f32(&x, feature_num, label_num, coef);
    arm_mat_mult_f32(&a, &b, &x);

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
    arm_matrix_instance_f32 coef_mat;

    arm_mat_init_f32(&coef_mat, feature_num, label_num, (float32_t *)coef);
    arm_mat_vec_mult_f32(&coef_mat, x, y);
    return 1;
}

/**
 * @author 黄艺华 (lernanto@foxmail.com)
 * @brief 测试增量线性回归
 */

#include <stdlib.h>
#include <float.h>
#include <math.h>
#include <time.h>

#include "arm_math.h"

#include "log.h"
#include "dsp.h"
#include "data.h"


static float32_t marsaglia(void)
{
    float32_t v1;
    float32_t v2;
    float32_t s;

    do
    {
        v1 = (float32_t)rand() / (float32_t)RAND_MAX * 2.0f - 1.0f;
        v2 = (float32_t)rand() / (float32_t)RAND_MAX * 2.0f - 1.0f;
        s = v1 * v1 + v2 * v2;
        log_verbose("v1 = %f, v2 = %f, s = %f", v1, v2, s);
    } while ((s >= 1.0f) || (s <= FLT_MIN * 1000));

    arm_sqrt_f32(log(s) * -2.0f / s, &s);
    return v1 * s;
}

static inline float32_t gaussian(float32_t mean, float32_t std)
{
    return marsaglia() * std + mean;
}

static inline float32_t square_error(float32_t y_true, float32_t y_pred)
{
    float32_t err = y_true - y_pred;
    return err * err * 0.5f;
}

int test_locally_weighted_linear_regression(void)
{
    static const float32_t weight = 0.1f;
    float32_t ex;
    float32_t ey;
    float32_t ex2;
    float32_t exy;
    float32_t mse;

    srand(time(NULL));
    dsp_linear_regression1_init(&ex, &ey, &ex2, &exy);
    mse = 0.0f;

    for (size_t i = 0; i < 1000; ++i)
    {
        float32_t x = (float32_t)i * 0.01f;
        float32_t y_true = arm_sin_f32(x) + x;
        float32_t y = y_true + gaussian(0.0f, 0.1f);
        float32_t coef;
        float32_t bias;
        float32_t y_pred;
        float32_t w = 1.0f / (float32_t)(i + 1);

        dsp_linear_regression1_add_sample(
            x,
            y,
            weight,
            &ex,
            &ey,
            &ex2,
            &exy
        );

        dsp_linear_regression1_solve(ex, ey, ex2, exy, 0.0f, &coef, &bias);
        dsp_linear_regression1_predict(coef, bias, x, &y_pred);
        mse = (1.0f - w) * mse + w * square_error(y_true, y_pred);

        log_debug(
            "x = %f, y_true = %f, y = %f, coef = %f, bias = %f, "
            "y_pred = %f, error = %f, MSE = %f",
            x, y_true, y, coef, bias, y_pred, square_error(y_true, y_pred), mse
        );
        log_verbose("Ex = %f, Ey = %f, Ex2 = %f, Exy = %f", ex, ey, ex2, exy);
    }

    log_info("MSE = %f", mse);
    return 1;
}

int test_linear_regression(void)
{
    static float32_t xwx[FEATURE_DIM][FEATURE_DIM];
    static float32_t ywx[TARGET_DIM][FEATURE_DIM];
    static float32_t xwx_inv[FEATURE_DIM][FEATURE_DIM];
    static float32_t coef[TARGET_DIM][FEATURE_DIM];
    static float32_t ywx_vec[FEATURE_DIM];
    static float32_t coef_vec[FEATURE_DIM];
    static char buf[256];

    float32_t pred_y[TARGET_DIM];
    float32_t ex;
    float32_t ey;
    float32_t ex2;
    float32_t exy;
    float32_t coef1;
    float32_t bias1;
    float32_t pred_y1;
    float32_t mse;

    /* 特征和目标均为向量，多目标线性回归 */
    log_info(
        "test linear regression, feature num = %u(with dummy), target num = %u",
        FEATURE_DIM,
        TARGET_DIM
    );

    dsp_linear_regression_init(
        FEATURE_DIM,
        TARGET_DIM,
        (float32_t *)xwx,
        (float32_t *)ywx
    );

    for (size_t i = 0; i < TRAIN_SAMPLE_NUM; ++i)
    {
        dsp_linear_regression_add_sample(
            FEATURE_DIM,
            TARGET_DIM,
            train_x[i],
            train_y[i],
            1.0f / (float32_t)(i + 1),
            (float32_t *)xwx,
            (float32_t *)ywx
        );
    }

    dsp_linear_regression_solve(
        FEATURE_DIM,
        TARGET_DIM,
        (float32_t *)xwx,
        (float32_t *)ywx,
        0.0f,
        NULL,
        (float32_t *)xwx_inv,
        (float32_t *)coef
    );

    log_debug("solve linear regression, coef =");
    for (size_t i = 0; i < TARGET_DIM; ++i)
    {
        for (size_t j = 0, n = 0; j < FEATURE_DIM; ++j)
        {
            n += snprintf(buf + n, sizeof(buf) - n, "%f, ", coef[i][j]);
        }
        log_debug(buf);
    }

    /* 在测试集上计算 MSE 损失 */
    mse = 0.0f;
    for (size_t i = 0; i < TEST_SAMPLE_NUM; ++i)
    {
        float32_t w = 1.0f / (float32_t)(i + 1);
        float32_t dist;

        dsp_linear_regression_predict(
            FEATURE_DIM,
            TARGET_DIM,
            (float32_t *)coef,
            test_x[i],
            pred_y
        );

        dist = arm_euclidean_distance_f32(
            (float32_t *)test_y[i],
            pred_y,
            TARGET_DIM
        );
        mse = (1.0f - w) * mse + w / (float32_t)TARGET_DIM * dist * dist;
    }

    log_info("done, test MSE = %f", mse);

    /* 特征为向量，目标为标量，普通线性回归 */
    log_info(
        "test linear regression, feature num = %u(with dummy), target num = 1",
        FEATURE_DIM,
        TARGET_DIM
    );

    dsp_linear_regression_init(
        FEATURE_DIM,
        1,
        (float32_t *)xwx,
        ywx_vec
    );

    for (size_t i = 0; i < TRAIN_SAMPLE_NUM; ++i)
    {
        dsp_linear_regression_add_sample(
            FEATURE_DIM,
            1,
            train_x[i],
            &train_y_vec[i],
            1.0f / (float32_t)(i + 1),
            (float32_t *)xwx,
            ywx_vec
        );
    }

    dsp_linear_regression_solve(
        FEATURE_DIM,
        1,
        (float32_t *)xwx,
        ywx_vec,
        0.0f,
        NULL,
        (float32_t *)xwx_inv,
        coef_vec
    );

    for (size_t i = 0, n = 0; i < FEATURE_DIM; ++i)
    {
        n += snprintf(buf + n, sizeof(buf) - n, "%f, ", coef_vec[i]);
    }
    log_debug("solve linear regression, coef = %s", buf);

    mse = 0.0f;
    for (size_t i = 0; i < TEST_SAMPLE_NUM; ++i)
    {
        float32_t w = 1.0f / (float32_t)(i + 1);

        dsp_linear_regression_predict(
            FEATURE_DIM,
            1,
            coef_vec,
            test_x[i],
            &pred_y1
        );

        mse = (1.0f - w) * mse + w * square_error(test_y_vec[i], pred_y1);
    }

    log_info("done, test MSE = %f", mse);

    /* 特征和目标均为标量，一元线性回归 */
    log_info("test linear regression, feature num = 1(without dummy), target num = 1");

    dsp_linear_regression1_init(&ex, &ey, &ex2, &exy);

    for (size_t i = 0; i < TRAIN_SAMPLE_NUM; ++i)
    {
        dsp_linear_regression1_add_sample(
            train_x1[i],
            train_y1[i],
            1.0f / (float32_t)(i + 1),
            &ex,
            &ey,
            &ex2,
            &exy
        );
    }

    dsp_linear_regression1_solve(ex, ey, ex2, exy, 0.0f, &coef1, &bias1);
    log_debug(
        "solve univariate linear regression, coef = %f, bias = %f",
        coef1,
        bias1
    );

    mse = 0.0f;
    for (size_t i = 0; i < TEST_SAMPLE_NUM; ++i)
    {
        float32_t w = 1.0f / (float32_t)(i + 1);

        dsp_linear_regression1_predict(coef1, bias1, test_x1[i], &pred_y1);
        mse = (1.0f - w) * mse + w * square_error(test_y_vec[i], pred_y1);
    }

    log_info("done, test MSE = %f", mse);
    return 1;
}

#ifdef __GNUC__
__attribute__((weak)) int main(int argc, char **argv)
#else
int main(int argc, char **argv)
#endif  /* __GNUC__ */
{
    int succ = 1;

    succ &= test_linear_regression();
    succ &= test_locally_weighted_linear_regression();
    return succ ? 0 : -1;
}

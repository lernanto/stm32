/**
 * @author 黄艺华 (lernanto@foxmail.com)
 * @brief 测试增量线性回归
 */

#include "log.h"
#include "dsp.h"
#include "data.h"


int test_linear_regression(void)
{
    static float32_t xwx[FEATURE_NUM][FEATURE_NUM];
    static float32_t ywx[TARGET_NUM][FEATURE_NUM];
    static float32_t xwx_inv[FEATURE_NUM][FEATURE_NUM];
    static float32_t coef[TARGET_NUM][FEATURE_NUM];
    static float32_t ywx_vec[FEATURE_NUM];
    static float32_t coef_vec[FEATURE_NUM];
    static char buf[256];

    float32_t pred_y[TARGET_NUM];
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
        FEATURE_NUM,
        TARGET_NUM
    );

    dsp_linear_regression_init(
        FEATURE_NUM,
        TARGET_NUM,
        (float32_t *)xwx,
        (float32_t *)ywx
    );

    for (size_t i = 0; i < TRAIN_SAMPLE_NUM; ++i)
    {
        dsp_linear_regression_add_sample(
            FEATURE_NUM,
            TARGET_NUM,
            train_x[i],
            train_y[i],
            1.0f / (float32_t)(i + 1),
            (float32_t *)xwx,
            (float32_t *)ywx
        );
    }

    dsp_linear_regression_solve(
        FEATURE_NUM,
        TARGET_NUM,
        (float32_t *)xwx,
        (float32_t *)ywx,
        0.0f,
        NULL,
        (float32_t *)xwx_inv,
        (float32_t *)coef
    );

    log_debug("solve linear regression, coef =");
    for (size_t i = 0; i < TARGET_NUM; ++i)
    {
        for (size_t j = 0, n = 0; j < FEATURE_NUM; ++j)
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
            FEATURE_NUM,
            TARGET_NUM,
            (float32_t *)coef,
            test_x[i],
            pred_y
        );

        dist = arm_euclidean_distance_f32(
            (float32_t *)test_y[i],
            pred_y,
            TARGET_NUM
        );
        mse = (1.0f - w) * mse + w * 0.5f / (float32_t)TARGET_NUM * dist * dist;
    }

    log_info("done, test MSE = %f", mse);

    /* 特征为向量，目标为标量，普通线性回归 */
    log_info(
        "test linear regression, feature num = %u(with dummy), target num = 1",
        FEATURE_NUM,
        TARGET_NUM
    );

    dsp_linear_regression_init(
        FEATURE_NUM,
        1,
        (float32_t *)xwx,
        ywx_vec
    );

    for (size_t i = 0; i < TRAIN_SAMPLE_NUM; ++i)
    {
        dsp_linear_regression_add_sample(
            FEATURE_NUM,
            1,
            train_x[i],
            &train_y_vec[i],
            1.0f / (float32_t)(i + 1),
            (float32_t *)xwx,
            ywx_vec
        );
    }

    dsp_linear_regression_solve(
        FEATURE_NUM,
        1,
        (float32_t *)xwx,
        ywx_vec,
        0.0f,
        NULL,
        (float32_t *)xwx_inv,
        coef_vec
    );

    for (size_t i = 0, n = 0; i < FEATURE_NUM; ++i)
    {
        n += snprintf(buf + n, sizeof(buf) - n, "%f, ", coef_vec[i]);
    }
    log_debug("solve linear regression, coef = %s", buf);

    mse = 0.0f;
    for (size_t i = 0; i < TEST_SAMPLE_NUM; ++i)
    {
        float32_t w = 1.0f / (float32_t)(i + 1);
        float32_t dist;

        dsp_linear_regression_predict(
            FEATURE_NUM,
            1,
            coef_vec,
            test_x[i],
            &pred_y1
        );

        dist = test_y_vec[i] - pred_y1;
        mse = (1.0f - w) * mse + w * 0.5f * dist * dist;
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
        float32_t dist;

        dsp_linear_regression1_predict(coef1, bias1, test_x1[i], &pred_y1);
        dist = test_y1[i] - pred_y1;
        mse = (1.0f - w) * mse + w * 0.5f * dist * dist;
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
    return test_linear_regression() ? 0 : -1;
}

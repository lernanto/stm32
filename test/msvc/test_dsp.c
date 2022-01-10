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

static int linear_regression(
    size_t train_sample_num,
    size_t test_sample_num,
    size_t feature_num,
    size_t target_num,
    const float32_t *train_x,
    const float32_t *train_y,
    const float32_t *test_x,
    const float32_t *test_y,
    float32_t scale
)
{
    size_t dim = feature_num + 1;
#ifdef __GNUC__
    float32_t pre[dim * dim];
    float32_t pre_mean[dim * target_num];
    float32_t mean[dim];
    float32_t cov[dim * dim];
    float32_t y_pred[target_num];
#else
    float32_t *pre = (float32_t *)malloc(sizeof(float32_t) * dim * dim);
    float32_t *pre_mean = (float32_t *)malloc(sizeof(float32_t) * dim * target_num);
    float32_t *mean = (float32_t *)malloc(sizeof(float32_t) * dim);
    float32_t *cov = (float32_t *)malloc(sizeof(float32_t) * dim * dim);
    float32_t *y_pred = (float32_t *)malloc(sizeof(float32_t) * target_num);
#endif  /* __GNUC__ */
    int ret;
    float32_t mse;

    log_info(
        "linear regression, feature num = %u, target num = %u",
        feature_num,
        target_num
    );

    dsp_linear_regression_init_scale(
        feature_num,
        target_num,
        pre,
        pre_mean,
        NULL,
        scale
    );

    log_verbose("prior precision matrix =");
    for (size_t i = 0; i < dim; ++i)
    {
        for (size_t j = 0; j < dim; ++j)
        {
            log_verbose("%f", pre[i * dim + j]);
        }
    }
    log_verbose("prior precision * mean =");
    for (size_t i = 0; i < dim; ++i)
    {
        for (size_t j = 0; j < target_num; ++j)
        {
            log_verbose("%f", pre_mean[i * target_num + j]);
        }
    }

    for (size_t i = 0; i < train_sample_num; ++i)
    {
        dsp_linear_regression_update(
            feature_num,
            target_num,
            train_x + i * feature_num,
            train_y + i * target_num,
            1.0f,
            1.0f,
            pre,
            pre_mean
        );

        log_verbose("x =");
        for (size_t j = 0; j < feature_num; ++j)
        {
            log_verbose("%f", train_x[i * feature_num + j]);
        }
        log_verbose("y =");
        for (size_t j = 0; j < target_num; ++j)
        {
            log_verbose("%f", train_y[i * target_num + j]);
        }
        log_verbose("precision matrix =");
        for (size_t i = 0; i < dim; ++i)
        {
            for (size_t j = 0; j < dim; ++j)
            {
                log_verbose("%f", pre[i * dim + j]);
            }
        }
        log_verbose("precision * mean =");
        for (size_t i = 0; i < dim; ++i)
        {
            for (size_t j = 0; j < target_num; ++j)
            {
                log_verbose("%f", pre_mean[i * target_num + j]);
            }
        }
    }

    log_debug("precision matrix =");
    for (size_t i = 0; i < dim * dim; ++i)
    {
        log_debug("%f", pre[i]);
    }
    log_debug("precision * mean =");
    for (size_t i = 0; i < dim * target_num; ++i)
    {
        log_debug("%f", pre_mean[i]);
    }

    ret = dsp_linear_regression_solve(
        feature_num,
        target_num,
        pre,
        pre_mean,
        mean,
        cov
    );

    if (ret)
    {
        log_debug("solve linear regression, mean =");
        for (size_t i = 0; i < dim; ++i)
        {
            for (size_t j = 0; j < target_num; ++j)
            {
                log_debug("%f", mean[i * target_num + j]);
            }
        }
        log_debug("covariance matrix = ");
        for (size_t i = 0; i < dim; ++i)
        {
            for (size_t j = 0; j < dim; ++j)
            {
                log_debug("%f", cov[i * dim + j]);
            }
        }

        /* 在测试集上计算 MSE 损失 */
        mse = 0.0f;
        for (size_t i = 0; i < test_sample_num; ++i)
        {
            float32_t w = 1.0f / (float32_t)(i + 1);
            float32_t dist;

            dsp_linear_regression_predict(
                feature_num,
                target_num,
                mean,
                test_x + i * feature_num,
                y_pred
            );

            dist = arm_euclidean_distance_f32(
                test_y + i * target_num,
                y_pred,
                target_num
            );
            mse = (1.0f - w) * mse + w / (float32_t)target_num * dist * dist;
        }

        log_info("done, test MSE = %f", mse);
    }

    else
    {
        log_error("cannot solve linear regression, code = %d", ret);
    }

#ifndef __GNUC__
    free(pre);
    free(pre_mean);
    free(mean);
    free(cov);
    free(y_pred);
#endif  /* __GNUC__ */
    return ret;
}

int test_linear_regression(void)
{
    int succ = 1;

    succ &= linear_regression(
        TRAIN_SAMPLE_NUM,
        TEST_SAMPLE_NUM,
        FEATURE_NUM_1_1,
        TARGET_NUM_1_1,
        train_x_1_1,
        train_y_1_1,
        test_x_1_1,
        test_y_1_1,
        0.0f
    );

    succ &= linear_regression(
        TRAIN_SAMPLE_NUM,
        TEST_SAMPLE_NUM,
        FEATURE_NUM_10_1,
        TARGET_NUM_10_1,
        (const float32_t *)train_x_10_1,
        train_y_10_1,
        (const float32_t *)test_x_10_1,
        test_y_10_1,
        0.0f
    );

    succ &= linear_regression(
        TRAIN_SAMPLE_NUM,
        TEST_SAMPLE_NUM,
        FEATURE_NUM_10_5,
        TARGET_NUM_10_5,
        (const float32_t *)train_x_10_5,
        (const float32_t *)train_y_10_5,
        (const float32_t *)test_x_10_5,
        (const float32_t *)test_y_10_5,
        0.0f
    );

    return succ;
}

int test_locally_weighted_linear_regression(void)
{
    float32_t pre[2][2];
    float32_t pre_mean[2];
    float32_t mean[2];
    float32_t cov[2][2];
    float32_t mse;

    srand(time(NULL));
    dsp_linear_regression_uni_init((float32_t *)pre, pre_mean);
    mse = 0.0f;

    for (size_t i = 0; i < 1000; ++i)
    {
        float32_t x = (float32_t)i * 0.01f;
        float32_t y_true = arm_sin_f32(x) + x;
        float32_t y = y_true + gaussian(0.0f, 0.1f);
        float32_t y_pred;
        float32_t mse_weight = 1.0f / (float32_t)(i + 1);

        dsp_linear_regression_uni_update(
            x,
            y,
            0.9f,
            1.0f,
            (float32_t *)pre,
            pre_mean
        );
        log_verbose(
            "precision matrix = ((%f, %f), (%f, %f)), precision * mean = (%f, %f)",
            pre[0][0], pre[0][1], pre[1][0], pre[1][1], pre_mean[0], pre_mean[1]
        );

        dsp_linear_regression_uni_solve(
            (float32_t *)pre,
            pre_mean,
            mean,
            (float32_t *)cov
        );
        y_pred = dsp_linear_regression_uni_predict(mean, x);
        mse = (1.0f - mse_weight) * mse + mse_weight * square_error(y_true, y_pred);

        log_debug(
            "x = %f, y_true = %f, y = %f, mean = (%f, %f), "
            "y_pred = %f, error = %f, MSE = %f",
            x, y_true, y, mean[0], mean[1], y_pred, square_error(y_true, y_pred), mse
        );
        log_verbose(
            "precision matrix = ((%f, %f), (%f, %f)), precision * mean = (%f, %f)",
            pre[0][0], pre[0][1], pre[1][0], pre[1][1], pre_mean[0], pre_mean[1]
        );
    }

    log_info("MSE = %f", mse);
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

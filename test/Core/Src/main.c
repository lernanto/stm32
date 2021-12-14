/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"

#include "log.h"
#include "hcsr04.h"
#include "l298n.h"
#include "misc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint32_t time;
  uint32_t distance;
} Hcsr04Record;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HCSR04_MAX_RECORD   10
#define HCSR04_EXPIRE_TIME  100

#define COUNTER_MIN_INTERVAL  13
#define COUNTER_SCALE         (65.0f * M_PI / 20)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static size_t g_log_write_max_retry = 4;

static Hcsr04Control g_hcsr04;
static Hcsr04Record g_hcsr04_records[HCSR04_MAX_RECORD + 1];
static Hcsr04Record *g_hcsr04_record_begin = g_hcsr04_records;
static Hcsr04Record *g_hcsr04_record_end = g_hcsr04_records;

static L298nControl g_l298n_left;
static L298nControl g_l298n_right;

static Counter g_counter_left;
static Counter g_counter_right;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int init(void)
{
  int result = 1;
  int res = 0;

  if (!(res = hcsr04_init(
    &g_hcsr04,
    HCSR04_TRIG_GPIO_Port,
    HCSR04_TRIG_Pin,
    HCSR04_ECHO_GPIO_Port,
    HCSR04_ECHO_Pin,
    0
  )))
  {
    log_error("initialize HC-SR04 error");
  }
  result |= res;

  if (!(res = l298n_init(&g_l298n_left, &htim2, TIM_CHANNEL_2, TIM_CHANNEL_1)))
  {
    log_error("initialize L298N left motor error");
  }
  result |= res;

  if (!(res = l298n_init(&g_l298n_right, &htim2, TIM_CHANNEL_3, TIM_CHANNEL_4)))
  {
    log_error("initialize L298N right motor error");
  }
  result |= res;

  if (!(res = counter_init(&g_counter_left, COUNTER_MIN_INTERVAL)))
  {
    log_error("initialize left counter error");
  }
  result |= res;

  if (!(res = counter_init(&g_counter_right, COUNTER_MIN_INTERVAL)))
  {
    log_error("initialize right counter error");
  }
  result |= res;

  return result;
}

static uint32_t hcsr04_get_distance(Hcsr04Control *hcsr04, float l2)
{
  uint32_t now = HAL_GetTick();
  Hcsr04Record *p = NULL;
  size_t n = 0.0f;
  float x = 0.0f;
  float y = 0.0f;
  float x2 = 0.0f;
  float xy = 0.0f;
  float det = 0.0f;
  float w = 0.0f;
  float b = 0.0f;

  while (
    (g_hcsr04_record_begin->time + HCSR04_EXPIRE_TIME < now)
    && (g_hcsr04_record_begin != g_hcsr04_record_end)
  )
  {
    circular_inc(g_hcsr04_records, ARRAYSIZE(g_hcsr04_records), g_hcsr04_record_begin);
  }

  if (circular_size(
    g_hcsr04_record_begin,
    g_hcsr04_record_end,
    ARRAYSIZE(g_hcsr04_records)
  ) == 0)
  {
    return HCSR04_INVALID_DISTANCE;
  }

  /* 线性回归求当前距离，迭代求解 */
  for (
    p = g_hcsr04_record_begin, n = 1, x = 0, y = 0, x2 = 0, xy = 0;
    p != g_hcsr04_record_end;
    circular_inc(g_hcsr04_records, ARRAYSIZE(g_hcsr04_records), p), ++n
  )
  {
    float a = 1.0f / n;
    x = (1 - a) * x + a * p->time;
    y = (1 - a) * y + a * p->distance;
    x2 = (1 - a) * x2 + a * p->time * p->time;
    xy = (1 - a) * xy + a * p->time * p->distance;
  }

  det = (x2 + l2) * (1 + l2) - x * x;
  if ((det < -1e-6f) || (det > 1e-6f))
  {
    /* 矩阵满秩，线性方程有特解 */
    w = (xy * (1 + l2) - x * y) / det;
    b = (y - w * x) / (1 + l2);
  }
  else
  {
    /* 方程有无穷多解，退化为求均值 */
    w = 0;
    b = y / (1 + l2);
  }

  return (uint32_t)(w * now + b);
}

static int test_log(void)
{
  static const char msg[] = "test log write";

  if (_log_write(msg, ARRAYSIZE(msg)) == 0)
  {
    g_log_write_max_retry = 1;
  }

  log_verbose("verbose level = %d", LOG_VERBOSE);
  HAL_Delay(100);
  log_debug("debug level = %d", LOG_DEBUG);
  HAL_Delay(100);
  log_info("info level = %d", LOG_INFO);
  HAL_Delay(100);
  log_warn("warn level = %d", LOG_WARN);
  HAL_Delay(100);
  log_error("error level = %d", LOG_ERROR);
  HAL_Delay(100);
  log_assert(1);

  return 1;
}

static int test_hcsr04(void)
{
  for (size_t i = 0; i < 100; ++i)
  {
    hcsr04_trigger(&g_hcsr04);
    HAL_Delay(10);
    if (hcsr04_get_distance(&g_hcsr04, 0) != HCSR04_INVALID_DISTANCE)
    {
      log_info("test HC-SR04 OK");
      return 1;
    }
  }

  log_error("HC-SR04 cannot get distance. may not work");
  return 0;
}

static int test_l298n(void)
{
  uint32_t time = 0;
  uint32_t left_count = 0;
  uint32_t right_count = 0;
  int32_t left_dist = 0;
  int32_t right_dist = 0;
  int32_t left_speed = 0;
  int32_t right_speed = 0;
  int32_t acc = 0;

  l298n_start(&g_l298n_left);
  l298n_start(&g_l298n_right);
  l298n_set_speed(&g_l298n_left, l298n_get_max_speed(&g_l298n_left));
  l298n_set_speed(&g_l298n_right, -l298n_get_max_speed(&g_l298n_right));

  for (
    size_t i = 0;
    (
      (0 == left_count) || (0 == right_count)
      || (0 == left_dist) || (0 == right_dist)
      || (0 == left_speed) || (0 == right_speed)
    ) && (i < 50);
  ++i)
  {
    HAL_Delay(20);
    counter_get_state(&g_counter_left, &time, &left_count, &left_dist, &left_speed, &acc);
    counter_get_state(&g_counter_right, &time, &right_count, &right_dist, &right_speed, &acc);
  }

  l298n_stop(&g_l298n_left);
  l298n_stop(&g_l298n_right);

  if ((0 == left_count) || (0 == left_dist) || (0 == left_speed))
  {
    log_error(
      "read left counter error, count = %u, dist = %d, speed = %d",
      left_count,
      left_dist,
      left_speed
    );
    return 0;
  }

  if ((0 == right_count) || (0 == right_dist) || (0 == right_speed))
  {
    log_error(
      "read right counter error, count = %u, dist = %d, speed = %d",
      right_count,
      right_dist,
      right_speed
    );
    return 0;
  }

  HAL_Delay(20);
  l298n_start(&g_l298n_left);
  l298n_start(&g_l298n_right);
  l298n_set_speed(&g_l298n_left, -l298n_get_max_speed(&g_l298n_left));
  l298n_set_speed(&g_l298n_right, l298n_get_max_speed(&g_l298n_right));

  for (
    size_t i = 0;
    (
      (0 == left_count) || (0 == right_count)
      || (0 == left_dist) || (0 == right_dist)
      || (0 == left_speed) || (0 == right_speed)
    ) && (i < 50);
  ++i)
  {
    HAL_Delay(20);
    counter_get_state(&g_counter_left, &time, &left_count, &left_dist, &left_speed, &acc);
    counter_get_state(&g_counter_right, &time, &right_count, &right_dist, &right_speed, &acc);
  }

  l298n_stop(&g_l298n_left);
  l298n_stop(&g_l298n_right);

  if ((0 == left_count) || (0 == left_dist) || (0 == left_speed))
  {
    log_error(
      "read left counter error, count = %u, dist = %d, speed = %d",
      left_count,
      left_dist,
      left_speed
    );
    return 0;
  }

  if ((0 == right_count) || (0 == right_dist) || (0 == right_speed))
  {
    log_error(
      "read right counter error, count = %u, dist = %d, speed = %d",
      right_count,
      right_dist,
      right_speed
    );
    return 0;
  }

  log_info("test L298N OK");
  return 1;
}

static int test(void)
{
  int result =  test_log()
    & test_hcsr04()
    & test_l298n();

  if (result)
  {
    log_info("all tests OK");
  }
  else
  {
    log_error("some test error");
  }

  return result;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  init();

  HAL_Delay(10000);
  test();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
  case HCSR04_ECHO_Pin:
    if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(g_hcsr04.echo_port, g_hcsr04.echo_pin))
    {
      uint32_t dist = HCSR04_INVALID_DISTANCE;
      log_assert(HCSR04_ECHO_BEGIN == g_hcsr04.state);
      hcsr04_echo_end(&g_hcsr04);

      if ((dist = hcsr04_read_nowait(&g_hcsr04)) != HCSR04_INVALID_DISTANCE)
      {
        g_hcsr04_record_end->time = g_hcsr04.echo_end_ms;
        g_hcsr04_record_end->distance = dist;
        circular_push(
          g_hcsr04_records,
          ARRAYSIZE(g_hcsr04_records),
          g_hcsr04_record_begin,
          g_hcsr04_record_end
        );
      }
    }
    else {
      hcsr04_echo_begin(&g_hcsr04);
    }
    break;

  case COUNTER_LEFT_Pin:
    counter_inc(&g_counter_left);
    break;

  case COUNTER_RIGHT_Pin:
    counter_inc(&g_counter_right);
    break;

  default:
    break;
  }
}

size_t _log_write(const void *buf, size_t len)
{
  uint8_t status = CDC_Transmit_FS((uint8_t *)buf, (uint16_t)len);
  size_t delay = 10;
  for (size_t i = 1; (status != USBD_OK) && (i < g_log_write_max_retry); ++i)
  {
    HAL_Delay(delay);
    status = CDC_Transmit_FS((uint8_t *)buf, (uint16_t)len);
    delay += delay;
  }

  return (USBD_OK == status) ? len : 0;
}

void _log_abort(void)
{
  log_error("system panic! please reset");
  while (1);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


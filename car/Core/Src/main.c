/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include <math.h>

#include "usbd_cdc_if.h"

#include "util.h"
#include "log.h"
#include "hcsr04.h"
#include "adxl345.h"
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

#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

#define COUNTER_MIN_INTERVAL  13
#define COUNTER_SCALE (65 * M_PI / 20)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static __IO uint32_t g_timerMs = 0;
static __IO char g_run = 0;
static Hcsr04Control g_hcsr04;
static Hcsr04Record g_hcsr04_records[HCSR04_MAX_RECORD + 1];
static Hcsr04Record *g_hcsr04_record_begin = g_hcsr04_records;
static Hcsr04Record *g_hcsr04_record_end = g_hcsr04_records;
static Counter g_counter_left;
static Counter g_counter_right;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t hcsr04_get_distance(Hcsr04Control *hcsr04, double l2)
{
  uint32_t now = HAL_GetTick();
  Hcsr04Record *p = NULL;
  size_t n = 0;
  double x = 0;
  double y = 0;
  double x2 = 0;
  double xy = 0;
  double det = 0;
  double w = 0;
  double b = 0;

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
    double a = 1.0 / n;
    x = (1 - a) * x + a * p->time;
    y = (1 - a) * y + a * p->distance;
    x2 = (1 - a) * x2 + a * p->time * p->time;
    xy = (1 - a) * xy + a * p->time * p->distance;
  }

  det = (x2 + l2) * (1 + l2) - x * x;
  /* TODO: 判断实数不为零 */
  if (det != 0)
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
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  HAL_StatusTypeDef status = HAL_OK;
  Adxl345I2cControl adxl345_i2c;
  Adxl345Control *padxl345 = (Adxl345Control *)&adxl345_i2c;
  L298nControl l298n_left;
  L298nControl l298n_right;

  uint32_t time = 0;
  uint32_t ir_front = 0;
  uint32_t distance = 0;
  int32_t x = 0;
  int32_t y = 0;
  int32_t z = 0;
  uint32_t left_count = 0;
  uint32_t right_count = 0;
  int32_t left_dist = 0;
  int32_t right_dist = 0;
  int32_t left_speed = 0;
  int32_t right_speed = 0;
  int32_t left_acc = 0;
  int32_t right_acc = 0;
  int dir = 0;
  int old_dir = 0;
  int32_t left_pulse = 0;
  int32_t right_pulse = 0;
  double speed = 0;
  double acc = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  counter_init(&g_counter_left, COUNTER_MIN_INTERVAL);
  counter_init(&g_counter_right, COUNTER_MIN_INTERVAL);
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
  g_run = 1;
  hcsr04_init(
    &g_hcsr04,
    HCSR04_TRIG_GPIO_Port,
    HCSR04_TRIG_Pin,
    HCSR04_ECHO_GPIO_Port,
    HCSR04_ECHO_Pin,
    0
  );
  adxl345_i2c_init(&adxl345_i2c, &hi2c1, ADXL345_ALT_ADDR, 10);
  status = adxl345_init(padxl345);
  status = l298n_init(&l298n_left, &htim2, TIM_CHANNEL_2, TIM_CHANNEL_1);
  status = l298n_init(&l298n_right, &htim2, TIM_CHANNEL_3, TIM_CHANNEL_4);
  status = l298n_start(&l298n_left);
  status = l298n_start(&l298n_right);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t max_speed = l298n_get_max_speed(&l298n_left);
    ir_front = (HAL_GPIO_ReadPin(IR_FRONT_GPIO_Port, IR_FRONT_Pin) == GPIO_PIN_RESET);
    status = hcsr04_trigger(&g_hcsr04);
    distance = hcsr04_get_distance(&g_hcsr04, 0);
    status = adxl345_get_acc(padxl345, &x, &y, &z);
    counter_get_state(&g_counter_left, &time, &left_count, &left_dist, &left_speed, &left_acc);
    counter_get_state(&g_counter_right, &time, &right_count, &right_dist, &right_speed, &right_acc);

    if ((HCSR04_INVALID_DISTANCE == distance) || (distance > 2000))
    {
      l298n_stop(&l298n_left);
      l298n_stop(&l298n_right);
    }
    else 
    {
      double d = 200.0 - distance;
      double sl = ((dir >= 0) ? left_dist : -left_dist) * COUNTER_SCALE;
      double sr = ((dir >= 0) ? right_dist : -right_dist) * COUNTER_SCALE;
      double s = (sl + sr) / 2;
      double vl = ((dir >= 0) ? left_speed : -left_speed) * COUNTER_SCALE;
      double vr = ((dir >= 0) ? right_speed : -right_speed) * COUNTER_SCALE;
      double v = (vl + vr) / 2;
      double al = ((dir >= 0) ? left_acc : -left_acc) * COUNTER_SCALE / 1000;
      double ar = ((dir >= 0) ? right_acc : -right_acc) * COUNTER_SCALE / 1000;
      double dsl = sl - s;
      double dsr = sr - s;
      double dvl = vl - v;
      double dvr = vr - v;

      double dest_a = MIN(MAX(-(d + v), -10), 10);
      double dest_al = dest_a - (dsl + dvl);
      double dest_ar = dest_a - (dsr + dvr);

      old_dir = dir;
      if (dest_a >= 0)
      {
        dir = 1;
        dest_al = MAX(dest_al, 0);
        dest_ar = MAX(dest_ar, 0);
      }
      else
      {
        dir = -1;
        dest_al = MIN(dest_al, 0);
        dest_ar = MIN(dest_ar, 0);
      }

      if (dir != old_dir)
      {
        /* 转向 */
        left_pulse = 0;
        right_pulse = 0;
        l298n_stop(&l298n_left);
        l298n_stop(&l298n_right);
        counter_reset(&g_counter_left);
        counter_reset(&g_counter_right);
      }
      else
      {
        if (al < dest_al)
        {
          left_pulse = MIN(left_pulse + 1, max_speed);
        }
        else if (al > dest_al)
        {
          left_pulse = MAX(left_pulse - 1, -max_speed);
        }

        if (ar < dest_ar)
        {
          right_pulse = MIN(right_pulse + 1, max_speed);
        }
        else if (ar > dest_ar)
        {
          right_pulse = MAX(right_pulse - 1, -max_speed);
        }
      }

      log_debug(
        "d = %f, sl = %f, sr = %f, vl = %f, vr = %f, al = %f, ar = %f, dest_al = %f, dest_ar = %f, left_pulse = %d, right_pulse = %d",
        d,
        sl,
        sr,
        vl,
        vr,
        al,
        ar,
        dest_al,
        dest_ar,
        left_pulse,
        right_pulse
      );
    }

    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, ((left_pulse != 0) || (right_pulse != 0)) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    l298n_set_speed(&l298n_left, left_pulse);
    l298n_set_speed(&l298n_right, right_pulse);
    if (l298n_get_state(&l298n_left) == L298N_STOP)
    {
      l298n_start(&l298n_left);
    }
    if (l298n_get_state(&l298n_right) == L298N_STOP)
    {
      l298n_start(&l298n_right);
    }

    HAL_Delay(10);
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
  case SW18015P_Pin:
    break;

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
  static const size_t max_retry = 0;

  uint8_t status = CDC_Transmit_FS((uint8_t *)buf, (uint16_t)len);
  for (size_t i = 1; (status != USBD_OK) && (i < max_retry); ++i)
  {
    HAL_Delay(20);
    status = CDC_Transmit_FS((uint8_t *)buf, (uint16_t)len);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

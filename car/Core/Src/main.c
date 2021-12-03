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
#include "usbd_cdc_if.h"

#include "util.h"
#include "hcsr04.h"
#include "adxl345.h"
#include "l298n.h"
#include "misc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HCSR04_INVALID_DISTANCE	UINT32_MAX

#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

#define ADXL345_ADDR		0xA6
#define ADXL345_ADDR_WRITE	0xA6
#define ADXL345_ADDR_READ	0xA7

#define ADXL345_DEVID		0x00
#define ADXL345_THRESH_TAP	0x1D
#define ADXL345_OFSX		0x1E
#define ADXL345_OFSY		0X1F
#define ADXL345_OFSZ		0x20
#define ADXL345_DUR			0x21
#define ADXL345_LATENT		0x22
#define ADXL345_WINDOW		0x23
#define ADXL345_THRESH_ACK	0x24
#define ADXL345_THRESH_INACT	0x25
#define ADXL345_TIME_INACT	0x26
#define ADXL345_ACT_INACT_CTL	0x27
#define ADXL345_THRESH_FF	0x28
#define ADXL345_TIME_FF		0x29
#define ADXL345_TAP_AXES	0x2A
#define ADXL345_ACT_TAP_STATUS	0x2B
#define ADXL345_BW_RATE		0x2C
#define ADXL345_POWER_CTL	0x2D
#define ADXL345_INT_ENABLE	0x2E
#define ADXL345_INT_MAP		0x2F
#define ADXL345_INT_SOURCE	0x30
#define ADXL345_DATA_FORMAT	0x31
#define ADXL345_DATAX0		0x32
#define ADXL345_DATAX1		0x33
#define ADXL345_DATAY0		0x34
#define ADXL345_DATAY1		0x35
#define ADXL345_DATAZ0		0x36
#define ADXL345_DATAZ1		0x37
#define ADXL345_FIFO_CTL	0x38
#define ADXL345_FIFO_STATUS	0x39

#define ADXL345_RW			0x80
#define ADXL345_MB			0x40

#define COUNTER_MIN_INTERVAL	13
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ARRAYSIZE(a)  (sizeof(a) / sizeof((a)[0]))
#define MIN(x, y)	(((x) < (y)) ? (x) : (y))
#define MAX(x, y)	(((x) > (y)) ? (x) : (y))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static __IO uint32_t g_timerMs = 0;
static __IO char g_run = 0;
static char g_strBuf[256];
static Hcsr04Control g_hcsr04;
static Counter g_counter_left;
static Counter g_counter_right;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t DelayUs(uint32_t us)
{
  if (us == 0)
  {
    return 0;
  }
  else
  {
    uint32_t startMs = HAL_GetTick();
    uint32_t startUs = SysTick->VAL;
    uint32_t endMs = 0;
    uint32_t endUs = 0;
    uint32_t timeUs = 0;

    do
    {
      endMs = HAL_GetTick();
      endUs = SysTick->VAL;
      timeUs = (endMs - startMs) * 1000 + ((startUs >= endUs) ? (startUs - endUs) : (startUs + SysTick->LOAD - endUs)) / 72;
    } while (timeUs < us);

    return timeUs;
  }
}

static uint32_t HCSR04_Read(void)
{
  uint32_t startMs = 0;
  uint32_t startUs = 0;
  uint32_t endMs = 0;
  uint32_t endUs = 0;
  uint32_t timeUs = 0;
  GPIO_PinState echo = GPIO_PIN_RESET;

  HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_SET);
  DelayUs(20);
  HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);

  startMs = HAL_GetTick();
  do
  {
    echo = HAL_GPIO_ReadPin(HCSR04_ECHO_GPIO_Port, HCSR04_ECHO_Pin);
  } while ((GPIO_PIN_RESET == echo) && (HAL_GetTick() < startMs + 1));

  if (GPIO_PIN_RESET == echo)
  {
    return HCSR04_INVALID_DISTANCE;
  }
  else
  {
    startMs = HAL_GetTick();
    startUs = SysTick->VAL;
    while (GPIO_PIN_SET == HAL_GPIO_ReadPin(HCSR04_ECHO_GPIO_Port, HCSR04_ECHO_Pin));
    endMs = HAL_GetTick();
    endUs = SysTick->VAL;
    timeUs = (endMs - startMs) * 1000 + ((startUs >= endUs) ? (startUs - endUs) : (startUs + SysTick->LOAD - endUs)) / 72;

    return timeUs * 170 / 1000;
  }
}

static uint32_t HCSR04_GetDistance(Hcsr04Control *hcsr04)
{
  static uint32_t distances[] = {0, 0, 0, 0, 0};
  static size_t current = UINT32_MAX;
  size_t minIdx = 0;
  size_t maxIdx = 0;
  size_t count = 0;
  uint32_t sum = 0;

  if (current >= ARRAYSIZE(distances))
  {
    for (size_t i = 0; i < ARRAYSIZE(distances); ++i)
    {
      distances[i] = hcsr04_get_dist(hcsr04);
    }
    current = 0;
  }
  else
  {
    distances[current] = hcsr04_get_dist_sync(hcsr04);
    if (++current >= ARRAYSIZE(distances))
    {
      current = 0;
    }
  }

  minIdx = 0;
  maxIdx = 0;
  for (size_t i = 0; i < ARRAYSIZE(distances); ++i)
  {
    if (distances[i] < distances[minIdx])
    {
      minIdx = i;
    }
    else
    {
      if (distances[i] > distances[maxIdx])
      {
        maxIdx = i;
      }
    }
  }

  count = 0;
  sum = 0;
  for (size_t i = 0; i < ARRAYSIZE(distances); ++i)
  {
    if ((i != minIdx) && (i != maxIdx) && (distances[i] != HCSR04_INVALID_DISTANCE))
    {
      ++count;
      sum += distances[i];
    }
  }

  if (count >= 3)
  {
	return sum / count;
  }
  else
  {
	return HCSR04_INVALID_DISTANCE;
  }
}

static void Motor_SetSpeed(uint32_t motor, int32_t speed)
{
  uint32_t channel1 = ((MOTOR_LEFT == motor) ? TIM_CHANNEL_1 : TIM_CHANNEL_3);
  uint32_t channel2 = ((MOTOR_LEFT == motor) ? TIM_CHANNEL_2 : TIM_CHANNEL_4);
  uint32_t pulse1 = 0;
  uint32_t pulse2 = 0;
  TIM_OC_InitTypeDef sConfigOC = {0};

  if (0 == speed)
  {
	pulse1 = 0;
	pulse2 = 0;
  }
  else if (speed < 0)
  {
	pulse1 = 0;
	pulse2 = -speed;
  }
  else
  {
	pulse1 = speed;
	pulse2 = 0;
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulse1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, channel1);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulse2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, channel2);

  HAL_TIM_PWM_Start(&htim3, channel1);
  HAL_TIM_PWM_Start(&htim3, channel2);
}

static HAL_StatusTypeDef ADXL345_Read(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t *data, uint32_t timeout)
{
  return HAL_I2C_Mem_Read(hi2c, ADXL345_ADDR, addr, I2C_MEMADD_SIZE_8BIT, data, 1, timeout);
}

static HAL_StatusTypeDef ADXL345_ReadMultiByte(I2C_HandleTypeDef *hi2c, uint8_t addr, uint16_t *data, uint32_t timeout)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t buf[2] = {0};

  status = HAL_I2C_Mem_Read(hi2c, ADXL345_ADDR, addr | ADXL345_MB, I2C_MEMADD_SIZE_8BIT, buf, 2, timeout);

  *data = (((uint16_t)buf[1] << 8) | buf[0]);
  return status;
}

static HAL_StatusTypeDef ADXL345_Write(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t data, uint32_t timeout)
{
  return HAL_I2C_Mem_Write(hi2c, ADXL345_ADDR, addr, I2C_MEMADD_SIZE_8BIT, &data, 1, timeout);
}

#if 0
static HAL_StatusTypeDef ADXL345_Read(uint8_t addr, uint8_t *data, uint32_t timeout)
{
  HAL_StatusTypeDef status = HAL_OK;

  addr |= 0x80;

  HAL_Delay(10);
  HAL_GPIO_WritePin(ADXL345_CS_GPIO_Port, ADXL345_CS_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  status = HAL_SPI_Transmit(&hspi1, &addr, 1, timeout);
  status = HAL_SPI_Receive(&hspi1, data, 1, timeout);
  HAL_GPIO_WritePin(ADXL345_CS_GPIO_Port, ADXL345_CS_Pin, GPIO_PIN_SET);

  return status;
}

static HAL_StatusTypeDef ADXL345_Read16(uint8_t addr, int16_t *data, uint32_t timeout)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t buf[2] = {0};

  addr |= 0x80;

  HAL_Delay(10);
  HAL_GPIO_WritePin(ADXL345_CS_GPIO_Port, ADXL345_CS_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  status = HAL_SPI_Transmit(&hspi1, &addr, 1, timeout);
  status = HAL_SPI_Receive(&hspi1, buf, 2, timeout);
  HAL_GPIO_WritePin(ADXL345_CS_GPIO_Port, ADXL345_CS_Pin, GPIO_PIN_SET);

  *data = (((int16_t)buf[1] << 8) | buf[0]);
  return status;
}

static HAL_StatusTypeDef ADXL345_Write(uint8_t addr, uint8_t data, uint32_t timeout)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t buf[2] = {0};

  buf[0] = addr;
  buf[1] = data;

  HAL_Delay(10);
  HAL_GPIO_WritePin(ADXL345_CS_GPIO_Port, ADXL345_CS_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  status = HAL_SPI_Transmit(&hspi1, buf, 2, timeout);
  HAL_GPIO_WritePin(ADXL345_CS_GPIO_Port, ADXL345_CS_Pin, GPIO_PIN_SET);

  return status;
}
#endif

static HAL_StatusTypeDef ADXL345_Init(I2C_HandleTypeDef *hi2c)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t devid = 0;

  for (size_t i = 0; i < 10; ++i)
  {
    status = ADXL345_Write(hi2c, ADXL345_DATA_FORMAT, 0x0B, 100);	// SPI 4线模式，全分辨率，量�???????????? +-16g

    status = ADXL345_Read(hi2c, ADXL345_DEVID, &devid, 100);
    if ((HAL_OK == status) && (0xE5 == devid))
    {
      break;
    }
  }

  if (status != HAL_OK) {
    return status;
  }
  else if (devid != 0xE5) {
    return HAL_ERROR;
  }

  status = ADXL345_Write(hi2c, ADXL345_OFSX, 0, 100);				// 设置 X 轴偏移，下面类似
  status = ADXL345_Write(hi2c, ADXL345_OFSY, 0, 100);
  status = ADXL345_Write(hi2c, ADXL345_OFSZ, 0, 100);
  status = ADXL345_Write(hi2c, ADXL345_POWER_CTL, 0x08, 100);		// 测量模式
  status = ADXL345_Write(hi2c, ADXL345_BW_RATE, 0x0E, 100);		// 输出速率800Hz
  status = ADXL345_Write(hi2c, ADXL345_INT_ENABLE, 0x00, 100);
  status = ADXL345_Write(hi2c, ADXL345_INT_MAP, 0x00, 100);
  status = ADXL345_Write(hi2c, ADXL345_FIFO_CTL, 0x00, 100);		// 旁路模式，不使用 FIFO

  return status;
}

static HAL_StatusTypeDef ADXL345_GetAcc(I2C_HandleTypeDef *hi2c, int32_t *x, int32_t *y, int32_t *z)
{
  HAL_StatusTypeDef status = HAL_OK;

#if 0
  uint8_t data0 = 0;
  uint8_t data1 = 0;

  status = ADXL345_Read(hi2c, ADXL345_DATAX0, &data0, 100);
  status = ADXL345_Read(hi2c, ADXL345_DATAX1, &data1, 100);
  *x = (((int32_t)(int8_t)data1 << 8) | data0);

  status = ADXL345_Read(hi2c, ADXL345_DATAY0, &data0, 100);
  status = ADXL345_Read(hi2c, ADXL345_DATAY1, &data1, 100);
  *y = (((int32_t)(int8_t)data1 << 8) | data0);

  status = ADXL345_Read(hi2c, ADXL345_DATAZ0, &data0, 100);
  status = ADXL345_Read(hi2c, ADXL345_DATAZ1, &data1, 100);
  *z = (((int32_t)(int8_t)data1 << 8) | data0);
#endif

  uint16_t data = 0;

  status = ADXL345_ReadMultiByte(hi2c, ADXL345_DATAX0, &data, 10);
  *x = (int16_t)data;

  status = ADXL345_ReadMultiByte(hi2c, ADXL345_DATAY0, &data, 10);
  *y = (int16_t)data;

  status = ADXL345_ReadMultiByte(hi2c, ADXL345_DATAZ0, &data, 10);
  *z = (int16_t)data;

  return status;
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
  int32_t left_speed = 0;
  int32_t right_speed = 0;
  int32_t left_acc = 0;
  int32_t right_acc = 0;
  int32_t left_pulse = 0;
  int32_t right_pulse = 0;
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
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  g_run = 1;
  hcsr04_init(&g_hcsr04, HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, HCSR04_ECHO_GPIO_Port, HCSR04_ECHO_Pin, 10);
  // status = ADXL345_Init(&hi2c1);
  adxl345_i2c_init(&adxl345_i2c, &hi2c1, ADXL345_ALT_ADDR, 10);
  status = adxl345_init(padxl345);
  status = l298n_init(&l298n_left, &htim2, TIM_CHANNEL_2, TIM_CHANNEL_1);
  status = l298n_init(&l298n_right, &htim2, TIM_CHANNEL_3, TIM_CHANNEL_4);
  // HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	/*
	if (!g_run)
	{
	  continue;
	}
	*/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// uint8_t data = ADXL345_Test(ADXL345_DEVID);

    // status = ADXL345_GetAcc(&hi2c1, &x, &y, &z);
	ir_front = (HAL_GPIO_ReadPin(IR_FRONT_GPIO_Port, IR_FRONT_Pin) == GPIO_PIN_RESET);
	distance = HCSR04_GetDistance(&g_hcsr04);
	status = adxl345_get_acc(padxl345, &x, &y, &z);
	counter_get_state(&g_counter_left, &time, &left_count, &left_speed, &left_acc);
	counter_get_state(&g_counter_right, &time, &right_count, &right_speed, &right_acc);

    if ((HCSR04_INVALID_DISTANCE == distance) || (distance > 1500)
		|| ((distance > 400) && (distance < 600)))
    {
      left_pulse = right_pulse = 0;
    }
    else if (distance < 500)
    {
      left_pulse = right_pulse = MIN(((int32_t)distance - 500) * 2, -200);
    }
    else
    {
      left_pulse = right_pulse = MAX((distance - 500) * 2, 200);
    }

    if (ir_front)
    {
    	left_pulse = MIN(left_pulse, 0);
    	right_pulse = MIN(right_pulse, 0);
    }

    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, ((left_pulse != 0) || (right_pulse != 0)) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    // Motor_SetSpeed(MOTOR_LEFT, speed);
    // Motor_SetSpeed(MOTOR_RIGHT, speed);
    l298n_set_speed(&l298n_left, left_pulse);
    l298n_set_speed(&l298n_right, right_pulse);

    snprintf(
      g_strBuf,
	  ARRAYSIZE(g_strBuf),
	  "%lu,%lu,%lu,%ld,%ld,%ld,%lu,%lu,%ld,%ld,%ld,%ld,%ld,%ld\n",
	  time,
	  ir_front,
	  distance,
	  x,
	  y,
	  z,
	  left_count,
	  right_count,
	  left_speed,
	  right_speed,
	  left_acc,
	  right_acc,
	  left_pulse,
	  right_pulse
	);
    CDC_Transmit_FS((uint8_t*)g_strBuf, strlen(g_strBuf));

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
#if 0
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *tim_baseHandle)
{
  if (tim_baseHandle->Instance == htim2.Instance)
  {
    ++g_timerMs;
  }
}
#endif

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
  case SW18015P_Pin:
    break;

  case HCSR04_ECHO_Pin:
	if (HCSR04_ECHO_BEGIN == g_hcsr04.state)
	{
	  hcsr04_echo_end(&g_hcsr04);
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

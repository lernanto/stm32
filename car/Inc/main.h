/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW18015P_Pin GPIO_PIN_0
#define SW18015P_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOA
#define HCSR04_TRIG_Pin GPIO_PIN_4
#define HCSR04_TRIG_GPIO_Port GPIOA
#define HCSR04_ECHO_Pin GPIO_PIN_5
#define HCSR04_ECHO_GPIO_Port GPIOA
#define L298N_IN1_Pin GPIO_PIN_6
#define L298N_IN1_GPIO_Port GPIOA
#define L298N_IN2_Pin GPIO_PIN_7
#define L298N_IN2_GPIO_Port GPIOA
#define L298N_IN3_Pin GPIO_PIN_0
#define L298N_IN3_GPIO_Port GPIOB
#define L298N_IN4_Pin GPIO_PIN_1
#define L298N_IN4_GPIO_Port GPIOB
#define COUNTER_LEFT_Pin GPIO_PIN_10
#define COUNTER_LEFT_GPIO_Port GPIOB
#define COUNTER_RIGHT_Pin GPIO_PIN_11
#define COUNTER_RIGHT_GPIO_Port GPIOB
#define KEY_Pin GPIO_PIN_8
#define KEY_GPIO_Port GPIOA
#define KEY_EXTI_IRQn EXTI9_5_IRQn
#define IR_FRONT_Pin GPIO_PIN_9
#define IR_FRONT_GPIO_Port GPIOA
#define TCRT5000_Pin GPIO_PIN_10
#define TCRT5000_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

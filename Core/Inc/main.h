/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FORWARD_IN1_Pin GPIO_PIN_3
#define FORWARD_IN1_GPIO_Port GPIOE
#define FORWARD_IN2_Pin GPIO_PIN_4
#define FORWARD_IN2_GPIO_Port GPIOE
#define FORWARD_IN3_Pin GPIO_PIN_5
#define FORWARD_IN3_GPIO_Port GPIOE
#define FORWARD_IN4_Pin GPIO_PIN_6
#define FORWARD_IN4_GPIO_Port GPIOE
#define KEY_PB0_Pin GPIO_PIN_0
#define KEY_PB0_GPIO_Port GPIOB
#define KEY_PB0_EXTI_IRQn EXTI0_IRQn
#define BACK_IN1_Pin GPIO_PIN_11
#define BACK_IN1_GPIO_Port GPIOD
#define BACK_IN2_Pin GPIO_PIN_12
#define BACK_IN2_GPIO_Port GPIOD
#define BACK_IN3_Pin GPIO_PIN_14
#define BACK_IN3_GPIO_Port GPIOD
#define BACK_IN4_Pin GPIO_PIN_15
#define BACK_IN4_GPIO_Port GPIOD
#define Board_LED_Pin GPIO_PIN_8
#define Board_LED_GPIO_Port GPIOA
#define PC11_LED_Pin GPIO_PIN_11
#define PC11_LED_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

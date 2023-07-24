/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENA_2_Pin GPIO_PIN_2
#define ENA_2_GPIO_Port GPIOE
#define DIR_2_Pin GPIO_PIN_3
#define DIR_2_GPIO_Port GPIOE
#define ENA_1_Pin GPIO_PIN_4
#define ENA_1_GPIO_Port GPIOE
#define DIR_1_Pin GPIO_PIN_5
#define DIR_1_GPIO_Port GPIOE
#define BOARD_LED_Pin GPIO_PIN_0
#define BOARD_LED_GPIO_Port GPIOC
#define BOARD_KEY_Pin GPIO_PIN_1
#define BOARD_KEY_GPIO_Port GPIOC
#define SWITCH_UP_Pin GPIO_PIN_2
#define SWITCH_UP_GPIO_Port GPIOC
#define SWITCH_DOWN_Pin GPIO_PIN_3
#define SWITCH_DOWN_GPIO_Port GPIOC
#define ROCKER_KEY_Pin GPIO_PIN_0
#define ROCKER_KEY_GPIO_Port GPIOA
#define ROCKER_X_Pin GPIO_PIN_1
#define ROCKER_X_GPIO_Port GPIOA
#define ROCKER_Y_Pin GPIO_PIN_2
#define ROCKER_Y_GPIO_Port GPIOA
#define LOCK_LED_Pin GPIO_PIN_3
#define LOCK_LED_GPIO_Port GPIOA
#define PUL_3_Pin GPIO_PIN_6
#define PUL_3_GPIO_Port GPIOA
#define PUL_4_Pin GPIO_PIN_7
#define PUL_4_GPIO_Port GPIOA
#define LIM_IN_1_Pin GPIO_PIN_0
#define LIM_IN_1_GPIO_Port GPIOD
#define LIM_IN_1_EXTI_IRQn EXTI0_IRQn
#define LIM_CHECK_1_Pin GPIO_PIN_1
#define LIM_CHECK_1_GPIO_Port GPIOD
#define LIM_IN_2_Pin GPIO_PIN_2
#define LIM_IN_2_GPIO_Port GPIOD
#define LIM_IN_2_EXTI_IRQn EXTI2_IRQn
#define LIM_CHECK_2_Pin GPIO_PIN_3
#define LIM_CHECK_2_GPIO_Port GPIOD
#define LIM_IN_3_Pin GPIO_PIN_4
#define LIM_IN_3_GPIO_Port GPIOD
#define LIM_IN_3_EXTI_IRQn EXTI4_IRQn
#define LIM_CHECK_3_Pin GPIO_PIN_5
#define LIM_CHECK_3_GPIO_Port GPIOD
#define LIM_IN_4_Pin GPIO_PIN_6
#define LIM_IN_4_GPIO_Port GPIOD
#define LIM_IN_4_EXTI_IRQn EXTI9_5_IRQn
#define LIM_CHECK_4_Pin GPIO_PIN_7
#define LIM_CHECK_4_GPIO_Port GPIOD
#define ENA_4_Pin GPIO_PIN_6
#define ENA_4_GPIO_Port GPIOB
#define DIR_4_Pin GPIO_PIN_7
#define DIR_4_GPIO_Port GPIOB
#define PUL_1_Pin GPIO_PIN_8
#define PUL_1_GPIO_Port GPIOB
#define PUL_2_Pin GPIO_PIN_9
#define PUL_2_GPIO_Port GPIOB
#define ENA_3_Pin GPIO_PIN_0
#define ENA_3_GPIO_Port GPIOE
#define DIR_3_Pin GPIO_PIN_1
#define DIR_3_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

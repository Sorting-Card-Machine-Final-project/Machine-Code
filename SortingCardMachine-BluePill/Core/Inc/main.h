/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define DC_PWM1_Pin GPIO_PIN_0
#define DC_PWM1_GPIO_Port GPIOA
#define DC_PWM2_Pin GPIO_PIN_1
#define DC_PWM2_GPIO_Port GPIOA
#define switch1_Pin GPIO_PIN_12
#define switch1_GPIO_Port GPIOB
#define switch2_Pin GPIO_PIN_13
#define switch2_GPIO_Port GPIOB
#define STEPPER1_DIR_Pin GPIO_PIN_4
#define STEPPER1_DIR_GPIO_Port GPIOB
#define STEPPER1_STEP_Pin GPIO_PIN_5
#define STEPPER1_STEP_GPIO_Port GPIOB
#define STEPPER2_DIR_Pin GPIO_PIN_6
#define STEPPER2_DIR_GPIO_Port GPIOB
#define STEPPER2_STEP_Pin GPIO_PIN_7
#define STEPPER2_STEP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

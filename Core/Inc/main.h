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
#include "stm32l4xx_hal.h"

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
#define DRV_ENABLE_Pin GPIO_PIN_4
#define DRV_ENABLE_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define BUTTON_UP_Pin GPIO_PIN_5
#define BUTTON_UP_GPIO_Port GPIOC
#define LCD_D4_Pin GPIO_PIN_1
#define LCD_D4_GPIO_Port GPIOB
#define LCD_D5_Pin GPIO_PIN_2
#define LCD_D5_GPIO_Port GPIOB
#define LCD_D6_Pin GPIO_PIN_11
#define LCD_D6_GPIO_Port GPIOB
#define LCD_D7_Pin GPIO_PIN_12
#define LCD_D7_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_13
#define LCD_RS_GPIO_Port GPIOB
#define LCD_RW_Pin GPIO_PIN_14
#define LCD_RW_GPIO_Port GPIOB
#define LCD_E_Pin GPIO_PIN_15
#define LCD_E_GPIO_Port GPIOB
#define BUTTON_RIGHT_Pin GPIO_PIN_6
#define BUTTON_RIGHT_GPIO_Port GPIOC
#define BUTTON_DOWN_Pin GPIO_PIN_8
#define BUTTON_DOWN_GPIO_Port GPIOC
#define BUTTON_LEFT_Pin GPIO_PIN_9
#define BUTTON_LEFT_GPIO_Port GPIOC
#define DRV_MS1_Pin GPIO_PIN_10
#define DRV_MS1_GPIO_Port GPIOC
#define DRV_MS2_Pin GPIO_PIN_11
#define DRV_MS2_GPIO_Port GPIOC
#define DRV_I1_Pin GPIO_PIN_12
#define DRV_I1_GPIO_Port GPIOC
#define DRV_DIR_Pin GPIO_PIN_2
#define DRV_DIR_GPIO_Port GPIOD
#define BUTTON_ENTER_Pin GPIO_PIN_8
#define BUTTON_ENTER_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

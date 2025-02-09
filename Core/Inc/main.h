/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

typedef enum {
  SensorLevel0,
  SensorLevel1,
  SensorLevel2,
  SensorLevel3,
  SensorLevel4,
  SensorLevelInvalid = 255,
} SensorLevel;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern UART_HandleTypeDef huart1;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define LOG(format, args...)  printf("[%s][%s():%d] "format"\n", __FILE_NAME__, __func__, __LINE__, ##args)
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_RED_Pin GPIO_PIN_1
#define LED_RED_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_2
#define LED_GREEN_GPIO_Port GPIOA
#define LED_BLUE_Pin GPIO_PIN_3
#define LED_BLUE_GPIO_Port GPIOA
#define SENSOR0_Pin GPIO_PIN_10
#define SENSOR0_GPIO_Port GPIOB
#define SENSOR1_Pin GPIO_PIN_11
#define SENSOR1_GPIO_Port GPIOB
#define SENSOR_EMPTY_Pin GPIO_PIN_12
#define SENSOR_EMPTY_GPIO_Port GPIOB
#define PUMP_Pin GPIO_PIN_13
#define PUMP_GPIO_Port GPIOB
#define VALVE0_Pin GPIO_PIN_14
#define VALVE0_GPIO_Port GPIOB
#define VALVE1_Pin GPIO_PIN_15
#define VALVE1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

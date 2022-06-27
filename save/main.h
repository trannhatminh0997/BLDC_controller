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
#define sig_led_Pin GPIO_PIN_13
#define sig_led_GPIO_Port GPIOC
#define HC_Pin GPIO_PIN_6
#define HC_GPIO_Port GPIOA
#define LC_Pin GPIO_PIN_7
#define LC_GPIO_Port GPIOA
#define CSS_Pin GPIO_PIN_1
#define CSS_GPIO_Port GPIOB
#define EN_Pin GPIO_PIN_10
#define EN_GPIO_Port GPIOB
#define FO_Pin GPIO_PIN_11
#define FO_GPIO_Port GPIOB
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define SPI1_NSS_Pin GPIO_PIN_15
#define SPI1_NSS_GPIO_Port GPIOA
#define HA_Pin GPIO_PIN_6
#define HA_GPIO_Port GPIOB
#define LA_Pin GPIO_PIN_7
#define LA_GPIO_Port GPIOB
#define HB_Pin GPIO_PIN_8
#define HB_GPIO_Port GPIOB
#define LB_Pin GPIO_PIN_9
#define LB_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

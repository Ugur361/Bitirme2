/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#define button5_Pin GPIO_PIN_11
#define button5_GPIO_Port GPIOE
#define button6_Pin GPIO_PIN_12
#define button6_GPIO_Port GPIOE
#define temp_ledgreen_Pin GPIO_PIN_13
#define temp_ledgreen_GPIO_Port GPIOE
#define temp_ledyellow_Pin GPIO_PIN_14
#define temp_ledyellow_GPIO_Port GPIOE
#define temp_ledred_Pin GPIO_PIN_15
#define temp_ledred_GPIO_Port GPIOE
#define led_red_Pin GPIO_PIN_10
#define led_red_GPIO_Port GPIOB
#define button1_Pin GPIO_PIN_12
#define button1_GPIO_Port GPIOB
#define button2_Pin GPIO_PIN_13
#define button2_GPIO_Port GPIOB
#define button3_Pin GPIO_PIN_14
#define button3_GPIO_Port GPIOB
#define buttonhome_Pin GPIO_PIN_15
#define buttonhome_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

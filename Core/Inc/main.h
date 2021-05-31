/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f7xx_hal.h"

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
#define EN_1_Pin GPIO_PIN_13
#define EN_1_GPIO_Port GPIOC
#define WR_1_Pin GPIO_PIN_14
#define WR_1_GPIO_Port GPIOC
#define CS_1_Pin GPIO_PIN_15
#define CS_1_GPIO_Port GPIOC
#define A4_1_Pin GPIO_PIN_0
#define A4_1_GPIO_Port GPIOF
#define A3_1_Pin GPIO_PIN_1
#define A3_1_GPIO_Port GPIOF
#define A2_1_Pin GPIO_PIN_2
#define A2_1_GPIO_Port GPIOF
#define A1_1_Pin GPIO_PIN_3
#define A1_1_GPIO_Port GPIOF
#define A0_1_Pin GPIO_PIN_4
#define A0_1_GPIO_Port GPIOF
#define EN_0_Pin GPIO_PIN_0
#define EN_0_GPIO_Port GPIOA
#define WR_0_Pin GPIO_PIN_1
#define WR_0_GPIO_Port GPIOA
#define CS_0_Pin GPIO_PIN_2
#define CS_0_GPIO_Port GPIOA
#define A4_0_Pin GPIO_PIN_3
#define A4_0_GPIO_Port GPIOA
#define A3_0_Pin GPIO_PIN_4
#define A3_0_GPIO_Port GPIOA
#define A2_0_Pin GPIO_PIN_5
#define A2_0_GPIO_Port GPIOA
#define A1_0_Pin GPIO_PIN_6
#define A1_0_GPIO_Port GPIOA
#define A0_0_Pin GPIO_PIN_7
#define A0_0_GPIO_Port GPIOA
#define EN_3_Pin GPIO_PIN_7
#define EN_3_GPIO_Port GPIOE
#define WR_3_Pin GPIO_PIN_8
#define WR_3_GPIO_Port GPIOE
#define CS_3_Pin GPIO_PIN_9
#define CS_3_GPIO_Port GPIOE
#define A4_3_Pin GPIO_PIN_10
#define A4_3_GPIO_Port GPIOE
#define A3_3_Pin GPIO_PIN_11
#define A3_3_GPIO_Port GPIOE
#define A2_3_Pin GPIO_PIN_12
#define A2_3_GPIO_Port GPIOE
#define A1_3_Pin GPIO_PIN_13
#define A1_3_GPIO_Port GPIOE
#define A0_3_Pin GPIO_PIN_14
#define A0_3_GPIO_Port GPIOE
#define WR_2_Pin GPIO_PIN_2
#define WR_2_GPIO_Port GPIOG
#define CS_2_Pin GPIO_PIN_3
#define CS_2_GPIO_Port GPIOG
#define A4_2_Pin GPIO_PIN_4
#define A4_2_GPIO_Port GPIOG
#define A3_2_Pin GPIO_PIN_5
#define A3_2_GPIO_Port GPIOG
#define A2_2_Pin GPIO_PIN_6
#define A2_2_GPIO_Port GPIOG
#define A1_2_Pin GPIO_PIN_7
#define A1_2_GPIO_Port GPIOG
#define A0_2_Pin GPIO_PIN_8
#define A0_2_GPIO_Port GPIOG
#define EN_2_Pin GPIO_PIN_9
#define EN_2_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

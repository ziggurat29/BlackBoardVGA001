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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define KEY1_Pin GPIO_PIN_3
#define KEY1_GPIO_Port GPIOE
#define KEY1_EXTI_IRQn EXTI3_IRQn
#define KEY0_Pin GPIO_PIN_4
#define KEY0_GPIO_Port GPIOE
#define KEY0_EXTI_IRQn EXTI4_IRQn
#define BOGO_CARDDETECT_Pin GPIO_PIN_13
#define BOGO_CARDDETECT_GPIO_Port GPIOC
#define LED_D2_Pin GPIO_PIN_6
#define LED_D2_GPIO_Port GPIOA
#define LED_D3_Pin GPIO_PIN_7
#define LED_D3_GPIO_Port GPIOA
#define FLASH_CS_Pin GPIO_PIN_0
#define FLASH_CS_GPIO_Port GPIOB
#define R0_Pin GPIO_PIN_8
#define R0_GPIO_Port GPIOE
#define R1_Pin GPIO_PIN_9
#define R1_GPIO_Port GPIOE
#define R2_Pin GPIO_PIN_10
#define R2_GPIO_Port GPIOE
#define G0_Pin GPIO_PIN_11
#define G0_GPIO_Port GPIOE
#define G1_Pin GPIO_PIN_12
#define G1_GPIO_Port GPIOE
#define G2_Pin GPIO_PIN_13
#define G2_GPIO_Port GPIOE
#define B0_Pin GPIO_PIN_14
#define B0_GPIO_Port GPIOE
#define B1_Pin GPIO_PIN_15
#define B1_GPIO_Port GPIOE
#define PS2_CK_Pin GPIO_PIN_6
#define PS2_CK_GPIO_Port GPIOC
#define PS2_DATA_Pin GPIO_PIN_7
#define PS2_DATA_GPIO_Port GPIOC
#define FLASH_SCK_Pin GPIO_PIN_3
#define FLASH_SCK_GPIO_Port GPIOB
#define FLASH_MISO_Pin GPIO_PIN_4
#define FLASH_MISO_GPIO_Port GPIOB
#define FLASH_MOSI_Pin GPIO_PIN_5
#define FLASH_MOSI_GPIO_Port GPIOB
#define HSYNC_Pin GPIO_PIN_6
#define HSYNC_GPIO_Port GPIOB
#define VSYNC_Pin GPIO_PIN_7
#define VSYNC_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

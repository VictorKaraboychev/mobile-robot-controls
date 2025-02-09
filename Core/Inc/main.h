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
#include "stm32h7xx_hal.h"

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

  int _write(int file, char *ptr, int length);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUZZER_PA2_Pin GPIO_PIN_2
#define BUZZER_PA2_GPIO_Port GPIOA
#define V_SENSE_1_Pin GPIO_PIN_13
#define V_SENSE_1_GPIO_Port GPIOF
#define V_SENSE_2_Pin GPIO_PIN_14
#define V_SENSE_2_GPIO_Port GPIOF
#define GPIO_PB12_Pin GPIO_PIN_12
#define GPIO_PB12_GPIO_Port GPIOB
#define GPIO_PB13_Pin GPIO_PIN_13
#define GPIO_PB13_GPIO_Port GPIOB
#define GPIO_PB14_Pin GPIO_PIN_14
#define GPIO_PB14_GPIO_Port GPIOB
#define GPIO_PB15_Pin GPIO_PIN_15
#define GPIO_PB15_GPIO_Port GPIOB
#define GPIO_PD8_Pin GPIO_PIN_8
#define GPIO_PD8_GPIO_Port GPIOD
#define GPIO_PD9_Pin GPIO_PIN_9
#define GPIO_PD9_GPIO_Port GPIOD
#define GPIO_PD10_Pin GPIO_PIN_10
#define GPIO_PD10_GPIO_Port GPIOD
#define GPIO_PD11_Pin GPIO_PIN_11
#define GPIO_PD11_GPIO_Port GPIOD
#define DETECT_SDIO_Pin GPIO_PIN_15
#define DETECT_SDIO_GPIO_Port GPIOA
#define BAR_DDRY_Pin GPIO_PIN_0
#define BAR_DDRY_GPIO_Port GPIOD
#define BAR_CS_Pin GPIO_PIN_1
#define BAR_CS_GPIO_Port GPIOD
#define IMU_INT2_Pin GPIO_PIN_3
#define IMU_INT2_GPIO_Port GPIOD
#define IMU_INT1_Pin GPIO_PIN_4
#define IMU_INT1_GPIO_Port GPIOD
#define IMU_CS_Pin GPIO_PIN_5
#define IMU_CS_GPIO_Port GPIOD
#define MAG_CS_Pin GPIO_PIN_12
#define MAG_CS_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
// Chu kỳ ngắt TIM9 (500Hz): nhịp chung của vòng vận tốc (pid.c) và đọc IMU (mpu6500.c)
#define CONTROL_DT            0.002f
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
// In qua USB CDC (main.c). Chỉ gọi ở luồng chính, KHÔNG gọi trong ngắt/callback HAL
void CDC_Print(const char *str);
void CDC_WaitHost(void);
// In / nhận qua USART1 (HC-05, main.c). Chỉ gọi ở luồng chính
void UART_Print(const char *str);
void UART_WaitStart(const char *prompt);  // Chờ nhận 1 ký tự bất kỳ, in prompt mỗi 2s
int  UART_ReadLine(char *buf, int size, uint32_t timeout_ms);  // 1 dòng (bỏ dòng trống), -1 = hết giờ
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

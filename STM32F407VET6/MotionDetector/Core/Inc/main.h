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
#define KEY2_Pin GPIO_PIN_2
#define KEY2_GPIO_Port GPIOE
#define KEY3_Pin GPIO_PIN_3
#define KEY3_GPIO_Port GPIOE
#define KEY4_Pin GPIO_PIN_4
#define KEY4_GPIO_Port GPIOE
#define KEY5_Pin GPIO_PIN_5
#define KEY5_GPIO_Port GPIOE
#define KEY6_Pin GPIO_PIN_6
#define KEY6_GPIO_Port GPIOE
#define W25QXX_SCK_Pin GPIO_PIN_5
#define W25QXX_SCK_GPIO_Port GPIOA
#define W25QXX_MISO_Pin GPIO_PIN_6
#define W25QXX_MISO_GPIO_Port GPIOA
#define W25QXX_MOSI_Pin GPIO_PIN_7
#define W25QXX_MOSI_GPIO_Port GPIOA
#define W25QXX_CS_Pin GPIO_PIN_4
#define W25QXX_CS_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOE
#define ESP8266_TX_Pin GPIO_PIN_6
#define ESP8266_TX_GPIO_Port GPIOC
#define ESP8266_RX_Pin GPIO_PIN_7
#define ESP8266_RX_GPIO_Port GPIOC
#define DigitalTube_SER_Pin GPIO_PIN_8
#define DigitalTube_SER_GPIO_Port GPIOC
#define DigitalTube_DISEN_Pin GPIO_PIN_9
#define DigitalTube_DISEN_GPIO_Port GPIOC
#define DigitalTube_DISLK_Pin GPIO_PIN_8
#define DigitalTube_DISLK_GPIO_Port GPIOA
#define DigitalTube_SCK_Pin GPIO_PIN_11
#define DigitalTube_SCK_GPIO_Port GPIOA
#define DigitalTube_A3_Pin GPIO_PIN_12
#define DigitalTube_A3_GPIO_Port GPIOA
#define DigitalTube_A0_Pin GPIO_PIN_15
#define DigitalTube_A0_GPIO_Port GPIOA
#define DigitalTube_A1_Pin GPIO_PIN_10
#define DigitalTube_A1_GPIO_Port GPIOC
#define DigitalTube_A2_Pin GPIO_PIN_11
#define DigitalTube_A2_GPIO_Port GPIOC
#define MPU6050_SCL_Pin GPIO_PIN_6
#define MPU6050_SCL_GPIO_Port GPIOB
#define MPU6050_SDA_Pin GPIO_PIN_7
#define MPU6050_SDA_GPIO_Port GPIOB
#define KEY1_Pin GPIO_PIN_1
#define KEY1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "adc.h"
#include "tim.h"
#include "dac.h"
#include "w25qxx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern bool g_record_end_flag;
extern bool g_play_end_flag;

bool g_play_start_flag = false;
bool g_play_flag = false;
/* USER CODE END Variables */
/* Definitions for MainTask */
osThreadId_t MainTaskHandle;
const osThreadAttr_t MainTask_attributes = {
  .name = "MainTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for KeyTask */
osThreadId_t KeyTaskHandle;
const osThreadAttr_t KeyTask_attributes = {
  .name = "KeyTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartMainTask(void *argument);
void StartKeyTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MainTask */
  MainTaskHandle = osThreadNew(StartMainTask, NULL, &MainTask_attributes);

  /* creation of KeyTask */
  KeyTaskHandle = osThreadNew(StartKeyTask, NULL, &KeyTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMainTask */
/**
  * @brief  Function implementing the MainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void *argument)
{
  /* USER CODE BEGIN StartMainTask */
    W25QXX_Init();
    printf("W25QXX FLASH ID = 0x%04X\r\n", W25QXX_ReadID());
    /* Infinite loop */
    for (;;) {
        if (g_record_end_flag){
            printf("Recording end\n");
            W25QXX_Write((uint8_t *)dma_buff1, 0, sizeof(dma_buff1));
            printf("Recording save ok\n");
            g_record_end_flag = false;
        }
        if (g_play_end_flag){
            printf("Playing end\n");
            g_play_end_flag = false;
            g_play_start_flag = false;
        }
        osDelay(1);
    }
  /* USER CODE END StartMainTask */
}

/* USER CODE BEGIN Header_StartKeyTask */
/**
* @brief Function implementing the KeyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKeyTask */
void StartKeyTask(void *argument)
{
  /* USER CODE BEGIN StartKeyTask */
    uint8_t key_state;
    /* Infinite loop */
    for (;;) {
        //扫描按键
        key_state = 0;
        if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
            osDelay(10);
            if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
                key_state = 1;
            }
            while (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
                osDelay(1);
            }
        } else if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET) {
            osDelay(10);
            if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET) {
                key_state = 2;
            }
            while (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET) {
                osDelay(1);
            }
        } else if (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_RESET) {
            osDelay(10);
            if (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_RESET) {
                key_state = 3;
            }
            while (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_RESET) {
                osDelay(1);
            }
        } else if (HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin) == GPIO_PIN_RESET) {
            osDelay(10);
            if (HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin) == GPIO_PIN_RESET) {
                key_state = 4;
            }
            while (HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin) == GPIO_PIN_RESET) {
                osDelay(1);
            }
        } else if (HAL_GPIO_ReadPin(KEY5_GPIO_Port, KEY5_Pin) == GPIO_PIN_SET) {
            osDelay(10);
            if (HAL_GPIO_ReadPin(KEY5_GPIO_Port, KEY5_Pin) == GPIO_PIN_SET) {
                key_state = 5;
            }
            while (HAL_GPIO_ReadPin(KEY5_GPIO_Port, KEY5_Pin) == GPIO_PIN_SET) {
                osDelay(1);
            }
        } else if (HAL_GPIO_ReadPin(KEY6_GPIO_Port, KEY6_Pin) == GPIO_PIN_SET) {
            osDelay(10);
            if (HAL_GPIO_ReadPin(KEY6_GPIO_Port, KEY6_Pin) == GPIO_PIN_SET) {
                key_state = 6;
            }
            while (HAL_GPIO_ReadPin(KEY6_GPIO_Port, KEY6_Pin) == GPIO_PIN_SET) {
                osDelay(1);
            }
        }
        //按键触发控制
        switch (key_state) {
            case 1:
                W25QXX_Read((uint8_t *)dma_buff1, 0, sizeof(dma_buff1));
                printf("Read recording ok\n");
                break;
            case 2:
                if (g_play_start_flag) {
                    if (g_play_flag){
                        HAL_TIM_Base_Stop(&htim5);
                        g_play_flag = false;
                        printf("Playing pause\n");
                    } else {
                        HAL_TIM_Base_Start(&htim5);
                        g_play_flag = true;
                        printf("Playing continue\n");
                    }
                }
                break;
            case 3:
                if (g_play_start_flag) {
                    HAL_TIM_Base_Stop(&htim5);
                    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
                    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)dma_buff1, MAX_DMA_BUFF_SIZE, DAC_ALIGN_12B_R);
                    HAL_TIM_Base_Start(&htim5);
                    printf("Playing Restart\n");
                    g_play_flag = true;
                }
                break;
            case 4:
                for (int i = 0; i < MAX_DMA_BUFF_SIZE; ++i) {
                    printf("%02X %02X", (uint8_t)(dma_buff1[i] >> 8), (uint8_t)dma_buff1[i]);
                    if (i == MAX_DMA_BUFF_SIZE - 1){
                        printf("\n");
                    } else {
                        printf(" ");
                    }
                }
                break;
            case 5:
                printf("Recording begin\n");
                HAL_ADC_Start_DMA(&hadc1, (uint32_t *)dma_buff1, MAX_DMA_BUFF_SIZE);
                HAL_TIM_Base_Start(&htim2);
                break;
            case 6:
                printf("Playing begin\n");
#if dac_use_tim_it
                HAL_TIM_Base_Start_IT(&htim5);
                HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
#elif dac_use_dma
                HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)dma_buff1, MAX_DMA_BUFF_SIZE, DAC_ALIGN_12B_R);
                HAL_TIM_Base_Start(&htim5);
#endif
                g_play_start_flag = true;
                g_play_flag = true;
                break;
            default:
                break;
        }
        osDelay(1);
    }
  /* USER CODE END StartKeyTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

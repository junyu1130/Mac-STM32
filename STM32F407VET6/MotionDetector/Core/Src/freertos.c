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
#include <string.h>
#include <stdlib.h>
#include "MPU6050.h"
#include "ESP8266.h"
#include "usart.h"
#include "tim.h"
#include "DigitalTube.h"
#include "rtc.h"
#include "fatfs.h"
#include "w25qxx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//WIFI配置
#define WIFI_NAME       "MotionDetector"
#define WIFI_PASSWD     "12345678"
#define UDP_IP          "192.168.4.2"
#define UDP_PORT        9000
//W25QXX FLASH
#define FLASH_BUF_MAX 300
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern uint32_t g_time_count;

uint8_t g_seg_show = 2;//数码管默认显示秒、百分秒
int g_timetxt_count = 0;
bool g_mpu_start = false;
bool g_mpu_save = false;
char g_mpu_data[FLASH_BUF_MAX][100];
int g_mpu_data_index_in = 0;
int g_mpu_data_index_out = 0;
/* USER CODE END Variables */
/* Definitions for MainTask */
osThreadId_t MainTaskHandle;
const osThreadAttr_t MainTask_attributes = {
        .name = "MainTask",
        .priority = (osPriority_t) osPriorityAboveNormal,
        .stack_size = 256 * 4
};
/* Definitions for KeyTask */
osThreadId_t KeyTaskHandle;
const osThreadAttr_t KeyTask_attributes = {
        .name = "KeyTask",
        .priority = (osPriority_t) osPriorityNormal,
        .stack_size = 2048 * 4
};
/* Definitions for DigitaTubeTask */
osThreadId_t DigitaTubeTaskHandle;
const osThreadAttr_t DigitaTubeTask_attributes = {
        .name = "DigitaTubeTask",
        .priority = (osPriority_t) osPriorityAboveNormal,
        .stack_size = 256 * 4
};
/* Definitions for RTCTask */
osThreadId_t RTCTaskHandle;
const osThreadAttr_t RTCTask_attributes = {
        .name = "RTCTask",
        .priority = (osPriority_t) osPriorityNormal,
        .stack_size = 256 * 4
};
/* Definitions for SaveTask */
osThreadId_t SaveTaskHandle;
const osThreadAttr_t SaveTask_attributes = {
        .name = "SaveTask",
        .priority = (osPriority_t) osPriorityNormal,
        .stack_size = 2048 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void SaveTime(void);

void PrintfTime(void);

void SavePara(void);

void LoadPara(void);

void SaveData(void);

void PrintfData(void);

/* USER CODE END FunctionPrototypes */

void StartMainTask(void *argument);

void StartKeyTask(void *argument);

void StartDigitaTubeTask(void *argument);

void StartRTCTask(void *argument);

void StartSaveTask(void *argument);

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

    /* creation of DigitaTubeTask */
    DigitaTubeTaskHandle = osThreadNew(StartDigitaTubeTask, NULL, &DigitaTubeTask_attributes);

    /* creation of RTCTask */
    RTCTaskHandle = osThreadNew(StartRTCTask, NULL, &RTCTask_attributes);

    /* creation of SaveTask */
    SaveTaskHandle = osThreadNew(StartSaveTask, NULL, &SaveTask_attributes);

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
void StartMainTask(void *argument) {
    /* USER CODE BEGIN StartMainTask */
    uint32_t tick = 0;
    char mpu_data[100];

    float SelfTest[6];               // Gyro and accelerometer self-test sensor output
    float aRes, gRes;                // scale resolutions per LSB for the sensors
    int16_t accelCount[3];           // Stores the 16-bit signed accelerometer sensor output
    float ax, ay, az;                // Stores the real accel value in g's
    int16_t gyroCount[3];            // Stores the 16-bit signed gyro sensor output
    float gx, gy, gz;                // Stores the real gyro value in degrees per seconds
    float gyroBias[3], accelBias[3]; // Bias corrections for gyro and accelerometer
    int16_t tempCount;               // Stores the internal chip temperature sensor output
    float temperature;               // Scaled temperature in degrees Celsius

    if (MPU6050_Check() == MPU6050_ADDRESS) {
        MPU6050_SelfTest(SelfTest); // Start by performing self test and reporting values
        printf("x-axis self test: acceleration trim within : ");
        printf("%.1f", SelfTest[0]);
        printf("%% of factory value\r\n");
        printf("y-axis self test: acceleration trim within : ");
        printf("%.1f", SelfTest[1]);
        printf("%% of factory value\r\n");
        printf("z-axis self test: acceleration trim within : ");
        printf("%.1f", SelfTest[2]);
        printf("%% of factory value\r\n");
        printf("x-axis self test: gyration trim within : ");
        printf("%.1f", SelfTest[3]);
        printf("%% of factory value\r\n");
        printf("y-axis self test: gyration trim within : ");
        printf("%.1f", SelfTest[4]);
        printf("%% of factory value\r\n");
        printf("z-axis self test: gyration trim within : ");
        printf("%.1f", SelfTest[5]);
        printf("%% of factory value\r\n");

        if (SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f &&
            SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
            printf("Pass Selftest!\r\n");

            MPU6050_calibrate(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
            MPU6050_init();
            printf("MPU6050 initialized for active data mode....\r\n"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
        } else {
            printf("Could not connect to MPU6050\r\n");
        }
    }

    HAL_UART_Receive_IT(&huart6, &rx_byte, 1);

    if (ESP8266_Init()) {
        printf("ESP8266 Init OK!\r\n");
    } else {
        printf("ESP8266 Init FAIL!\r\n");
    }

    if (ESP8266_AP(WIFI_NAME, WIFI_PASSWD)) {
        printf("ESP8266 Create AP OK!\r\n");
    } else {
        printf("ESP8266 Create AP FAIL!\r\n");
    }
//    if (ESP8266_UDP_Connect(UDP_IP,UDP_PORT)) {
//        printf("ESP8266 Create UDP OK!\r\n");
//    } else {
//        printf("ESP8266 Create UDP FAIL!\r\n");
//    }

    /* Infinite loop */
    for (;;) {
        if (g_mpu_start == true) {
            if (MPU6050_readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {  // check if data ready interrupt
                MPU6050_readAccelData(accelCount);  // Read the x/y/z adc values
                aRes = MPU6050_getAres();

                // Now we'll calculate the accleration value into actual g's
                ax = (float) accelCount[0] * aRes -
                     accelBias[0];  // get actual g value, this depends on scale being set
                ay = (float) accelCount[1] * aRes - accelBias[1];
                az = (float) accelCount[2] * aRes - accelBias[2];

                MPU6050_readGyroData(gyroCount);  // Read the x/y/z adc values
                gRes = MPU6050_getGres();

                // Calculate the gyro value into actual degrees per second
                gx = (float) gyroCount[0] * gRes -
                     gyroBias[0];  // get actual gyro value, this depends on scale being set
                gy = (float) gyroCount[1] * gRes - gyroBias[1];
                gz = (float) gyroCount[2] * gRes - gyroBias[2];

                tempCount = MPU6050_readTempData();  // Read the x/y/z adc values
                temperature = ((float) tempCount) / 340. + 36.53; // Temperature in degrees Centigrade

                uint8_t test_min = (g_time_count / 100) / 60;
                uint8_t test_sec = (g_time_count / 100) % 60;
                uint8_t test_sec_100 = g_time_count % 100;
                sprintf(mpu_data,
                        "ax:%.1f,ay:%.1f,az:%.1f mg gx:%.1f,gy:%.1f,gz:%.1f deg/s (%02d:%02d:%02d)\r\n",
                        1000 * ax, 1000 * ay, 1000 * az, gx, gy, gz, test_min, test_sec, test_sec_100);
                strcpy(g_mpu_data[g_mpu_data_index_in], mpu_data);
                if (g_mpu_data_index_in < FLASH_BUF_MAX - 1) {
                    g_mpu_data_index_in++;
                } else {
                    g_mpu_data_index_in = 0;
                }
                if (g_mpu_save == false) {
                    f_unlink("0:/data.txt");
                    g_mpu_save = true;
                }
            }

            if (osKernelGetTickCount() - tick >= 500) {//500ms
                // Print acceleration values in milligs!
                printf("X-acceleration: ");
                printf("%.1f", 1000 * ax);
                printf(" mg ");
                printf("Y-acceleration: ");
                printf("%.1f", 1000 * ay);
                printf(" mg ");
                printf("Z-acceleration: ");
                printf("%.1f", 1000 * az);
                printf(" mg\r\n");

                // Print gyro values in degree/sec
                printf("X-gyro rate: ");
                printf("%.1f", gx);
                printf(" degrees/sec ");
                printf("Y-gyro rate: ");
                printf("%.1f", gy);
                printf(" degrees/sec ");
                printf("Z-gyro rate: ");
                printf("%.1f", gz);
                printf(" degrees/sec\r\n");

                // Print temperature in degrees Centigrade
                printf("Temperature is ");
                printf("%.2f", temperature);
                printf(" degrees C\r\n"); // Print T values to tenths of s degree C
                printf("\r\n");

                tick = osKernelGetTickCount();
            }
        }
        osDelay(50);
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
void StartKeyTask(void *argument) {
    /* USER CODE BEGIN StartKeyTask */
    uint8_t key_state;
    W25QXX_Init();
    printf("W25QXX FLASH ID = 0x%04X\r\n", W25QXX_ReadID());
    LoadPara();
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
            case 1://读取测试时间记录
                PrintfTime();
                break;
            case 2:
                g_seg_show = 1;//数码管显示分、秒
                break;
            case 3:
                g_seg_show = 2;//数码管显示秒、百分秒
                break;
            case 4:
                PrintfData();
                break;
            case 5://清零并开始计时
                g_time_count = 0;
                HAL_TIM_Base_Start_IT(&htim3);
                g_mpu_start = true;
                break;
            case 6://停止计时并记录
                HAL_TIM_Base_Stop_IT(&htim3);
                g_mpu_start = false;
                if (g_timetxt_count >= 100) {//大于100个数据时覆盖文件
                    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
                    f_unlink("0:/time.txt");
                    printf("The time record data has exceeded 100. This file has been overwritten for you.\r\n");
                    g_timetxt_count = 0;
                    osDelay(1000);
                    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
                } else {
                    SaveTime();
                    g_timetxt_count++;
                    SavePara();
                    printf("Save time OK! Now time.txt have %d data.\r\n", g_timetxt_count);
                }
                break;
            default:
                break;
        }
        osDelay(1);
    }
    /* USER CODE END StartKeyTask */
}

/* USER CODE BEGIN Header_StartDigitaTubeTask */
/**
* @brief Function implementing the DigitaTubeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDigitaTubeTask */
void StartDigitaTubeTask(void *argument) {
    /* USER CODE BEGIN StartDigitaTubeTask */
    uint8_t seg[4] = {0};
    /* Infinite loop */
    for (;;) {
        if (g_seg_show == 1) {//分、秒
            seg[0] = ((g_time_count / 100) / 60) / 10;
            seg[1] = ((g_time_count / 100) / 60) % 10;
            seg[2] = ((g_time_count / 100) % 60) / 10;
            seg[3] = ((g_time_count / 100) % 60) % 10;
        } else if (g_seg_show == 2) {//秒、百分秒
            seg[0] = ((g_time_count / 100) % 60) / 10;
            seg[1] = ((g_time_count / 100) % 60) % 10;
            seg[2] = (g_time_count % 100) / 10;
            seg[3] = g_time_count % 10;
        }
        for (int i = 0; i < 4; ++i) {
            if (i == 1 && osKernelGetTickCount() % 1000 < 500) {
                Write595(i, seg[i], 1);
            } else {
                Write595(i, seg[i], 0);
            }
            osDelay(5);
        }
    }
    /* USER CODE END StartDigitaTubeTask */
}

/* USER CODE BEGIN Header_StartRTCTask */
/**
* @brief Function implementing the RTCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRTCTask */
void StartRTCTask(void *argument) {
    /* USER CODE BEGIN StartRTCTask */
    uint32_t tick = 0;
    /* Infinite loop */
    for (;;) {
        if (osKernelGetTickCount() - tick >= 1000) {
            tick = osKernelGetTickCount();
            ReadRTCDateTime();
        }
        osDelay(1);
    }
    /* USER CODE END StartRTCTask */
}

/* USER CODE BEGIN Header_StartSaveTask */
/**
* @brief Function implementing the SaveTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSaveTask */
void StartSaveTask(void *argument) {
    /* USER CODE BEGIN StartSaveTask */
    /* Infinite loop */
    for (;;) {
        if (g_mpu_save == true) {
            SaveData();
            if (g_mpu_start == false && g_mpu_data_index_in == g_mpu_data_index_out) {
                g_mpu_save = false;
                printf("Save MPU Data OK!\r\n");
            }
        }
        osDelay(1);
    }
    /* USER CODE END StartSaveTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void SaveTime(void) {
    FIL fil;//文件对象
    FRESULT res;//操作返回结果
    char buf[100];

    res = f_open(&fil, "0:/time.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_OPEN_APPEND);
    if (res != FR_OK) {
        printf("open file error.\r\n");
    } else {
        uint8_t test_min = (g_time_count / 100) / 60;
        uint8_t test_sec = (g_time_count / 100) % 60;
        uint8_t test_sec_100 = g_time_count % 100;
        sprintf(buf, "Test Time:%02d:%02d:%02d (%4d.%2d.%2d %02d:%02d:%02d)\r\n", test_min, test_sec, test_sec_100,
                RTC_Year, RTC_Mon, RTC_Dat, RTC_Hour,
                RTC_Min, RTC_Sec);
        f_puts(buf, &fil);
        f_close(&fil);
    }
}

void PrintfTime(void) {
    FIL fil;//文件对象
    FRESULT res;//操作返回结果
    char buf[100];

    res = f_open(&fil, "0:/time.txt", FA_READ | FA_OPEN_ALWAYS);
    if (res != FR_OK) {
        printf("open file error.\r\n");
    } else {
        f_lseek(&fil, 0);
        while (f_gets(buf, 100, &fil)) {
            printf("%s", buf);
        }
        f_close(&fil);
    }
}

void SavePara(void) {
    FIL fil;//文件对象
    FRESULT res;//操作返回结果

    res = f_open(&fil, "0:/para.txt", FA_WRITE | FA_OPEN_ALWAYS);
    if (res != FR_OK) {
        printf("open file error.\r\n");
    } else {
        f_lseek(&fil, 0);
        f_printf(&fil, "time.txt count:%d\r\n", g_timetxt_count);
        f_close(&fil);
    }
}

void LoadPara(void) {
    FIL fil;//文件对象
    FRESULT res;//操作返回结果
    char buf[50];

    res = f_open(&fil, "0:/para.txt", FA_READ | FA_OPEN_ALWAYS);
    if (res != FR_OK) {
        printf("open file error.\r\n");
    } else {
        f_lseek(&fil, 0);
        while (f_gets(buf, 50, &fil)) {
            if (strstr(buf, "time.txt count:") != NULL) {
                g_timetxt_count = atoi(buf + 15);
            } else {
                printf("load error!\r\n");
            }
        }
        f_close(&fil);
    }
}

void SaveData(void) {
    FIL fil;//文件对象
    FRESULT res;//操作返回结果

    res = f_open(&fil, "0:/data.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_OPEN_APPEND);
    if (res != FR_OK) {
        printf("open file error.\r\n");
    } else {
        f_puts(g_mpu_data[g_mpu_data_index_out], &fil);
        if (g_mpu_data_index_out < FLASH_BUF_MAX - 1) {
            g_mpu_data_index_out++;
        } else {
            g_mpu_data_index_out = 0;
        }
        f_close(&fil);
    }
}

void PrintfData(void) {
    FIL fil;//文件对象
    FRESULT res;//操作返回结果
    char buf[100];

    res = f_open(&fil, "0:/data.txt", FA_READ | FA_OPEN_ALWAYS);
    if (res != FR_OK) {
        printf("open file error.\r\n");
    } else {
        f_lseek(&fil, 0);
        while (f_gets(buf, 100, &fil)) {
            printf("%s", buf);
        }
        f_close(&fil);
    }
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

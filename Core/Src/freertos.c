/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "nrf24.h"
#include "spi.h"
#include "stm32f1xx.h"
#include "tim.h"
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint16_t mapVal(uint16_t sourceMin, uint16_t sourceMax, uint16_t targetMin,
                uint16_t targetMax, uint16_t value) {
  uint16_t mappedValue = targetMin + (value - sourceMin) *
                                         (targetMax - targetMin) /
                                         (sourceMax - sourceMin);
  return mappedValue;
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for nrf24Task */
osThreadId_t nrf24TaskHandle;
const osThreadAttr_t nrf24Task_attributes = {
    .name = "nrf24Task",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartNrf24Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  // 电机舵机pwm初始化
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

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
  /* creation of defaultTask */
  defaultTaskHandle =
      osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of nrf24Task */
  nrf24TaskHandle = osThreadNew(StartNrf24Task, NULL, &nrf24Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  (void)argument;
  // int pwmValue = 0;
  for (;;) {
    osDelay(100);
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwmValue);
    // pwmValue += 1;
    // if(pwmValue > 100) pwmValue = 0;
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartNrf24Task */
/**
 * @brief Function implementing the nrf24Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartNrf24Task */
void StartNrf24Task(void *argument) {
  /* USER CODE BEGIN StartNrf24Task */
  /* Infinite loop */
  (void)argument;
  // NRF24初始化
  NRF24_Init(&hspi1);
  // 检测NRF24L01是否存在
  if (NRF24_Check() != 1) {
    // 未检测到NRF24L01，可以在这里添加错误处理
    while (1)
      ;
  }
  // 设置为接收模式
  NRF24_SetRxMode(nrf24_addr);
  /**
x轴 舵机值 范围 50-150 对应 90度范围 ； 接收侧pwm分辨率2000
y轴上 范围0-100； 接收侧收到数据x10，因为pwm分辨率是1000
y轴下 范围0-100； 接收侧收到数据x10，因为pwm分辨率是1000
*/
  uint8_t rx_data[10];
  uint8_t rx_len = 0;

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);

  uint8_t carType = 1; // 0-翻斗车 1-装载机 2-挖掘机

  uint32_t linking_timestamp = 0;
  for (;;) {
    rx_len = NRF24_ReceiveData(rx_data, 10);
    if (rx_len > 0) {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

      linking_timestamp = HAL_GetTick(); // 记录接收数据的时间戳
      // 接收到数据，通过串口发送出去

      /**
      翻斗车
      */
      if (carType == 0) {
        // 进退控制
        if (rx_data[1] < 100) {
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
                                (100 - rx_data[1])); // x轴舵机控制
        } else if (rx_data[1] > 100) {
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (rx_data[1] - 100));
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0); // x轴舵机控制
        } else {

          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0); // x轴舵机控制
        }

        // 方向舵机
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,
                              mapVal(0, 200, 75, 125, rx_data[2]));

        // 翻斗起落
        if (rx_data[6] < 100) {
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,
                                (100 - rx_data[6])); // y轴舵机控制
        } else if (rx_data[6] > 100) {
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (rx_data[6] - 100));
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0); // y轴舵机控制
        } else {

          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0); // y轴舵机控制
        }
      }

      /*
      装载机
      */
      if (carType == 1) {
        // 进退控制
        if (rx_data[1] < 100) {
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
                                (100 - rx_data[1])); // x轴舵机控制
        } else if (rx_data[1] > 100) {
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (rx_data[1] - 100));
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0); // x轴舵机控制
        } else {

          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0); // x轴舵机控制
        }

        // 方向舵机
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,
                              200 - mapVal(0, 200, 75, 125, rx_data[2]));

        // 大臂起落
        if (rx_data[4] < 100) {
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,
                                (100 - rx_data[4])); // y轴舵机控制
        } else if (rx_data[4] > 100) {
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (rx_data[4] - 100));
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0); // y轴舵机控制
        } else {

          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
          __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0); // y轴舵机控制
        }
        // 挖斗起落
        if (rx_data[5] < 100) {
          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,
                                (100 - rx_data[5])); // y轴舵机控制
        } else if (rx_data[5] > 100) {
          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (rx_data[5] - 100));
          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0); // y轴舵机控制
        } else {

          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0); // y轴舵机控制
        }
      }
      /**
      挖掘机
      */
      if (carType == 2) {
      }

      // 清空接收缓冲区
      memset(rx_data, 0, 4);
    } else {
      if (HAL_GetTick() - linking_timestamp > 1000) {
        // 重置舵机
        // 超过1秒没有接收到数据，关闭LED指示灯
        // 重置电机
      }
    }
    osDelay(1);
  }
  /* USER CODE END StartNrf24Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

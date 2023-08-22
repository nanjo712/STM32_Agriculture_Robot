/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#ifdef __cplusplus
extern "C"{
#endif
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#ifdef __cplusplus
}
#endif
#define LOG_TAG    "defTask"
#ifdef __cplusplus
extern "C"{
#endif

#include "oslib.h"
#include <stdint.h>
#include "MotorLib/dji_boardv2_can.h"

#ifdef __cplusplus
}
#endif

#include <cstring>
#include "chassis.h"

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
chassis myChassis;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for chassisControl */
osThreadId_t chassisControlHandle;
const osThreadAttr_t chassisControl_attributes = {
  .name = "chassisControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for balanceMotorTas */
osThreadId_t balanceMotorTasHandle;
const osThreadAttr_t balanceMotorTas_attributes = {
  .name = "balanceMotorTas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for waterVase */
osThreadId_t waterVaseHandle;
const osThreadAttr_t waterVase_attributes = {
  .name = "waterVase",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for WaterSema */
osSemaphoreId_t WaterSemaHandle;
const osSemaphoreAttr_t WaterSema_attributes = {
  .name = "WaterSema"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
#ifdef __cplusplus
extern "C"{
#endif
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartChassisControl(void *argument);
void StartBalance(void *argument);
void StartWaterVase(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
    OSLIB_Init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of WaterSema */
  WaterSemaHandle = osSemaphoreNew(3, 3, &WaterSema_attributes);

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
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of chassisControl */
  chassisControlHandle = osThreadNew(StartChassisControl, NULL, &chassisControl_attributes);

  /* creation of balanceMotorTas */
  balanceMotorTasHandle = osThreadNew(StartBalance, NULL, &balanceMotorTas_attributes);

  /* creation of waterVase */
  waterVaseHandle = osThreadNew(StartWaterVase, NULL, &waterVase_attributes);

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
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
      HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
      osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartChassisControl */
/**
* @brief Function implementing the chassisControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartChassisControl */
void StartChassisControl(void *argument)
{
  /* USER CODE BEGIN StartChassisControl */
  osDelay(1000);
  myChassis.chassis_init(&hcan2);
  OSLIB_UART_Handle_t *uart_handle= OSLIB_UART_Handle_Get(&huart2);
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreAcquire(uart_handle->rx.dma.rx_sema,osWaitForever);
    SerialVelMsgTypeDef msg;
    msg.raw_msg[0]=0,msg.raw_msg[5]=0;
    memcpy(msg.ui8,uart_handle->rx.dma.rx_task_buffer,uart_handle->rx.dma.rx_buffer_len);
    if (msg.raw_msg[1]!=0)
    {
        for (int i=0;i<msg.raw_msg[1];i++)
        {
            osSemaphoreRelease(WaterSemaHandle);
            osDelay(1);
        }
    }
    myChassis.chassis_move(&hcan2,msg);
    osDelay(1);
  }
  /* USER CODE END StartChassisControl */
}

/* USER CODE BEGIN Header_StartBalance */
/**
* @brief Function implementing the balanceMotorTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBalance */
void StartBalance(void *argument)
{
  /* USER CODE BEGIN StartBalance */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartBalance */
}

/* USER CODE BEGIN Header_StartWaterVase */
/**
* @brief Function implementing the waterVase thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWaterVase */
void StartWaterVase(void *argument)
{
  /* USER CODE BEGIN StartWaterVase */
  /* Infinite loop */
  for(;;)
  {
      osSemaphoreAcquire(WaterSemaHandle,osWaitForever);
      HAL_GPIO_WritePin(IN1_GPIO_Port,IN1_Pin,GPIO_PIN_SET);
      HAL_GPIO_WritePin(IN2_GPIO_Port,IN2_Pin,GPIO_PIN_SET);
      osDelay(500);
      HAL_GPIO_WritePin(IN1_GPIO_Port,IN1_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(IN2_GPIO_Port,IN2_Pin,GPIO_PIN_RESET);
      osDelay(350);
  }
  /* USER CODE END StartWaterVase */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
#ifdef __cplusplus
}
#endif
/* USER CODE END Application */


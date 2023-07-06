/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "ins_task.h"
#include "detect_task.h"
#include "bsp_ros.h"
#include "bsp_chassis.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"
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

/* USER CODE END Variables */
osThreadId INSTaskHandle;
osThreadId ROSTaskHandle;
osThreadId CANTaskHandle;
osThreadId DetectTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartINSTask(void const * argument);
void StartROSTask(void const * argument);
void StartCANTask(void const * argument);
void StartDetectTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

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
  /* definition and creation of INSTask */
  osThreadDef(INSTask, StartINSTask, osPriorityNormal, 0, 1024);
  INSTaskHandle = osThreadCreate(osThread(INSTask), NULL);

  /* definition and creation of ROSTask */
  osThreadDef(ROSTask, StartROSTask, osPriorityNormal, 0, 1024);
  ROSTaskHandle = osThreadCreate(osThread(ROSTask), NULL);

  /* definition and creation of CANTask */
  osThreadDef(CANTask, StartCANTask, osPriorityNormal, 0, 1024);
  CANTaskHandle = osThreadCreate(osThread(CANTask), NULL);

  /* definition and creation of DetectTask */
  osThreadDef(DetectTask, StartDetectTask, osPriorityIdle, 0, 128);
  DetectTaskHandle = osThreadCreate(osThread(DetectTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartINSTask */
/**
 * @brief Function implementing the INSTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartINSTask */
void StartINSTask(void const * argument)
{
  /* USER CODE BEGIN StartINSTask */
    INS_Init();
    /* Infinite loop */
    for (;;)
    {
        INS_Task();
			//printf("%.02f,%.02f,%.02f\n", INS.Pitch, INS.Roll, INS.Yaw);
        osDelay(1);
    }
  /* USER CODE END StartINSTask */
}

/* USER CODE BEGIN Header_StartROSTask */
/**
* @brief Function implementing the ROSTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartROSTask */
void StartROSTask(void const * argument)
{
  /* USER CODE BEGIN StartROSTask */
	
  Message_Init();
  /* Infinite loop */
  for(;;)
  {
    Ros_Upload();
    Ros_Transmit();
		//printf("%.2f,%.2f\n",get_chassisMove_point()->wz_set, get_chassisMove_point()->error_angle);
    osDelay(1);
  }
  /* USER CODE END StartROSTask */
}

/* USER CODE BEGIN Header_StartCANTask */
/**
* @brief Function implementing the CANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCANTask */
void StartCANTask(void const * argument)
{
  /* USER CODE BEGIN StartCANTask */
  while (toe_is_error(ROSTOE))
  {
    vTaskDelay(2);
  }
  Chassis_init();
  /* Infinite loop */
  for(;;)
  {
		chassis_ctrl();
		
    osDelay(1);
  }
  /* USER CODE END StartCANTask */
}

/* USER CODE BEGIN Header_StartDetectTask */
/**
* @brief Function implementing the DetectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDetectTask */
void StartDetectTask(void const * argument)
{
  /* USER CODE BEGIN StartDetectTask */
  static uint32_t systemTime;

  systemTime = xTaskGetTickCount();

  // ��ʼ��
  DetectInit(systemTime);
  /* Infinite loop */
  for(;;)
  {
    DetectTask();
  }
  /* USER CODE END StartDetectTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

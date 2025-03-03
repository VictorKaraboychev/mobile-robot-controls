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
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for encodersTask */
osThreadId_t encodersTaskHandle;
const osThreadAttr_t encodersTask_attributes = {
    .name = "encodersTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for fusionTask */
osThreadId_t fusionTaskHandle;
const osThreadAttr_t fusionTask_attributes = {
    .name = "fusionTask",
    .stack_size = 8192 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for debugTask */
osThreadId_t debugTaskHandle;
const osThreadAttr_t debugTask_attributes = {
    .name = "debugTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for controlTask */
osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
    .name = "controlTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
/* Definitions for barometerTask */
osThreadId_t barometerTaskHandle;
const osThreadAttr_t barometerTask_attributes = {
    .name = "barometerTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
    .name = "imuTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for magTask */
osThreadId_t magTaskHandle;
const osThreadAttr_t magTask_attributes = {
    .name = "magTask",
    .stack_size = 2048 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for spi1Mutex */
osMutexId_t spi1MutexHandle;
const osMutexAttr_t spi1Mutex_attributes = {
    .name = "spi1Mutex"};
/* Definitions for spi4Mutex */
osMutexId_t spi4MutexHandle;
const osMutexAttr_t spi4Mutex_attributes = {
    .name = "spi4Mutex"};
/* Definitions for i2c1Mutex */
osMutexId_t i2c1MutexHandle;
const osMutexAttr_t i2c1Mutex_attributes = {
    .name = "i2c1Mutex"};
/* Definitions for usbMutex */
osMutexId_t usbMutexHandle;
const osMutexAttr_t usbMutex_attributes = {
    .name = "usbMutex"};
/* Definitions for fdcanMutex */
osMutexId_t fdcanMutexHandle;
const osMutexAttr_t fdcanMutex_attributes = {
    .name = "fdcanMutex"};
/* Definitions for uart4Mutex */
osMutexId_t uart4MutexHandle;
const osMutexAttr_t uart4Mutex_attributes = {
    .name = "uart4Mutex"};
/* Definitions for uart7Mutex */
osMutexId_t uart7MutexHandle;
const osMutexAttr_t uart7Mutex_attributes = {
    .name = "uart7Mutex"};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void StartEncodersTask(void *argument);
extern void StartFusionTask(void *argument);
extern void StartDebugTask(void *argument);
extern void StartControlTask(void *argument);
extern void StartBarometerTask(void *argument);
extern void StartIMUTask(void *argument);
extern void StartMagTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
  /* Run time stack overflow checking is performed if
  configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
  called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of spi1Mutex */
  spi1MutexHandle = osMutexNew(&spi1Mutex_attributes);

  /* creation of spi4Mutex */
  spi4MutexHandle = osMutexNew(&spi4Mutex_attributes);

  /* creation of i2c1Mutex */
  i2c1MutexHandle = osMutexNew(&i2c1Mutex_attributes);

  /* creation of usbMutex */
  usbMutexHandle = osMutexNew(&usbMutex_attributes);

  /* creation of fdcanMutex */
  fdcanMutexHandle = osMutexNew(&fdcanMutex_attributes);

  /* creation of uart4Mutex */
  uart4MutexHandle = osMutexNew(&uart4Mutex_attributes);

  /* creation of uart7Mutex */
  uart7MutexHandle = osMutexNew(&uart7Mutex_attributes);

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
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of encodersTask */
  encodersTaskHandle = osThreadNew(StartEncodersTask, NULL, &encodersTask_attributes);

  /* creation of fusionTask */
  fusionTaskHandle = osThreadNew(StartFusionTask, NULL, &fusionTask_attributes);

  /* creation of debugTask */
  debugTaskHandle = osThreadNew(StartDebugTask, NULL, &debugTask_attributes);

  /* creation of controlTask */
  controlTaskHandle = osThreadNew(StartControlTask, NULL, &controlTask_attributes);

  /* creation of barometerTask */
  barometerTaskHandle = osThreadNew(StartBarometerTask, NULL, &barometerTask_attributes);

  /* creation of imuTask */
  imuTaskHandle = osThreadNew(StartIMUTask, NULL, &imuTask_attributes);

  /* creation of magTask */
  magTaskHandle = osThreadNew(StartMagTask, NULL, &magTask_attributes);

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
  /* init code for USB_DEVICE */
  // MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */

  /* Infinite loop */
  while (1)
  {
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

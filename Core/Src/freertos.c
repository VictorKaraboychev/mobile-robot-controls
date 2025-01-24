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

#include "tim.h"
#include "spi.h"
#include "i2c.h"
#include <stdio.h>
#include <lis2mdl.h>
#include <lsm6dso.h>

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
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for encodersTask */
osThreadId_t encodersTaskHandle;
const osThreadAttr_t encodersTask_attributes = {
    .name = "encodersTask",
    .stack_size = 512 * 4,
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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void StartImuTask(void *argument);
extern void StartMagTask(void *argument);
extern void StartEncodersTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* creation of imuTask */
  imuTaskHandle = osThreadNew(StartImuTask, NULL, &imuTask_attributes);

  /* creation of magTask */
  magTaskHandle = osThreadNew(StartMagTask, NULL, &magTask_attributes);

  /* creation of encodersTask */
  encodersTaskHandle = osThreadNew(StartEncodersTask, NULL, &encodersTask_attributes);

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

  // // GREEN LEDs
  // htim4.Instance->CCR1 = 100;
  // htim4.Instance->CCR2 = 100;
  // htim4.Instance->CCR3 = 100;

  // // RED LEDs
  // htim4.Instance->CCR4 = 100;
  // htim8.Instance->CCR1 = 100;
  // htim8.Instance->CCR2 = 100;

  // GREEN LEDs
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

  // RED LEDs
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

  // BUZZER
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);

  // htim15.Instance->CCR1 = 500;

  uint16_t pwm = 0;

  /* Infinite loop */
  for (;;)
  {
    // printf("Hello World!\n");

    // GREEN LEDs
    htim4.Instance->CCR4 = pwm % 300;
    htim8.Instance->CCR1 = (pwm + 100) % 300;
    htim8.Instance->CCR2 = (pwm + 200) % 300;

    pwm += 2;
    pwm %= 300;

    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// LSM6DSO

LSM6DSO_Object_t lsm6dso;

int32_t Write_LSM6DSO(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  return SPI_Write_Register(&hspi1, &spi1MutexHandle, IMU_CS_GPIO_Port, IMU_CS_Pin, reg, data, len);
}

int32_t Read_LSM6DSO(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  return SPI_Read_Register(&hspi1, &spi1MutexHandle, IMU_CS_GPIO_Port, IMU_CS_Pin, reg, data, len);
}

void StartImuTask(void *argument)
{
  lsm6dso.Ctx.handle = &hspi1;
  lsm6dso.Ctx.write_reg = Write_LSM6DSO;
  lsm6dso.Ctx.read_reg = Read_LSM6DSO;
  lsm6dso.Ctx.mdelay = HAL_Delay;

  lsm6dso.IO.BusType = LSM6DSO_SPI_4WIRES_BUS;

  uint8_t id = 0, rst = 0;

  // Reset the IMU
  lsm6dso_reset_set(&lsm6dso.Ctx, PROPERTY_ENABLE);
  do
  {
    lsm6dso_reset_get(&lsm6dso.Ctx, &rst);
  } while (rst);

  // Read the IMU ID
  do
  {
    LSM6DSO_ReadID(&lsm6dso, &id);
    osDelay(100);
  } while (id != LSM6DSO_ID);

  // Initialize the IMU
  LSM6DSO_Init(&lsm6dso);

  // Accelerometer configuration
  LSM6DSO_ACC_SetOutputDataRate_With_Mode(&lsm6dso, 6667.0f, LSM6DSO_ACC_HIGH_PERFORMANCE_MODE);
  LSM6DSO_ACC_SetFullScale(&lsm6dso, 4);
  LSM6DSO_ACC_Set_Filter_Mode(&lsm6dso, 0, LSM6DSO_LP_ODR_DIV_20);
  LSM6DSO_ACC_Enable(&lsm6dso);

  // Gyroscope configuration
  LSM6DSO_GYRO_SetOutputDataRate_With_Mode(&lsm6dso, 6667.0f, LSM6DSO_GYRO_HIGH_PERFORMANCE_MODE);
  LSM6DSO_GYRO_SetFullScale(&lsm6dso, 500);
  LSM6DSO_GYRO_Set_Filter_Mode(&lsm6dso, 0, LSM6DSO_LP_ODR_DIV_10);
  LSM6DSO_GYRO_Enable(&lsm6dso);

  osDelay(10);

  // Accelerometer and gyroscope scale factors
  float acc_scale = 9.81 / 1000.0f;
  float gyro_scale = (3.14159265 / 180.0f) / 1000.0f;

  // LSM6DSO_Axes_t acc, gyro;
  // bool status = false;

  // // Calibrate the IMU
  // uint16_t samples = 1000;
  // Vector acc_bias(3), gyro_bias(3);

  // for (uint16_t i = 0; i < samples; i++)
  // {
  // 	LSM6DSO_ACC_GetAxes(&lsm6dso, &acc);
  // 	LSM6DSO_GYRO_GetAxes(&lsm6dso, &gyro);

  // 	acc_bias += -Vector{(float)acc.x, (float)acc.y, (float)acc.z};
  // 	gyro_bias += Vector{(float)gyro.x, (float)gyro.y, (float)gyro.z};

  // 	osDelay(2);
  // }

  // acc_bias *= (acc_scale / (float)samples);
  // gyro_bias *= (gyro_scale / (float)samples);

  // *acc_bias.z += GRAVITY; // Subtract gravity from the z-axis

  // statistics_t stats;

  LSM6DSO_Axes_t acc, gyro;

  while (1)
  {
    // printf("LSM6DSO ID: 0x%02X\n", id);

    // LSM6DSO_ACC_GetAxes(&lsm6dso, &acc);
    // LSM6DSO_GYRO_GetAxes(&lsm6dso, &gyro);

    // printf("Acceleration: %.2f %.2f %.2f Gyroscope: %.4f %.4f %.4f\n",
    //        (float)(acc.x * acc_scale),
    //        (float)(acc.y * acc_scale),
    //        (float)(acc.z * acc_scale),
    //        (float)(gyro.x * gyro_scale),
    //        (float)(gyro.y * gyro_scale),
    //        (float)(gyro.z * gyro_scale));

    // status = (LSM6DSO_ACC_GetAxes(&lsm6dso, &acc) == LSM6DSO_OK) && (LSM6DSO_GYRO_GetAxes(&lsm6dso, &gyro) == LSM6DSO_OK);

    // // Check if the IMU is active
    // if (!status || id != LSM6DSO_ID)
    // {
    // 	imu_data.active = false;

    // 	osDelay(100);
    // 	continue;
    // }

    // // Map the IMU data to the IMU data structure
    // imu_data.acceleration = (-Vector{(float)acc.x, (float)acc.y, (float)acc.z} * acc_scale - acc_bias);
    // imu_data.angular_velocity = (Vector{(float)gyro.x, (float)gyro.y, (float)gyro.z} * gyro_scale - gyro_bias);

    // imu_data.active = true;

    // ComputeStatisticsRecursive(&stats, 1000, *imu_data.angular_velocity.z);

    // Print biases
    // printf("ID: 0x%02X Acceleration: %.4f %.4f %.4f Gyroscope: %.4f %.4f %.4f\n", id, *acc_bias.x, *acc_bias.y, *acc_bias.z, *gyro_bias.x, *gyro_bias.y, *gyro_bias.z);

    // printf("ID: 0x%02X Acceleration: %.2f %.2f %.2f Gyroscope: %.4f %.4f %.4f\n", id, *imu_data.acceleration.x, *imu_data.acceleration.y, *imu_data.acceleration.z, *imu_data.angular_velocity.x, *imu_data.angular_velocity.y, *imu_data.angular_velocity.z);

    osDelay(100);
  }
}

// LIS2MDL

LIS2MDL_Object_t lis2mdl;

int32_t Write_LIS2MDL(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  return SPI_Write_Register(&hspi1, &spi1MutexHandle, MAG_CS_GPIO_Port, MAG_CS_Pin, reg, data, len);
}

int32_t Read_LIS2MDL(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
  return SPI_Read_Register(&hspi1, &spi1MutexHandle, MAG_CS_GPIO_Port, MAG_CS_Pin, reg, data, len);
}

void StartMagTask(void *argument)
{
  lis2mdl.Ctx.handle = &hspi1;
  lis2mdl.Ctx.write_reg = Write_LIS2MDL;
  lis2mdl.Ctx.read_reg = Read_LIS2MDL;
  lis2mdl.Ctx.mdelay = HAL_Delay;

  lis2mdl.IO.BusType = LIS2MDL_SPI_4WIRES_BUS;

  uint8_t id = 0, rst = 0;

  // Reset the magnetometer
  lis2mdl_reset_set(&lis2mdl.Ctx, PROPERTY_ENABLE);
  do
  {
    lis2mdl_reset_get(&lis2mdl.Ctx, &rst);
  } while (rst);

  // Set the SPI mode to 4-wire
  lis2mdl_spi_mode_set(&lis2mdl.Ctx, LIS2MDL_SPI_4_WIRE);

  // Read the magnetometer ID
  do
  {
    LIS2MDL_ReadID(&lis2mdl, &id);
    osDelay(100);
  } while (id != LIS2MDL_ID);

  // Initialize the magnetometer
  LIS2MDL_Init(&lis2mdl);

  // Magnetometer configuration
  LIS2MDL_MAG_SetOutputDataRate(&lis2mdl, 100.0f);
  LIS2MDL_MAG_SetFullScale(&lis2mdl, 50);
  LIS2MDL_MAG_Set_Power_Mode(&lis2mdl, LIS2MDL_HIGH_RESOLUTION);
  LIS2MDL_MAG_Enable(&lis2mdl);

  osDelay(10);

  // Magnetometer scale factor
  float mag_scale = 1.5f / 1000.0f;
  // float RAD_TO_DEG = 180.0f / 3.14159265f;

  LIS2MDL_Axes_t mag;

  while (1)
  {
    // printf("LIS2MDL ID: 0x%02X\n", id);

    LIS2MDL_MAG_GetAxes(&lis2mdl, &mag);

    // printf("Magnetometer: %.2f %.2f %.2f\n",
    //        (float)(mag.x * mag_scale),
    //        (float)(mag.y * mag_scale),
    //        (float)(mag.z * mag_scale));

    // float roll, pitch, yaw;

    // roll = atan2(mag.y, mag.z);
    // pitch = atan2(-mag.x, sqrt(mag.y * mag.y + mag.z * mag.z));
    // yaw = atan2(mag.z, mag.x);

    // printf("Roll: %.1f Pitch: %.1f Yaw: %.1f\n", roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG);

    osDelay(100);
  }
}

// Encoders

#define AS5600_ADDR 0x36

void StartEncodersTask(void *argument)
{
  uint8_t data[2] = {0};

  while (1)
  {
    HAL_StatusTypeDef status = I2C_Read_Register(&hi2c1, &i2c1MutexHandle, 0x77, 0x0C, data, 2);

    if (status != HAL_OK)
    {
      printf("AS5600: Error\n");

      osDelay(50);
      continue;
    }

    printf("AS5600: %d\n", (data[0] << 8) | data[1]);

    osDelay(50);
  }
}

/* USER CODE END Application */

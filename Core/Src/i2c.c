/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    i2c.c
 * @brief   This file provides code for the configuration
 *          of the I2C instances.
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
#include "i2c.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00B03FDB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}
/* I2C2 init function */
void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00B03FDB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* I2C1 DMA Init */
    /* I2C1_RX Init */
    hdma_i2c1_rx.Instance = DMA1_Stream6;
    hdma_i2c1_rx.Init.Request = DMA_REQUEST_I2C1_RX;
    hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_i2c1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_i2c1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(i2cHandle,hdmarx,hdma_i2c1_rx);

    /* I2C1_TX Init */
    hdma_i2c1_tx.Instance = DMA1_Stream7;
    hdma_i2c1_tx.Init.Request = DMA_REQUEST_I2C1_TX;
    hdma_i2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_i2c1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_i2c1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(i2cHandle,hdmatx,hdma_i2c1_tx);

    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
  else if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
    PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_GPIOF_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PF0     ------> I2C2_SDA
    PF1     ------> I2C2_SCL
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* I2C2 clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
  /* USER CODE BEGIN I2C2_MspInit 1 */

  /* USER CODE END I2C2_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

    /* I2C1 DMA DeInit */
    HAL_DMA_DeInit(i2cHandle->hdmarx);
    HAL_DMA_DeInit(i2cHandle->hdmatx);

    /* I2C1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
  else if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PF0     ------> I2C2_SDA
    PF1     ------> I2C2_SCL
    */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_0);

    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_1);

  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/*
 * Structure to hold callback context. It stores a binary semaphore that will be released
 * in the DMA complete callback and the original callback pointer so it can be restored later.
 */
typedef struct
{
  osSemaphoreId_t semaphore;
  void (*origRxCpltCallback)(I2C_HandleTypeDef *);
  void (*origTxCpltCallback)(I2C_HandleTypeDef *);
} I2C_CallbackContext_t;

/*
 * Helper function to return a static context pointer for a given I2C.
 * Modify this function to include all I2C instances your application uses.
 */
static I2C_CallbackContext_t *GetI2CContext(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    static I2C_CallbackContext_t i2c1_context = {0};
    return &i2c1_context;
  }
  else if (hi2c->Instance == I2C2)
  {
    static I2C_CallbackContext_t i2c2_context = {0};
    return &i2c2_context;
  }
  // Add additional UART instances as needed.
  return NULL;
}

/*
 * Proxy callback for RX complete.
 */
static void I2C_RxCpltCallback_Proxy(I2C_HandleTypeDef *hi2c)
{
  I2C_CallbackContext_t *ctx = GetI2CContext(hi2c);
  if (ctx && ctx->semaphore != NULL)
  {
    osSemaphoreRelease(ctx->semaphore);
  }
  if (ctx && ctx->origRxCpltCallback)
  {
    ctx->origRxCpltCallback(hi2c);
  }
}

/*
 * Proxy callback for TX complete.
 */
static void I2C_TxCpltCallback_Proxy(I2C_HandleTypeDef *hi2c)
{
  I2C_CallbackContext_t *ctx = GetI2CContext(hi2c);
  if (ctx && ctx->semaphore != NULL)
  {
    osSemaphoreRelease(ctx->semaphore);
  }
  if (ctx && ctx->origTxCpltCallback)
  {
    ctx->origTxCpltCallback(hi2c);
  }
}

HAL_StatusTypeDef I2C_Read_Register(I2C_HandleTypeDef *hi2c, osMutexId_t *mi2c, uint16_t devAddr, uint8_t regAddr, uint8_t *pData, uint16_t size, uint32_t timeout)
{
  HAL_StatusTypeDef status;

  /* Acquire mutex for exclusive access */
  if (osMutexAcquire(*mi2c, timeout) != osOK)
  {
    return HAL_BUSY;
  }

  I2C_CallbackContext_t *ctx = GetI2CContext(hi2c);
  if (ctx == NULL)
  {
    osMutexRelease(*mi2c);
    return HAL_ERROR;
  }

  /* Create a semaphore for this transfer */
  ctx->semaphore = osSemaphoreNew(1, 0, NULL);
  if (ctx->semaphore == NULL)
  {
    osMutexRelease(*mi2c);
    return HAL_ERROR;
  }

  /* Save and override the RX complete callback */
  ctx->origRxCpltCallback = hi2c->MemRxCpltCallback;
  hi2c->MemRxCpltCallback = I2C_RxCpltCallback_Proxy;

  // Flush any leftover data register

  /* Start DMA-based reception */
  status = HAL_I2C_Mem_Read_DMA(hi2c, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT, pData, size);
  if (status != HAL_OK)
  {
    hi2c->MemRxCpltCallback = ctx->origRxCpltCallback;
    osSemaphoreDelete(ctx->semaphore);
    osMutexRelease(*mi2c);
    return status;
  }

  /* Wait for transfer completion or timeout */
  if (osSemaphoreAcquire(ctx->semaphore, timeout) != osOK)
  {
    HAL_I2C_AbortCpltCallback(hi2c);
    status = HAL_TIMEOUT;
  }

  /* Restore original callback and clean up */
  hi2c->MemRxCpltCallback = ctx->origRxCpltCallback;
  osSemaphoreDelete(ctx->semaphore);
  osMutexRelease(*mi2c);
  return status;
}

HAL_StatusTypeDef I2C_Write_Register(I2C_HandleTypeDef *hi2c, osMutexId_t *mi2c, uint16_t devAddr, uint8_t regAddr, const uint8_t *pData, uint16_t size, uint32_t timeout)
{
  HAL_StatusTypeDef status;

  /* Acquire mutex for exclusive access */
  if (osMutexAcquire(*mi2c, timeout) != osOK)
  {
    return HAL_BUSY;
  }

  I2C_CallbackContext_t *ctx = GetI2CContext(hi2c);
  if (ctx == NULL)
  {
    osMutexRelease(*mi2c);
    return HAL_ERROR;
  }

  /* Create a semaphore for this transfer */
  ctx->semaphore = osSemaphoreNew(1, 0, NULL);
  if (ctx->semaphore == NULL)
  {
    osMutexRelease(*mi2c);
    return HAL_ERROR;
  }

  /* Save and override the TX complete callback */
  ctx->origTxCpltCallback = hi2c->MemTxCpltCallback;
  hi2c->MemTxCpltCallback = I2C_TxCpltCallback_Proxy;

  /* Start DMA-based reception */
  status = HAL_I2C_Mem_Write_DMA(hi2c, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT, pData, size);
  if (status != HAL_OK)
  {
    hi2c->MemTxCpltCallback = ctx->origTxCpltCallback;
    osSemaphoreDelete(ctx->semaphore);
    osMutexRelease(*mi2c);
    return status;
  }

  /* Wait for transfer completion or timeout */
  if (osSemaphoreAcquire(ctx->semaphore, timeout) != osOK)
  {
    HAL_I2C_AbortCpltCallback(hi2c);
    status = HAL_TIMEOUT;
  }

  /* Restore original callback and clean up */
  hi2c->MemTxCpltCallback = ctx->origTxCpltCallback;
  osSemaphoreDelete(ctx->semaphore);
  osMutexRelease(*mi2c);
  return status;
}

/* USER CODE END 1 */

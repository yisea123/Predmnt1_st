/**
  ******************************************************************************
  * @file    stm32f4xx_UART.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version V1.0.0
  * @date    08-Feb-2019
  * @brief   This file provides the global UART implementation
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_UART.h"
#include "stm32f4xx_periph_conf.h"

/** @addtogroup Projects
  * @{
  */

/** @addtogroup DEMONSTRATIONS Demonstrations
  * @{
  */

/** @addtogroup PREDCTIVE_MAINTENANCE Predictive Maintenance
  * @{
  */

/** @addtogroup PREDCTIVE_MAINTENANCE_stm32f4xx_UART Predictive Maintenance stm32f4xx UART
  * @{
  */

/** @addtogroup PREDCTIVE_MAINTENANCE_stm32f4xx_UART_EXPORTED_VARIABLES Predictive Maintenance stm32f4xx UART Exported Variables
  * @{
  */

/* exported variables */

/* Global UART handle */
UART_HandleTypeDef UartHandle;

/**
  * @}
  */

/** @addtogroup PREDCTIVE_MAINTENANCE_stm32f4xx_UART_EXPORTED_FUNCTIONS Predictive Maintenance stm32f4xx UART Exported Functions
  * @{
  */

/* Exported Functions */
/**
 * @brief  Configures UART interface
 * @param  None
 * @retval HAL status
 */
HAL_StatusTypeDef UART_Global_Init(void)
{
  HAL_StatusTypeDef ret_val = HAL_OK;
  
  if(HAL_UART_GetState(&UartHandle) == HAL_UART_STATE_RESET) {
    /* DMA controller clock enable */
    USART2_CMN_DEFAULT_DMA_CLK_ENABLE();
    
    /* Service UART DMA Stream IRQ interrupt configuration for RX */
    HAL_NVIC_SetPriority(USART2_CMN_DEFAULT_RX_DMA_Stream_IRQn, USART2_CMN_DEFAULT_RX_DMA_Stream_IRQ_PP, USART2_CMN_DEFAULT_RX_DMA_Stream_IRQ_SP);
    HAL_NVIC_EnableIRQ(USART2_CMN_DEFAULT_RX_DMA_Stream_IRQn);

    /* Service UART DMA Stream IRQ interrupt configuration for TX */
    HAL_NVIC_SetPriority(USART2_CMN_DEFAULT_TX_DMA_Stream_IRQn, USART2_CMN_DEFAULT_TX_DMA_Stream_IRQ_PP, USART2_CMN_DEFAULT_TX_DMA_Stream_IRQ_SP);
    HAL_NVIC_EnableIRQ(USART2_CMN_DEFAULT_TX_DMA_Stream_IRQn);
  
    /* I2C configuration */
    UartHandle.Instance          = USART2;
    UartHandle.Init.BaudRate     = USART2_CMN_DEFAULT_BAUDRATE;
    UartHandle.Init.WordLength   = USART2_CMN_DEFAULT_WORLDLENGTH;
    UartHandle.Init.StopBits     = USART2_CMN_DEFAULT_STOPBITS;
    UartHandle.Init.Parity       = USART2_CMN_DEFAULT_PARITY;
    UartHandle.Init.HwFlowCtl    = USART2_CMN_DEFAULT_HWFLOWCTL;
    UartHandle.Init.Mode         = USART2_CMN_DEFAULT_MODE;
    UartHandle.Init.OverSampling = USART2_CMN_DEFAULT_OVERSAMPLING;

    ret_val = HAL_UART_Init(&UartHandle);
  }
  return ret_val;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/


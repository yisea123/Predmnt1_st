/**
  ******************************************************************************
  * @file    hci_tl_interface.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version V1.0.0
  * @date    08-Feb-2019
  * @brief   This file contains all the functions prototypes for the STM32
  *          BlueNRG-MS HCI Transport Layer interface
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HCI_TL_INTERFACE_H
#define __HCI_TL_INTERFACE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "nucleo_f446re_bus.h"
   
/** @addtogroup Projects
  * @{
  */

/** @addtogroup DEMONSTRATIONS Demonstrations
  * @{
  */

/** @addtogroup PREDCTIVE_MAINTENANCE Predictive Maintenance
  * @{
  */

/** @addtogroup PREDCTIVE_MAINTENANCE_HCI_TL_INTERFACE Predictive Maintenance hci tl interface
  * @{
  */

/** @addtogroup PREDCTIVE_MAINTENANCE_HCI_TL_INTERFACE_EXPORTED_DEFINES Predictive Maintenance hci tl interface Exported Defines
 * @{
 */
   
/* Exported Defines ----------------------------------------------------------*/
#define HCI_TL_SPI_EXTI_PORT  GPIOA
#define HCI_TL_SPI_EXTI_PIN   GPIO_PIN_0
#define HCI_TL_SPI_EXTI_IRQn  EXTI0_IRQn

#define HCI_TL_SPI_IRQ_PORT   GPIOA
#define HCI_TL_SPI_IRQ_PIN    GPIO_PIN_0

#define HCI_TL_SPI_CS_PORT    GPIOA
#define HCI_TL_SPI_CS_PIN     GPIO_PIN_1

#define HCI_TL_RST_PORT       GPIOA
#define HCI_TL_RST_PIN        GPIO_PIN_8
   
/**
  * @}
  */

   
/** @addtogroup PREDCTIVE_MAINTENANCE_HCI_TL_INTERFACE_EXPORTED_IO_BUS_FUNCTIONS Predictive Maintenance hci tl interface Exported IO Bus Functions
 * @{
 */
   
/* Exported Functions IO Operation and BUS services --------------------------------------------------------*/
int32_t HCI_TL_SPI_Init    (void* pConf);
int32_t HCI_TL_SPI_DeInit  (void);
int32_t HCI_TL_SPI_Receive (uint8_t* buffer, uint16_t size);
int32_t HCI_TL_SPI_Send    (uint8_t* buffer, uint16_t size);
int32_t HCI_TL_SPI_Reset   (void);

/**
  * @}
  */

/** @addtogroup PREDCTIVE_MAINTENANCE_HCI_TL_INTERFACE_EXPORTED_MAIN_FUNCTIONS Predictive Maintenance hci tl interface Exported Main Functions
 * @{
 */

/* Exported Functions hci_tl_interface main functions --------------------------------------------------------*/
void hci_tl_lowlevel_init  (void);
void hci_tl_lowlevel_isr   (void);

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


#ifdef __cplusplus
}
#endif

#endif /* __HCI_TL_INTERFACE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

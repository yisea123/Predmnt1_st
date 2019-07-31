/**
  ******************************************************************************
  * @file    PREDMNT1_config.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version V1.0.0
  * @date    08-Feb-2019
  * @brief   FP-IND-PREDMNT1 configuration
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
#ifndef __PREDMNT1_CONFIG_H
#define __PREDMNT1_CONFIG_H

/* Exported define ------------------------------------------------------------*/

/* Define the ALLMEMS1 MAC address, otherwise it will create a Unique MAC */
//#define MAC_PREDMNT1 0xFF, 0xEE, 0xDD, 0xAA, 0xAA, 0xAA

/* Environmental Sensor Istance */
#define TEMPERATURE_INSTANCE_1   IKS01A2_HTS221_0
#define HUMIDITY_INSTANCE        IKS01A2_HTS221_0
#define TEMPERATURE_INSTANCE_2   IKS01A2_LPS22HB_0
#define PRESSURE_INSTANCE        IKS01A2_LPS22HB_0

/* Motion Sensor Istance */
#define MAGNETO_ISTANCE         IKS01A2_LSM303AGR_MAG_0

#ifdef IKS01A2_ISM330DLC_0
  #define ACCELERO_INSTANCE     IKS01A2_ISM330DLC_0
  #define GYRO_INSTANCE         IKS01A2_ISM330DLC_0
#endif /* IKS01A2_ISM330DLC_0 */

#ifdef IKS01A2_LSM6DSL_0
  #define ACCELERO_INSTANCE     IKS01A2_LSM6DSL_0
  #define GYRO_INSTANCE         IKS01A2_LSM6DSL_0
#endif /* IKS01A2_LSM6DSL_0 */
  

#ifdef IKS01A2_ISM330DLC_0
  #define ACCELERO_FIFO_XL_NO_DEC       ISM330DLC_FIFO_XL_NO_DEC
  #define ACCELERO_BYPASS_MODE          ISM330DLC_BYPASS_MODE
  #define ACCELERO_FIFO_MODE            ISM330DLC_FIFO_MODE
  #define ACCELERO_DRDY_PULSED          ISM330DLC_DRDY_PULSED
  #define ACCELERO_DRDY_LATCHED         ISM330DLC_DRDY_LATCHED
  #define ACCELERO_STREAM_MODE          ISM330DLC_STREAM_MODE
#endif /* IKS01A2_ISM330DLC_0 */

#ifdef IKS01A2_LSM6DSL_0
  #define ACCELERO_FIFO_XL_NO_DEC       LSM6DSL_FIFO_XL_NO_DEC
  #define ACCELERO_BYPASS_MODE          LSM6DSL_BYPASS_MODE
  #define ACCELERO_FIFO_MODE            LSM6DSL_FIFO_MODE
  #define ACCELERO_DRDY_PULSED          LSM6DSL_DRDY_PULSED
  #define ACCELERO_DRDY_LATCHED         LSM6DSL_DRDY_LATCHED
#endif /* IKS01A2_LSM6DSL_0 */

#ifndef MAC_PREDMNT1
/* For creating one MAC related to STM32 UID, Otherwise the BLE will use it's random MAC */
#define MAC_STM32UID_PREDMNT1
#endif /* MAC_PREDMNT1 */

/*************** Debug Defines ******************/
/* For enabling the printf on UART */
#define PREDMNT1_ENABLE_PRINTF

/* For enabling connection and notification subscriptions debug */
#define PREDMNT1_DEBUG_CONNECTION

/* For enabling trasmission for notified services (except for quaternions) */
#define PREDMNT1_DEBUG_NOTIFY_TRAMISSION

/* Define The transmission interval in Multiple of 10ms for Microphones dB Values */
#define MICS_DB_UPDATE_MUL_10MS 5

/* Define The transmission interval in Multiple of 10ms for Environmental Values */
#define ENV_UPDATE_MUL_100MS 5

/*************** Don't Change the following defines *************/

/* Package Version only numbers 0->9 */
#define PREDMNT1_VERSION_MAJOR '1'
#define PREDMNT1_VERSION_MINOR '0'
#define PREDMNT1_VERSION_PATCH '0'

/* Define the ALLMEMS1 Name MUST be 7 char long */
#define NAME_BLUEMS 'P','M','1','V',PREDMNT1_VERSION_MAJOR,PREDMNT1_VERSION_MINOR,PREDMNT1_VERSION_PATCH

/* Package Name */
#define PREDMNT1_PACKAGENAME "FP-IND-PREDMNT1"
#define CONFIG_NAME "Application - Predictive Maintenance"

/*****************
* Sensor Setting *
******************/
#ifdef IKS01A2_ISM330DLC_0
  #define HPF_ODR_DIV_4           ISM330DLC_XL_HP_ODR_DIV_4   //!< ISM330DLC HPF Configuration  
  #define HPF_ODR_DIV_100         ISM330DLC_XL_HP_ODR_DIV_100 //!< ISM330DLC HPF Configuration 
  #define HPF_ODR_DIV_9           ISM330DLC_XL_HP_ODR_DIV_9   //!< ISM330DLC HPF Configuration  
  #define HPF_ODR_DIV_400         ISM330DLC_XL_HP_ODR_DIV_400 //!< ISM330DLC HPF Configuration
  #define HPF_NONE                ISM330DLC_XL_HP_NA          //!< HP Filter Disabling
#endif /* IKS01A2_ISM330DLC_0 */

#ifdef IKS01A2_LSM6DSL_0
  #define HPF_ODR_DIV_4           LSM6DSL_XL_HP_ODR_DIV_4   //!< ISM330DLC HPF Configuration  
  #define HPF_ODR_DIV_100         LSM6DSL_XL_HP_ODR_DIV_100 //!< ISM330DLC HPF Configuration 
  #define HPF_ODR_DIV_9           LSM6DSL_XL_HP_ODR_DIV_9   //!< ISM330DLC HPF Configuration  
  #define HPF_ODR_DIV_400         LSM6DSL_XL_HP_ODR_DIV_400 //!< ISM330DLC HPF Configuration
  #define HPF_NONE                LSM6DSL_XL_HP_NA          //!< HP Filter Disabling
#endif /* IKS01A2_LSM6DSL_0 */

/* Set defaul value for HPF Cut frequency */
#define SENSOR_HPF_CUT_VALUE                    HPF_ODR_DIV_400
/* Set defaul ODR value to 6600Hz for FFT demo */
#define SENSOR_ACC_ORD_VALUE                    416
/* Set default decimation value for the FIFO with no decimation */
#define SENSOR_ACC_FIFO_DECIMATION_VALUE        1
/* Set defaul FIFO ODR value */
#define SENSOR_ACC_FIFO_ORD_VALUE               SENSOR_ACC_ORD_VALUE
/* Default value for Accelerometer full scale in g */
#define SENSOR_ACC_FS_DEFAULT                   4

/*  FIFO size limit */
#define FIFO_WATERMARK   ((MotionSP_Parameters.size + 1)*3)

/*************************
* Serial control section *
**************************/
#ifdef PREDMNT1_ENABLE_PRINTF
  #define PREDMNT1_PRINTF(...) printf(__VA_ARGS__)
#else /* PREDMNT1_ENABLE_PRINTF */
  #define PREDMNT1_PRINTF(...)
#endif /* PREDMNT1_ENABLE_PRINTF */

#define PREDMNT1_SCANF(...) scanf(__VA_ARGS__)

/* STM32 Unique ID */
#define STM32_UUID ((uint32_t *)0x1FFF7A10)

/* STM32 MCU_ID */
#define STM32_MCU_ID ((uint32_t *)0xE0042000)
/* Control Section */

#endif /* __PREDMNT1_CONFIG_H */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/

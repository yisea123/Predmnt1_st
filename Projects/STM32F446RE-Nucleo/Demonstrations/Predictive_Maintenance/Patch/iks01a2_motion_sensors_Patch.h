/**
  ******************************************************************************
  * @file    iks01a2_motion_sensors_Patch.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version V1.0.0
  * @date    08-Feb-2019
  * @brief   This file provides a set of functions needed to manage
  *          the motion sensors
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
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
#ifndef IKS01A2_MOTION_SENSOR_H
#define IKS01A2_MOTION_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "iks01a2_conf.h"
#include "motion_sensor.h"

#ifndef USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0
#define USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0          1
#endif

#ifndef USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0
#define USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0    1
#endif

#ifndef USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0
#define USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0    1
#endif
  
#ifndef USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0
#define USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0    1
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
#include "lsm6dsl.h"
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
#include "ism330dlc_Patch.h"
#endif
  
#if ((USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 == 1) || (USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0 == 1))
#include "lsm303agr.h"
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
#include "ism330dlc_Patch.h"
#endif

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup IKS01A2 IKS01A2
 * @{
 */

/** @addtogroup IKS01A2_MOTION_SENSOR IKS01A2 MOTION SENSOR
 * @{
 */

/** @defgroup IKS01A2_MOTION_SENSOR_Exported_Types IKS01A2 MOTION SENSOR Exported Types
 * @{
 */

typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} IKS01A2_MOTION_SENSOR_Axes_t;

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} IKS01A2_MOTION_SENSOR_AxesRaw_t;

/* Motion Sensor instance Info */
typedef struct
{
  uint8_t  Acc;
  uint8_t  Gyro;
  uint8_t  Magneto;
  uint8_t  LowPower;
  uint32_t GyroMaxFS;
  uint32_t AccMaxFS;
  uint32_t MagMaxFS;
  float    GyroMaxOdr;
  float    AccMaxOdr;
  float    MagMaxOdr;
} IKS01A2_MOTION_SENSOR_Capabilities_t;

typedef struct
{
  uint32_t Functions;
} IKS01A2_MOTION_SENSOR_Ctx_t;

/**
 * @}
 */

/** @defgroup IKS01A2_MOTION_SENSOR_Exported_Constants IKS01A2 MOTION SENSOR Exported Constants
 * @{
 */

#if (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 == 1)
#define IKS01A2_LSM6DSL_0 0
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 == 1)
#define IKS01A2_LSM303AGR_ACC_0 (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0)
#endif

#if (USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0 == 1)
#define IKS01A2_LSM303AGR_MAG_0 (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 + USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0)
#endif

#if (USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0 == 1)
#define IKS01A2_ISM330DLC_0 (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 + USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 + USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0)
#endif

#ifndef MOTION_GYRO
#define MOTION_GYRO             1U
#endif
#ifndef MOTION_ACCELERO
#define MOTION_ACCELERO         2U
#endif
#ifndef MOTION_MAGNETO
#define MOTION_MAGNETO          4U
#endif

#define IKS01A2_MOTION_FUNCTIONS_NBR    3U
#define IKS01A2_MOTION_INSTANCES_NBR    (USE_IKS01A2_MOTION_SENSOR_LSM6DSL_0 + USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0 + USE_IKS01A2_MOTION_SENSOR_LSM303AGR_MAG_0 + USE_IKS01A2_MOTION_SENSOR_ISM330DLC_0)

#if (IKS01A2_MOTION_INSTANCES_NBR == 0)
#error "No motion sensor instance has been selected"
#endif

/**
 * @}
 */

/** @addtogroup IKS01A2_MOTION_SENSOR_Exported_Functions IKS01A2_MOTION_SENSOR Exported Functions
 * @{
 */

int32_t IKS01A2_MOTION_SENSOR_Init(uint32_t Instance, uint32_t Functions);
int32_t IKS01A2_MOTION_SENSOR_DeInit(uint32_t Instance);
int32_t IKS01A2_MOTION_SENSOR_GetCapabilities(uint32_t Instance, IKS01A2_MOTION_SENSOR_Capabilities_t *Capabilities);
int32_t IKS01A2_MOTION_SENSOR_ReadID(uint32_t Instance, uint8_t *Id);
int32_t IKS01A2_MOTION_SENSOR_Enable(uint32_t Instance, uint32_t Function);
int32_t IKS01A2_MOTION_SENSOR_Disable(uint32_t Instance, uint32_t Function);
int32_t IKS01A2_MOTION_SENSOR_GetAxes(uint32_t Instance, uint32_t Function, IKS01A2_MOTION_SENSOR_Axes_t *Axes);
int32_t IKS01A2_MOTION_SENSOR_GetAxesRaw(uint32_t Instance, uint32_t Function, IKS01A2_MOTION_SENSOR_AxesRaw_t *Axes);
int32_t IKS01A2_MOTION_SENSOR_GetSensitivity(uint32_t Instance, uint32_t Function, float *Sensitivity);
int32_t IKS01A2_MOTION_SENSOR_GetOutputDataRate(uint32_t Instance, uint32_t Function, float *Odr);
int32_t IKS01A2_MOTION_SENSOR_SetOutputDataRate(uint32_t Instance, uint32_t Function, float Odr);
int32_t IKS01A2_MOTION_SENSOR_GetFullScale(uint32_t Instance, uint32_t Function, int32_t *Fullscale);
int32_t IKS01A2_MOTION_SENSOR_SetFullScale(uint32_t Instance, uint32_t Function, int32_t Fullscale);

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

#endif /* IKS01A2_MOTION_SENSOR_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

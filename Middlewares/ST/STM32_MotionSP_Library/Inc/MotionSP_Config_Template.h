/**
  ******************************************************************************
  * @file    MotionSP_Config_Template.h
  * @author  SLAB Application Team
  * @version 
  * @date    
  * @brief   MotionSP configuration template file for motion signal processing.
  *          This file should be copied to the application folder and renamed
  *          to MotionSP_Config.h.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#ifndef __MOTION_SP_CONFIG_H
#define __MOTION_SP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @addtogroup MIDDLEWARES Middlewares
  * @{
  */

/** @addtogroup ST ST
  * @{
  */

/** @addtogroup STM32_MOTIONSP_LIB STM32 Motion Signal Processing Library
  * @{
  */

/** @addtogroup STM32_MOTIONSP_LIB_EXPORTED_DEFINES STM32 Motion Signal Processing Library Exported Defines
  * @{
  */
  
/* Uncomment this line code for enabling subrange */
/* #define USE_SUBRANGE */

/* Number of sensor axes */
#define NUM_AXES        3

/* Default value for FFT size */
#define SIZE_DEFAULT    512
/* Default value for Moving RMS Filtering in ms */
#define TAU_DEFAULT     500
/* Default value for Windowing Method */
#define WINDOW_DEFAULT  HANNING
/* Default value for Time Domain Analysis */
#define TD_DEFAULT      TD_BOTH_TAU
/* Defualt value for Total acquisition time in ms */
#define TACQ_DEFAULT    2000
  
/* Buffer Ratio */
#define CIRC_BUFFER_RATIO       1.0f
/* Max FFT size */
#define FFT_SIZE_MAX            512u
/* Max MAG size */
#define MAG_SIZE_MAX            (uint16_t)(FFT_SIZE_MAX/2)
/* Max circular buffer for storing input values for FFT */
#define CIRC_BUFFER_SIZE_MAX    (uint16_t)(FFT_SIZE_MAX*CIRC_BUFFER_RATIO)

/* Smooth parameter used for DC filtering */
#define DC_SMOOTH       0.975f
/* GAMMA parameter used for Integration Algorithm */
#define GAMMA           0.5f
/* CONSTANT value for 1g */
#define G_CONV          9.80665f

#ifdef USE_SUBRANGE
  /* Default value for FFT output subranges */
  #define SUBRANGE_DEFAULT      8
  /* Default value for MAX Subranges to analyze */
  #define SUBRANGE_MAX          64
#endif /* USE_SUBRANGE */

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

#endif /* __MOTION_SP_CONFIG_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

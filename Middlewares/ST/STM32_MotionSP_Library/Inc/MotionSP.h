/**
  ******************************************************************************
  * @file           : MotionSP.h
  * @author         : SLAB Application Team
  * @version        :
  * @date           :
  * @brief          : Header for MotionSP.c file.
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
#ifndef __MOTION_SP_H
#define __MOTION_SP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "MotionSP_Config.h"
#include "cube_hal.h"
#include "arm_math.h"

/** @addtogroup MIDDLEWARES Middlewares
  * @{
  */

/** @addtogroup ST ST
  * @{
  */

/** @addtogroup STM32_MOTIONSP_LIB STM32 Motion Signal Processing Library
  * @{
  */

/** @addtogroup STM32_MOTIONSP_LIB_EXPORTED_TYPES STM32 Motion Signal Processing Library Exported Types
  * @{
  */

/**
  * @brief  Window Filtering datatype
  */
typedef enum
{
  RECTANGULAR  = (uint16_t)0x00,       //!< RECTANGULAR FILTERING Type
  HANNING      = (uint16_t)0x01,       //!< HANNING FILTERING Type
  HAMMING      = (uint16_t)0x02,       //!< HAMMING FILTERING Type
  FLAT_TOP     = (uint16_t)0x03,       //!< FLAT TOP FILTERING Type
} Filt_Type_t;

/**
  * @brief  List of comma separated window filtering names - has to correspond with Filt_Type_t
  */
#define WINDOWS_LIST  "RECTANGULAR,HANNING,HAMMING,FLAT_TOP"

/**
  * @brief  Time Domain Analysis type
  */
typedef enum
{
  TD_SPEED      = (uint16_t)0x00, //!< Time Domain ANALYSIS: Speed RMS Moving AVERAGE
  TD_ACCELERO   = (uint16_t)0x01, //!< Time Domain ANALYSIS: Accelerometer RMS Moving AVERAGE
  TD_BOTH_TAU   = (uint16_t)0x02, //!< Time Domain ANALYSIS: Speed and both RMS Moving AVERAGE TAU
} Td_Type_t;

typedef struct
{
  float AXIS_X[CIRC_BUFFER_SIZE_MAX];   //!< Circular arrays for storing X accelero values
  float AXIS_Y[CIRC_BUFFER_SIZE_MAX];   //!< Circular arrays for storing Y accelero values
  float AXIS_Z[CIRC_BUFFER_SIZE_MAX];   //!< Circular arrays for storing Z accelero values
} sAccAxesCircBufferData_t;

/**
  * @brief  Accelerometer Circular Buffer Structure with flags and size
  */
typedef struct
{
  uint16_t Size;                  //!< actual size
  uint16_t IdPos;                 //!< index position
  uint8_t Ovf;                    //!< flag to report an OVF
  sAccAxesCircBufferData_t Data;  //!< Circular arrays for storing accelerometer values
} sCircBuffer_t;                  //!< circular buffer for storing input values for FFT

/**
  * @brief  Struct for MotionSP parameters
  */
typedef struct
{
  uint16_t size;          //!< FFT size
  uint16_t tau;           //!< RMS filter constant
  uint16_t window;        //!< Windowing type selection
  uint16_t td_type;       //!< Feature to monitor in Time Domain Analysis
  uint16_t tacq;          //!< Total acquisition time
  uint8_t  ovl;           //!< Overlapping Parameter
#ifdef USE_SUBRANGE  
  uint16_t subrange_num;  //!< Number of Subrange to evaluate FFT threshold
#endif /* USE_SUBRANGE */
} sMotionSP_Parameter_t;

/**
  * @brief  X-Y-Z Generic Value in float
  */
typedef struct
{
  float AXIS_X;         //!< Generic X Value in float
  float AXIS_Y;         //!< Generic Y Value in float
  float AXIS_Z;         //!< Generic Z Value in float
} SensorVal_f_t;

/**
  * @brief  Structure for values connected to real ODR
  */
typedef struct
{
  float Frequency;      //!< Real Accelerometer Frequency (ODR Measured)
  float Period;         //!< Real Accelerometer Period
  float Tau;            //!< Tau parameter to evaluate Moving RMS
} sAcceleroODR_t;

/**
  * @brief  Structure of all Time domain parameters to measure
  */
typedef struct
{
  SensorVal_f_t AccRms;    //!< X-Y-Z Accelerometer Fast Moving RMS
  SensorVal_f_t SpeedRms;  //!< X-Y-Z Speed Fast Moving RMS
  SensorVal_f_t AccPeak;   //!< X-Y-Z Accelerometer Peak
} sAcceleroParam_t;


/**
  * @brief  Board Parameters structure for storing in MCU flash
  */
typedef struct
{
  float AXIS_X[FFT_SIZE_MAX];   //!< X Array fot FFT Input
  float AXIS_Y[FFT_SIZE_MAX];   //!< Y Array fot FFT Input
  float AXIS_Z[FFT_SIZE_MAX];   //!< Z Array fot FFT Input
} sAccAxesArray_t;

/**
  * @brief  Sum Counter data structure definition
  */
typedef struct
{
  uint16_t AXIS_X;      //!< Sum counter for X FFT used during Averaging
  uint16_t AXIS_Y;      //!< Sum counter for Y FFT used during Averaging
  uint16_t AXIS_Z;      //!< Sum counter for Z FFT used during Averaging
} sSumCnt_t;

/**
  * @brief  Arrays for storing magnitude average values
  */
typedef struct
{
  float AXIS_X[MAG_SIZE_MAX];   //!< X Array magnitude average values
  float AXIS_Y[MAG_SIZE_MAX];   //!< Y Array magnitude average values
  float AXIS_Z[MAG_SIZE_MAX];   //!< Z Array magnitude average values
} sAxesMagBuff_t;

/**
  * @brief  Structure for FFT Results
  */
typedef struct
{
  float X_Value;                //!< X Max Value inside the FFT Output Array
  uint32_t X_Index;             //!< X Max Index inside the FFT Output Array
  uint16_t X_FFT_AVG;           //!< X FFT counter to build the FFT Average
  float Y_Value;                //!< Y Max Value inside the FFT Output Array
  uint32_t Y_Index;             //!< Z Max Index inside the FFT Output Array
  uint16_t Y_FFT_AVG;           //!< X FFT counter to build the FFT Average
  float Z_Value;                //!< Z Max Value inside the FFT Output Array
  uint32_t Z_Index;             //!< Y Max Index inside the FFT Output Array
  uint16_t Z_FFT_AVG;           //!< Z FFT counter to build the FFT Average
} sAxesMagResults_t;

#ifdef USE_SUBRANGE
typedef struct {
  float AXIS_X[SUBRANGE_MAX];   //!< X Array Subrange datatype
  float AXIS_Y[SUBRANGE_MAX];   //!< Y Array Subrange datatype
  float AXIS_Z[SUBRANGE_MAX];   //!< Z Array Subrange datatype
} sSubrange_t; 
#endif /* USE_SUBRANGE */

/**
  * @}
  */

/** @addtogroup STM32_MOTIONSP_LIB_EXPORTED_FUNCTIONS_PROTOTYPES STM32 Motion Signal Processing Library Exported Functions Prototypes
  * @{
  */

void MotionSP_accDelOffset(SensorVal_f_t *pDstArr, SensorVal_f_t *pSrcArr, float Smooth, uint16_t Restart);
void MotionSP_CreateAccCircBuffer(sCircBuffer_t *pCircBuff, SensorVal_f_t buffType);
void MotionSP_TimeDomainProcess(sAcceleroParam_t *sTimeDomain, Td_Type_t td_type, uint8_t Restart);

void MotionSP_fftCalc(arm_rfft_fast_instance_f32 *pfftS, float *pfftIn, float *pfftOut);
void MotionSP_fftAdapt(sAxesMagBuff_t *pfftCmplxMag, uint16_t size);
void MotionSP_fftFindPeak(sAxesMagBuff_t *pfftCmplxMag, uint16_t size, sAxesMagResults_t *AccAxesMagResults);
void MotionSP_SetWindFiltArray(float *Filter_Params, uint16_t size, Filt_Type_t Ftype);
void motionSP_fftUseWindow(float *pDstArr, float *pSrcArr, uint16_t SizeArr, float *Window_Params);
uint8_t MotionSP_fftInBuild(float *pDst, uint16_t DstSize, float *pSrc, uint16_t SrcSize, uint16_t SrcLastPos);
uint8_t MotionSP_fftAverageCalcSamples(float *pDstArr, float *pSrcArr, uint16_t LenArr, uint16_t *pSumCnt, uint8_t MaxSumCnt);
uint8_t MotionSP_fftAverageCalcTime(float *pDstArr, float *pSrcArr, uint16_t LenArr, uint16_t *pSumCnt, uint8_t FinishAvg);
void MotionSP_FrequencyDomainProcess(void);
void MotionSP_evalMaxAmplitudeRange(float *pfftCmplxMagAxis, uint16_t subrange, float *SR_Amplitude, float *SR_Bin_Value);

/**
  * @}
  */

/** @addtogroup STM32_MOTIONSP_LIB_EXPORTED_VARIABLES STM32 Motion Signal Processing Library Exported Variables
  * @{
  */

extern sCircBuffer_t AccCircBuffer;
extern uint8_t fftIsEnabled;
extern sMotionSP_Parameter_t MotionSP_Parameters;
extern sAcceleroParam_t sTimeDomain;
extern sAxesMagResults_t AccAxesMagResults;
extern uint8_t FinishAvgFlag;
extern uint16_t magSize;
extern sAcceleroODR_t AcceleroODR;
extern uint16_t accCircBuffIndexForFft;
extern sSumCnt_t AccSumCnt;
extern float Filter_Params[SIZE_DEFAULT];
extern arm_rfft_fast_instance_f32 fftS;
extern float fftOut[NUM_AXES][SIZE_DEFAULT];

#ifdef USE_SUBRANGE
extern sSubrange_t SRAmplitude;
extern sSubrange_t SRBinVal;
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

#endif /* __MOTION_SP_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
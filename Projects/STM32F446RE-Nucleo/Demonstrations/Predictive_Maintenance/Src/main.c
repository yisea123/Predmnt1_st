/**
  ******************************************************************************
  * @file    main.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version V1.0.0
  * @date    08-Feb-2019
  * @brief   Main program body
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

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>

#include <limits.h>
#include "TargetFeatures.h"
#include "main.h"
#include "OTA.h"
#include "MetaDataManager.h"
#include "sensor_service.h"
#include "bluenrg_utils.h"

/** @addtogroup Projects
  * @{
  */

/** @addtogroup DEMONSTRATIONS Demonstrations
  * @{
  */

/** @addtogroup PREDCTIVE_MAINTENANCE Predictive Maintenance
  * @{
  */

/** @addtogroup PREDCTIVE_MAINTENANCE_MAIN Predictive Maintenance main
  * @{
  */
   
/* Private typedef -----------------------------------------------------------*/

/** @defgroup PREDCTIVE_MAINTENANCE_MAIN_PRIVATE_DEFINE Predictive Maintenance Main Private Define
  * @{
  */

/* Private define ------------------------------------------------------------*/
#define CHECK_VIBRATION_PARAM ((uint16_t)0x1234)

/**
  * @}
  */

/** @defgroup PREDCTIVE_MAINTENANCE_MAIN_IMPORTED_VARIABLES Predictive Maintenance Main Imported Variables
  * @{
  */

/* Imported Variables -------------------------------------------------------------*/
extern uint8_t set_connectable;
extern int connected;
extern volatile uint32_t FFT_Alarm;
    
extern volatile float RMS_Ch[];
extern float DBNOISE_Value_Old_Ch[];
extern uint16_t PCM_Buffer[];
extern uint32_t NumSample;

extern uint32_t Start_Tick;

extern uint16_t PDM_Buffer[];

extern sTimeDomainAlarm_t sTdAlarm;
extern sTimeDomainThresh_t sTdRmsThresholds;
extern sTimeDomainThresh_t sTdPkThresholds;
extern sAcceleroParam_t sTimeDomainVal;

/* X-Y-Z Frequency domain Subranges Status */
extern sFreqDomainAlarm_t THR_Fft_Alarms;

extern sAccelerometer_Parameter_t Accelerometer_Parameters;

/**
  * @}
  */

/** @defgroup PREDCTIVE_MAINTENANCE_MAIN_EXPORTED_VARIABLES Predictive Maintenance Main Exported Variables
  * @{
  */

/* Exported Variables -------------------------------------------------------------*/
volatile uint8_t AccIntReceived= 0;
volatile uint8_t FifoEnabled = 0;
volatile uint32_t PredictiveMaintenance=     0;

uint8_t IsFirstTime = 0;

float sensitivity;

/* Acc sensitivity multiply by FROM_MG_TO_G constant */
float sensitivity_Mul;

uint32_t ConnectionBleStatus  =0;
uint32_t FirstConnectionConfig =0;

uint8_t BufferToWrite[256];
int32_t BytesToWrite;

TIM_HandleTypeDef    TimCCHandle;
TIM_HandleTypeDef    TimEnvHandle;
TIM_HandleTypeDef    TimAudioDataHandle;

uint8_t bdaddr[6];

uint32_t uhCCR4_Val = DEFAULT_uhCCR4_Val;

uint8_t  NodeName[8];
uint16_t VibrationParam[11];

/**
  * @}
  */

/** @defgroup PREDCTIVE_MAINTENANCE_MAIN_PRIVATE_VARIABLES Predictive Maintenance Main Private Variables
  * @{
  */

/* Private variables ---------------------------------------------------------*/
/* Table with All the known Meta Data */
MDM_knownGMD_t known_MetaData[]={
  {GMD_NODE_NAME,      (sizeof(NodeName))},
  {GMD_VIBRATION_PARAM,(sizeof(VibrationParam))},
  {GMD_END    ,0}/* THIS MUST BE THE LAST ONE */
};

static volatile uint32_t ButtonPressed=           0;
static volatile uint32_t HCI_ProcessEvent=        0;
static volatile uint32_t SendEnv=                 0;
static volatile uint32_t SendAudioLevel=          0;
static volatile uint32_t SendAccGyroMag=          0;

/* CRC handler declaration */
static CRC_HandleTypeDef hcrc;

float *FDWarnThresh;
float *FDAlarmThresh;

/**
  * @}
  */

/** @defgroup PREDCTIVE_MAINTENANCE_MAIN_PRIVATE_FUNCTIONS_PROTOTYPES Predictive Maintenance Main Private Functions Prototypes
  * @{
  */

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

static void InitTimers(void);
static void Init_BlueNRG_Custom_Services(void);
static void Init_BlueNRG_Stack(void);
static void InitPredictiveMaintenance(void);

static void Set2GAccelerometerFullScale(void);

static void MX_CRC_Init(void);

static unsigned char ReCallNodeNameFromMemory(void);
static unsigned char ReCallVibrationParamFromMemory(void);

static void SendEnvironmentalData(void);
static void ButtonCallback(void);
static void SendMotionData(void);
static void SendAudioLevelData(void);

static void AudioProcess(void);
static void AudioProcess_DB_Noise(void);

/**
  * @}
  */

/** @defgroup PREDCTIVE_MAINTENANCE_MAIN_PRIVATE_FUNCTIONS Predictive Maintenance Main Private Functions
  * @{
  */

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  HAL_Init();

  /* Configure the System clock */
  SystemClock_Config();
      
  InitTargetPlatform(TARGET_NUCLEO);
  
  /* For enabling CRC clock for using motion libraries (for checking if STM32 microprocessor is used)*/
  MX_CRC_Init();
  
  /* Check the MetaDataManager */
 InitMetaDataManager((void *)&known_MetaData,MDM_DATA_TYPE_GMD,NULL); 
  
  PREDMNT1_PRINTF("\n\t(HAL %ld.%ld.%ld_%ld)\r\n"
        "\tCompiled %s %s"

#if defined (__IAR_SYSTEMS_ICC__)
        " (IAR)\r\n"
#elif defined (__CC_ARM)
        " (KEIL)\r\n"
#elif defined (__GNUC__)
        " (openstm32)\r\n"
#endif
         "\tSend Every %4dmS Temperature/Humidity/Pressure\r\n"
         "\tSend Every %4dmS Acc/Gyro/Magneto\r\n"
         "\tSend Every %4dmS dB noise\r\n\n",
           HAL_GetHalVersion() >>24,
          (HAL_GetHalVersion() >>16)&0xFF,
          (HAL_GetHalVersion() >> 8)&0xFF,
           HAL_GetHalVersion()      &0xFF,
         __DATE__,__TIME__,
         ENV_UPDATE_MUL_100MS * 100,
         DEFAULT_uhCCR4_Val/10,
         MICS_DB_UPDATE_MUL_10MS * 10);

#ifdef PREDMNT1_DEBUG_CONNECTION
  PREDMNT1_PRINTF("Debug Connection         Enabled\r\n");
#endif /* PREDMNT1_DEBUG_CONNECTION */

#ifdef PREDMNT1_DEBUG_NOTIFY_TRAMISSION
  PREDMNT1_PRINTF("Debug Notify Trasmission Enabled\r\n\n");
#endif /* PREDMNT1_DEBUG_NOTIFY_TRAMISSION */

  /* Set Node Name */
  ReCallNodeNameFromMemory();

  /* Initialize the BlueNRG */
  Init_BlueNRG_Stack();

  /* Initialize the BlueNRG Custom services */
  Init_BlueNRG_Custom_Services();  
  
  /* Check the BootLoader Compliance */
  PREDMNT1_PRINTF("\r\n");
  if(CheckBootLoaderCompliance()) {
    PREDMNT1_PRINTF("BootLoader Compliant with FOTA procedure\r\n\n");
  } else {
    PREDMNT1_PRINTF("ERROR: BootLoader NOT Compliant with FOTA procedure\r\n\n");
  }

  /* Set Accelerometer Full Scale to 2G */
  Set2GAccelerometerFullScale();

  /* initialize timers */
  InitTimers();
  
  /* Predictive Maintenance Initialization */
  InitPredictiveMaintenance();
  
  /* Infinite loop */
  while (1)
  {
    /* Led Blinking when there is not a client connected */
    if(!connected)
    {
      if(!TargetBoardFeatures.LedStatus) {
        if(!(HAL_GetTick()&0x3FF)) {
          LedOnTargetPlatform();
        }
      } else {
        if(!(HAL_GetTick()&0x3F)) {
          LedOffTargetPlatform();
        }
      }
    }

    if(set_connectable){     
      if(NecessityToSaveMetaDataManager) {
        uint32_t Success = EraseMetaDataManager();
        if(Success) {
          SaveMetaDataManager();
        }
      }

      /* Now update the BLE advertize data and make the Board connectable */
      setConnectable();
      set_connectable = FALSE;
    }

    /* Handle user button */
    if(ButtonPressed) {
      ButtonCallback();
      ButtonPressed=0;       
    }
    
    if(PredictiveMaintenance){
      if (IsFirstTime)
      {
        IsFirstTime = 0;

	/* Initializes the MotionSP Vibration parameters */
        MotionSP_VibrationInit();
		
        if(FFT_Alarm)
        {
          /* Initialization of Alarm Status on Axes, Alarm Values Reported
             and Thresholds to detect WARNING and ALARM conditions */
          MotionSP_TimeDomainAlarmInit(&sTdAlarm, &sTimeDomainVal, &sTdRmsThresholds, &sTdPkThresholds);
          
          /* Frequency domain initialization of Alarm Status */
          MotionSP_FreqDomainAlarmInit(&FDWarnThresh, &FDAlarmThresh, &THR_Fft_Alarms, MotionSP_Parameters.subrange_num);
        }
        
        /* Configure the FIFO settings in according with others parammeters changed */
        MotionSP_ConfigFifo();
        
        enable_FIFO();
        
        /* Start of the Time Window */
        Start_Tick = HAL_GetTick();
      }
	  
      MotionSP_VibrationAnalysis();
    }

    /* handle BLE event */
    if(HCI_ProcessEvent) {
      HCI_ProcessEvent=0;
      hci_user_evt_proc();
    }

    /* Environmental Data */
    if(SendEnv) {
      SendEnv=0;
      SendEnvironmentalData();
    }
    
    /* Mic Data */
    if (SendAudioLevel) {
      SendAudioLevel = 0;
      SendAudioLevelData();
    }

    /* Motion Data */
    if(SendAccGyroMag) {
      SendAccGyroMag=0;
      SendMotionData();
    }
    
    /* Wait for Event */
    __WFI();
  }
}

/**
  * @brief  This function sets the ACC FS to 2g
  * @param  None
  * @retval None
  */
static void Set2GAccelerometerFullScale(void)
{
  /* Set Full Scale to +/-2g */
  IKS01A2_MOTION_SENSOR_SetFullScale(ACCELERO_INSTANCE, MOTION_ACCELERO, 2.0f);
  
  /* Read the Acc Sensitivity */
  IKS01A2_MOTION_SENSOR_GetSensitivity(ACCELERO_INSTANCE, MOTION_ACCELERO, &sensitivity);
  sensitivity_Mul = sensitivity* ((float) FROM_MG_TO_G);
}

/**
  * @brief  Callback for user button
  * @param  None
  * @retval None
  */
static void ButtonCallback(void)
{
  PREDMNT1_PRINTF("\r\nUser Button Pressed\r\n\r\n");
}


/**
  * @brief  Send Motion Data Acc/Mag/Gyro to BLE
  * @param  None
  * @retval None
  */
static void SendMotionData(void)
{
  IKS01A2_MOTION_SENSOR_Axes_t ACC_Value;
  IKS01A2_MOTION_SENSOR_Axes_t GYR_Value;
  IKS01A2_MOTION_SENSOR_Axes_t MAG_Value;

  /* Read the Acc values */
  if(TargetBoardFeatures.AccSensorIsInit)
  {
    IKS01A2_MOTION_SENSOR_GetAxes(ACCELERO_INSTANCE, MOTION_ACCELERO, &ACC_Value);
  }
  else
  {
    ACC_Value.x = ACC_Value.y = ACC_Value.z =0;
  }

  /* Read the Magneto values */
  if(TargetBoardFeatures.MagSensorIsInit)
  {
    IKS01A2_MOTION_SENSOR_GetAxes(MAGNETO_ISTANCE, MOTION_MAGNETO, &MAG_Value);
  }
  else
  {
    MAG_Value.x = MAG_Value.y = MAG_Value.z =0;
  }
  
  /* Read the Gyro values */
  if(TargetBoardFeatures.GyroSensorIsInit)
  {
    IKS01A2_MOTION_SENSOR_GetAxes(GYRO_INSTANCE,MOTION_GYRO, &GYR_Value);
  }
  else
  {
    GYR_Value.x = GYR_Value.y = GYR_Value.z =0;
  }
  
  AccGyroMag_Update(&ACC_Value,&GYR_Value,&MAG_Value);
}

/**
* @brief  User function that is called when 1 ms of PDM data is available.
* @param  none
* @retval None
*/
static void AudioProcess(void)
{
  BSP_AUDIO_IN_PDMToPCM(BSP_AUDIO_INSTANCE,(uint16_t * )PDM_Buffer,PCM_Buffer);

  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL))
  {
    AudioProcess_DB_Noise();
  }
}

/**
* @brief  User function that is called when 1 ms of PDM data is available.
* @param  none
* @retval None
*/
static void AudioProcess_DB_Noise(void)
{
  int32_t i;
  int32_t NumberMic;
  
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL)) {
    for(i = 0; i < (NumSample/2); i++){
      for(NumberMic=0;NumberMic<AUDIO_CHANNELS;NumberMic++) {
        RMS_Ch[NumberMic] += (float)((int16_t)PCM_Buffer[i*AUDIO_CHANNELS+NumberMic] * ((int16_t)PCM_Buffer[i*AUDIO_CHANNELS+NumberMic]));
      }
    }
  }
}

/**
  * @brief  Send Audio Level Data (Ch1) to BLE
  * @param  None
  * @retval None
  */
static void SendAudioLevelData(void)
{
  int32_t NumberMic;
  uint16_t DBNOISE_Value_Ch[AUDIO_CHANNELS];
  
  for(NumberMic=0;NumberMic<(AUDIO_CHANNELS);NumberMic++) {
    DBNOISE_Value_Ch[NumberMic] = 0;

    //RMS_Ch[NumberMic] /= (16.0f*MICS_DB_UPDATE_MUL_10MS*10);
    RMS_Ch[NumberMic] /= ((float)(NumSample/2)*MICS_DB_UPDATE_MUL_10MS*10);

    DBNOISE_Value_Ch[NumberMic] = (uint16_t)((120.0f - 20 * log10f(32768 * (1 + 0.25f * (AUDIO_VOLUME_INPUT /*AudioInVolume*/ - 4))) + 10.0f * log10f(RMS_Ch[NumberMic])) * 0.3f + DBNOISE_Value_Old_Ch[NumberMic] * 0.7f);
    DBNOISE_Value_Old_Ch[NumberMic] = DBNOISE_Value_Ch[NumberMic];
    RMS_Ch[NumberMic] = 0.0f;
  }
  
  AudioLevel_Update(DBNOISE_Value_Ch);
}

/**
  * @brief  Send Environmetal Data (Temperature/Pressure/Humidity) to BLE
  * @param  None
  * @retval None
  */
static void SendEnvironmentalData(void)
{
#ifdef PREDMNT1_DEBUG_NOTIFY_TRAMISSION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
     BytesToWrite = sprintf((char *)BufferToWrite,"Sending: ");
     Term_Update(BufferToWrite,BytesToWrite);
  } else {
    PREDMNT1_PRINTF("Sending: ");
  }
#endif /* PREDMNT1_DEBUG_NOTIFY_TRAMISSION */

  if(FirstConnectionConfig)
  {
    FirstConnectionConfig=0;
  }

  /* Pressure,Humidity, and Temperatures*/
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV)) {
    float SensorValue;
    int32_t PressToSend=0;
    uint16_t HumToSend=0;
    int16_t Temp2ToSend=0,Temp1ToSend=0;
    int32_t decPart, intPart;

    if(TargetBoardFeatures.PressSensorIsInit)
    {
      IKS01A2_ENV_SENSOR_GetValue(PRESSURE_INSTANCE,ENV_PRESSURE, &SensorValue);
      MCR_BLUEMS_F2I_2D(SensorValue, intPart, decPart);
      PressToSend=intPart*100+decPart;
    }
#ifdef PREDMNT1_DEBUG_NOTIFY_TRAMISSION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite = sprintf((char *)BufferToWrite,"Press=%ld ",PressToSend);
      Term_Update(BufferToWrite,BytesToWrite);
    } else {
      PREDMNT1_PRINTF("Press=%ld ",PressToSend);
    }
#endif /* PREDMNT1_DEBUG_NOTIFY_TRAMISSION */

    if(TargetBoardFeatures.HumSensorIsInit)
    {
      IKS01A2_ENV_SENSOR_GetValue(HUMIDITY_INSTANCE,ENV_HUMIDITY,&SensorValue);
      MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
      HumToSend = intPart*10+decPart;
    }
#ifdef PREDMNT1_DEBUG_NOTIFY_TRAMISSION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite = sprintf((char *)BufferToWrite,"Hum=%d ",HumToSend);
      Term_Update(BufferToWrite,BytesToWrite);
    } else {
      PREDMNT1_PRINTF("Hum=%d ",HumToSend);
    }
#endif /* PREDMNT1_DEBUG_NOTIFY_TRAMISSION */

    if(TargetBoardFeatures.NumTempSensors==2) {
      if(TargetBoardFeatures.TempSensorsIsInit[0])
      {
        IKS01A2_ENV_SENSOR_GetValue(TEMPERATURE_INSTANCE_1,ENV_TEMPERATURE,&SensorValue);
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        Temp1ToSend = intPart*10+decPart;
      }
#ifdef PREDMNT1_DEBUG_NOTIFY_TRAMISSION
      if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
        BytesToWrite = sprintf((char *)BufferToWrite,"Temp=%d ",Temp1ToSend);
        Term_Update(BufferToWrite,BytesToWrite);
      } else {
        PREDMNT1_PRINTF("Temp=%d ",Temp1ToSend);
      }
#endif /* PREDMNT1_DEBUG_NOTIFY_TRAMISSION */

      if(TargetBoardFeatures.TempSensorsIsInit[1])
      {
        IKS01A2_ENV_SENSOR_GetValue(TEMPERATURE_INSTANCE_2,ENV_TEMPERATURE,&SensorValue);
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        Temp2ToSend = intPart*10+decPart;
      }
#ifdef PREDMNT1_DEBUG_NOTIFY_TRAMISSION
      if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
        BytesToWrite = sprintf((char *)BufferToWrite,"Temp2=%d ",Temp2ToSend);
        Term_Update(BufferToWrite,BytesToWrite);
      } else {
        PREDMNT1_PRINTF("Temp2=%d ",Temp2ToSend);
      }
#endif /* PREDMNT1_DEBUG_NOTIFY_TRAMISSION */
    } else if(TargetBoardFeatures.NumTempSensors==1) {
      if(TargetBoardFeatures.TempSensorsIsInit[0])
      {
        IKS01A2_ENV_SENSOR_GetValue(TEMPERATURE_INSTANCE_1,ENV_TEMPERATURE,&SensorValue);
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        Temp1ToSend = intPart*10+decPart;
      }
#ifdef PREDMNT1_DEBUG_NOTIFY_TRAMISSION
      if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
        BytesToWrite = sprintf((char *)BufferToWrite,"Temp1=%d ",Temp1ToSend);
        Term_Update(BufferToWrite,BytesToWrite);
      } else {
        PREDMNT1_PRINTF("Temp1=%d ",Temp1ToSend);
      }
#endif /* PREDMNT1_DEBUG_NOTIFY_TRAMISSION */
    }
    Environmental_Update(PressToSend,HumToSend,Temp2ToSend,Temp1ToSend);
  }

#ifdef PREDMNT1_DEBUG_NOTIFY_TRAMISSION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
     BytesToWrite = sprintf((char *)BufferToWrite,"\r\n");
     Term_Update(BufferToWrite,BytesToWrite);
  } else {
    PREDMNT1_PRINTF("\r\n");
  }
#endif /* PREDMNT1_DEBUG_NOTIFY_TRAMISSION */
}

/**
  * @brief  CRC init function.
  * @param  None
  * @retval None
  */
static void MX_CRC_Init(void)
{
  hcrc.Instance = CRC;

  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
* @brief  Function for initializing timers for sending the information to BLE:
 *  - 1 for sending MotionFX/AR/CP and Acc/Gyro/Mag
 *  - 1 for sending the Environmental info
 * @param  None
 * @retval None
 */
static void InitTimers(void)
{
  uint32_t uwPrescalerValue;
  
  /* Timer Output Compare Configuration Structure declaration */
  TIM_OC_InitTypeDef sConfig;
  
  /* Compute the prescaler value to have TIM4 counter clock equal to 2 KHz */
  uwPrescalerValue = (uint32_t) (((SystemCoreClock / 2) / 2000) - 1);
  
  /* Set TIM4 instance ( Environmental ) */
  TimEnvHandle.Instance = TIM4;
  /* Initialize TIM4 peripheral */
  TimEnvHandle.Init.Period = ENV_UPDATE_MUL_100MS*200 - 1;
  TimEnvHandle.Init.Prescaler = uwPrescalerValue;
  TimEnvHandle.Init.ClockDivision = 0;
  TimEnvHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&TimEnvHandle) != HAL_OK) {
    /* Initialization Error */
  }

  /* Compute the prescaler value to have TIM1 counter clock equal to 10 KHz */
  uwPrescalerValue = (uint32_t) ((SystemCoreClock / 10000) - 1); 
  
  /* Set TIM1 instance ( Motion ) */
  TimCCHandle.Instance = TIM1;  
  TimCCHandle.Init.Period        = 65535;
  TimCCHandle.Init.Prescaler     = uwPrescalerValue;
  TimCCHandle.Init.ClockDivision = 0;
  TimCCHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;
  if(HAL_TIM_OC_Init(&TimCCHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
 /* Configure the Output Compare channels */
 /* Common configuration for all channels */
  sConfig.OCMode     = TIM_OCMODE_TOGGLE;
  sConfig.OCPolarity = TIM_OCPOLARITY_LOW;
  
  /* Output Compare Toggle Mode configuration: Channel4 */
  sConfig.Pulse = DEFAULT_uhCCR4_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_4) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  
  /* Compute the prescaler value to have TIM5 counter clock equal to 10 KHz */
  uwPrescalerValue = (uint32_t) (((SystemCoreClock / 2) / 10000) - 1);
  
  /* Set TIM5 instance ( Mic ) */
  TimAudioDataHandle.Instance = TIM5;
  TimAudioDataHandle.Init.Period = MICS_DB_UPDATE_MUL_10MS*100 - 1;
  TimAudioDataHandle.Init.Prescaler = uwPrescalerValue;
  TimAudioDataHandle.Init.ClockDivision = 0;
  TimAudioDataHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&TimAudioDataHandle) != HAL_OK) {
    /* Initialization Error */
    Error_Handler();
  }
}

/** @brief Initialize the BlueNRG Stack
 * @param None
 * @retval None
 */
static void Init_BlueNRG_Stack(void)
{
  //const char BoardName[8] = {NAME_BLUEMS,0};
  char BoardName[8];
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  int ret;
  uint8_t  hwVersion;
  uint16_t fwVersion;
  
    for(int i=0; i<7; i++)
      BoardName[i]= NodeName[i+1];
    
    BoardName[7]= 0;
  
#ifdef MAC_PREDMNT1
  {
    uint8_t tmp_bdaddr[6]= {MAC_PREDMNT1};
    int32_t i;
    for(i=0;i<6;i++)
      bdaddr[i] = tmp_bdaddr[i];
  }
#endif /* MAC_PREDMNT1 */

  /* Initialize the BlueNRG SPI driver */
  hci_init(HCI_Event_CB, NULL);

  /* get the BlueNRG HW and FW versions */
  getBlueNRGVersion(&hwVersion, &fwVersion);

  if (hwVersion > 0x30) {
    /* X-NUCLEO-IDB05A1 expansion board is used */
    TargetBoardFeatures.bnrg_expansion_board = IDB05A1;
  } else {
    /* X-NUCLEO-IDB0041 expansion board is used */
    TargetBoardFeatures.bnrg_expansion_board = IDB04A1;
  }
  
  /* 
   * Reset BlueNRG again otherwise we won't
   * be able to change its MAC address.
   * aci_hal_write_config_data() must be the first
   * command after reset otherwise it will fail.
   */
  hci_reset();

#ifndef MAC_PREDMNT1
  #ifdef MAC_STM32UID_PREDMNT1
  /* Create a Unique BLE MAC Related to STM32 UID */
  {
    bdaddr[0] = (STM32_UUID[1]>>24)&0xFF;
    bdaddr[1] = (STM32_UUID[0]    )&0xFF;
    bdaddr[2] = (STM32_UUID[2] >>8)&0xFF;
    bdaddr[3] = (STM32_UUID[0]>>16)&0xFF;

    /* if IDB05A1 = Number between 100->199
     * if IDB04A1 = Number between 0->99
     * where Y == (PREDMNT1_VERSION_MAJOR + PREDMNT1_VERSION_MINOR)&0xF */
    bdaddr[4] = (hwVersion > 0x30) ?
         ((((PREDMNT1_VERSION_MAJOR-48)*10) + (PREDMNT1_VERSION_MINOR-48)+100)&0xFF) :
         ((((PREDMNT1_VERSION_MAJOR-48)*10) + (PREDMNT1_VERSION_MINOR-48)    )&0xFF) ;
    bdaddr[5] = 0xC0; /* for a Legal BLE Random MAC */
  }
  #else /* MAC_STM32UID_PREDMNT1 */
  {
    /* we will let the BLE chip to use its Random MAC address */
    uint8_t data_len_out;
    ret = aci_hal_read_config_data(CONFIG_DATA_RANDOM_ADDRESS, 6, &data_len_out, bdaddr);

    if(ret){
      PREDMNT1_PRINTF("\r\nReading  Random BD_ADDR failed\r\n");
      goto fail;
    }
  }
  #endif /* MAC_STM32UID_PREDMNT1 */
#else /* MAC_PREDMNT1 */
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  bdaddr);

  if(ret){
     PREDMNT1_PRINTF("\r\nSetting Pubblic BD_ADDR failed\r\n");
     goto fail;
  }
#endif /* MAC_PREDMNT1 */

  ret = aci_gatt_init();    
  if(ret){
     PREDMNT1_PRINTF("\r\nGATT_Init failed\r\n");
     goto fail;
  }

  if (TargetBoardFeatures.bnrg_expansion_board == IDB05A1) {
    ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  }else {
    ret = aci_gap_init_IDB04A1(GAP_PERIPHERAL_ROLE_IDB04A1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  }

  if(ret != BLE_STATUS_SUCCESS){
     PREDMNT1_PRINTF("\r\nGAP_Init failed\r\n");
     goto fail;
  }

#ifndef  MAC_PREDMNT1
  #ifdef MAC_STM32UID_PREDMNT1
    ret = hci_le_set_random_address(bdaddr);

    if(ret){
       PREDMNT1_PRINTF("\r\nSetting the Static Random BD_ADDR failed\r\n");
       goto fail;
    }
  #endif /* MAC_STM32UID_PREDMNT1 */
#endif /* MAC_PREDMNT1 */

  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                   7/*strlen(BoardName)*/, (uint8_t *)BoardName);

  if(ret){
     PREDMNT1_PRINTF("\r\naci_gatt_update_char_value failed\r\n");
    while(1);
  }

  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL, 7, 16,
                                     USE_FIXED_PIN_FOR_PAIRING, 123456,
                                     BONDING);
  if (ret != BLE_STATUS_SUCCESS) {
     PREDMNT1_PRINTF("\r\nGAP setting Authentication failed\r\n");
     goto fail;
  }

  PREDMNT1_PRINTF("SERVER: BLE Stack Initialized \r\n"
         "\t\tBoard type=%s HWver=%d, FWver=%d.%d.%c\r\n"
         "\t\tBoardName= %s\r\n"
         "\t\tBoardMAC = %x:%x:%x:%x:%x:%x\r\n\n",
         (TargetBoardFeatures.bnrg_expansion_board==IDB05A1) ? "IDB05A1" : "IDB04A1",
         hwVersion,
         fwVersion>>8,
         (fwVersion>>4)&0xF,
         (hwVersion > 0x30) ? ('a'+(fwVersion&0xF)-1) : 'a',
         BoardName,
         bdaddr[5],bdaddr[4],bdaddr[3],bdaddr[2],bdaddr[1],bdaddr[0]);

  /* Set output power level */
  aci_hal_set_tx_power_level(1,4);

  return;

fail:
  return;
}

/** @brief Initialize all the Custom BlueNRG services
  * @param None
  * @retval None
  */
static void Init_BlueNRG_Custom_Services(void)
{
  int ret;
  
  ret = Add_HW_ServW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS)
  {
     PREDMNT1_PRINTF("HW      Service W2ST added successfully\r\n");
  }
  else
  {
     PREDMNT1_PRINTF("\r\nError while adding HW & SW Service W2ST\r\n");
  }
  
  ret = Add_SW_ServW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS)
  {
     PREDMNT1_PRINTF("SW      Service W2ST added successfully\r\n");
  } else
  {
     PREDMNT1_PRINTF("\r\nError while adding SW Service W2ST\r\n");
  }

  ret = Add_ConsoleW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS)
  {
     PREDMNT1_PRINTF("Console Service W2ST added successfully\r\n");
  }
  else
  {
     PREDMNT1_PRINTF("\r\nError while adding Console Service W2ST\r\n");
  }

  ret = Add_ConfigW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS)
  {
     PREDMNT1_PRINTF("Config  Service W2ST added successfully\r\n");
  }
  else
  {
     PREDMNT1_PRINTF("\r\nError while adding Config Service W2ST\r\n");
  }
}

/** @brief Predictive Maintenance Initialization
  * @param None
  * @retval None
  */
static void InitPredictiveMaintenance(void)
{
  /* Set the vibration parameters with default values */
  MotionSP_SetDefaultVibrationParam();
  
  /* Read Vibration Parameters From Memory */
  ReCallVibrationParamFromMemory();
  
  PREDMNT1_PRINTF("\r\nAccelerometer parameters:\r\n");
  PREDMNT1_PRINTF("AccOdr= %d\t", Accelerometer_Parameters.AccOdr);
  PREDMNT1_PRINTF("FifoOdr= %d\t", Accelerometer_Parameters.FifoOdr);   
  PREDMNT1_PRINTF("fs= %d\t", Accelerometer_Parameters.fs);   
  PREDMNT1_PRINTF("\r\n");

  PREDMNT1_PRINTF("\r\nMotionSP parameters:\r\n");
  PREDMNT1_PRINTF("size= %d\t", MotionSP_Parameters.size); 
  PREDMNT1_PRINTF("wind= %d\t", MotionSP_Parameters.window);  
  PREDMNT1_PRINTF("tacq= %d\t", MotionSP_Parameters.tacq);
  PREDMNT1_PRINTF("ovl= %d\t", MotionSP_Parameters.ovl);
  PREDMNT1_PRINTF("subrange_num= %d\t", MotionSP_Parameters.subrange_num);
  PREDMNT1_PRINTF("\r\n\n");
  
  PREDMNT1_PRINTF("************************************************************************\r\n\r\n");
  
  /* Initializes accelerometer with vibration parameters values */
  if(SetAccelerometerParameters()) {
    PREDMNT1_PRINTF("\tOK Set Accelerometer Parameters\r\n\n");
  } else {
    PREDMNT1_PRINTF("\tFailed Set Accelerometer Parameters\r\n\n");
  }
  
  IsFirstTime = 1;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            PLL_R                          = 2
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
  
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
  clocked below the maximum system frequency, to update the voltage scaling value 
  regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336; //192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;//4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  /* Activate the OverDrive to reach the 180 MHz Frequency */  
  //HAL_PWREx_EnableOverDrive();
  
  /*Select Main PLL output as USB clock source */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CK48CLKSOURCE_PLLQ;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

/**
 * @brief  Check if there are a valid Node Name Values in Memory and read them
 * @param  None
 * @retval unsigned char Success/Not Success
 */
static unsigned char ReCallNodeNameFromMemory(void)
{
  const char DefaultBoardName[7] = {NAME_BLUEMS};
  
  /* ReLoad the Node Name Values from RAM */
  unsigned char Success=0;

  /* Recall the node name Credential saved */
  MDM_ReCallGMD(GMD_NODE_NAME,(void *)&NodeName);
  
  if(NodeName[0] != 0x12)
  {
    NodeName[0]= 0x12;
    
    for(int i=0; i<7; i++)
      NodeName[i+1]= DefaultBoardName[i];
    
    MDM_SaveGMD(GMD_NODE_NAME,(void *)&NodeName);
    NecessityToSaveMetaDataManager=1;
  }

  return Success;
}

/**
 * @brief  Check if there are a valid Vibration Parameters Values in Memory and read them
 * @param pAccelerometer_Parameters Pointer to Accelerometer parameter structure
 * @param pMotionSP_Parameters Pointer to Board parameter structure
 * @retval unsigned char Success/Not Success
 */
static unsigned char ReCallVibrationParamFromMemory(void)
{
  /* ReLoad the Vibration Parameters Values from RAM */
  unsigned char Success=0;
  
  PREDMNT1_PRINTF("Recall the vibration parameter values from FLASH\r\n");

  /* Recall the Vibration Parameters Values saved */
  MDM_ReCallGMD(GMD_VIBRATION_PARAM,(void *)VibrationParam);
  
  if(VibrationParam[0] == CHECK_VIBRATION_PARAM)
  {
    Accelerometer_Parameters.AccOdr=    VibrationParam[1];
    Accelerometer_Parameters.FifoOdr=   VibrationParam[2];
    Accelerometer_Parameters.fs=        VibrationParam[3];
    MotionSP_Parameters.size=           VibrationParam[4];
    MotionSP_Parameters.tau=            VibrationParam[5];
    MotionSP_Parameters.window=         VibrationParam[6];
    MotionSP_Parameters.td_type=        VibrationParam[7];
    MotionSP_Parameters.tacq=           VibrationParam[8];
    MotionSP_Parameters.ovl=            VibrationParam[9];
    MotionSP_Parameters.subrange_num=   VibrationParam[10];
    
    PREDMNT1_PRINTF("Vibration parameter values read from FLASH\r\n");
    
    NecessityToSaveMetaDataManager=0;
  }
  else
  {
    PREDMNT1_PRINTF("Vibration parameters values not present in FLASH\r\n");
    SaveVibrationParamToMemory();
  }

  return Success;
}

/**
  * @}
  */

/** @defgroup PREDCTIVE_MAINTENANCE_MAIN_CALLBACK_FUNCTIONS Predictive Maintenance Main CallBack Functions
  * @{
  */

/**
  * @brief  Output Compare callback in non blocking mode 
  * @param  htim : TIM OC handle
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t uhCapture=0;

  /* TIM1_CH4 toggling with frequency = 20 Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
  {
     uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_4, (uhCapture + uhCCR4_Val));
    SendAccGyroMag=1;
  }
}

/**
  * @brief  Period elapsed callback in non blocking mode for Environmental timer
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == (&TimEnvHandle)) {
    /* Environmental */
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV))
      SendEnv=1;
  } else if(htim == (&TimAudioDataHandle)) {
    /* Mic Data */
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL))
      SendAudioLevel=1;
  }
}

/**
* @brief  Half Transfer user callback, called by BSP functions.
* @param  None
* @retval None
*/
void BSP_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance)
{
  AudioProcess();
}

/**
* @brief  Transfer Complete user callback, called by BSP functions.
* @param  None
* @retval None
*/
void BSP_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance)
{
  AudioProcess();
}

/**
 * @brief  EXTI line detection callback.
 * @param  uint16_t GPIO_Pin Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{  
  switch(GPIO_Pin){
  case SPI1_CMN_DEFAULT_IRQ_PIN:
    hci_tl_lowlevel_isr();
    HCI_ProcessEvent=1;
    break;
#ifdef IKS01A2_LSM6DSL_0
  case GPIO_PIN_5:
#endif /* IKS01A2_LSM6DSL_0 */
#ifdef IKS01A2_ISM330DLC_0
  case M_INT2_O_PIN:
#endif /* IKS01A2_ISM330DLC_0 */
    AccIntReceived = 1;
    if(FifoEnabled)
      FuncOn_FifoFull();
    else 
      FuncOn_DRDY_XL();
    break;
  case KEY_BUTTON_PIN:
    ButtonPressed = 1;
    break;
  }
}

/**
  * @}
  */

/** @defgroup PREDCTIVE_MAINTENANCE_MAIN_EXPORTED_FUNCTIONS Predictive Maintenance Main Exported Functions
  * @{
  */

/**
  * @brief This function provides accurate delay (in milliseconds) based 
  *        on variable incremented.
  * @note This is a user implementation using WFI state
  * @param Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */
void HAL_Delay(__IO uint32_t Delay)
{
  uint32_t tickstart = 0;
  tickstart = HAL_GetTick();
  while((HAL_GetTick() - tickstart) < Delay){
    __WFI();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1){
  }
}

/**
  *  @brief   Get MCU Identifiers
  *  @details See "MCU device ID code" paragraph into the MCU Reference Manual
  *  @param   pMcuId Pointer to the MCU ID structure to be filled
  *  @return  None
  */
void Get_McuId(sMcuId_t *pMcuId)
{
#define DEV_ID_MASK       (uint32_t)0x00000FFF
#define REV_ID_MASK       (uint32_t)0xFFFF0000
#define DEV_ID_BIT        0
#define REV_ID_BIT        16

#define UNQ_DEV_ID_BASE   0x1FFF7A10U //!< Unique device ID register (96 bits) base address
#define FLASH_SIZE_BASE   0x1FFF7A22U //!< Flash size register base address
#define PACKAGE_DATA_BASE 0x1FFF7BF0U //!< Package data register base address
  
  if (pMcuId != NULL)
  {
    /* Read the device Id code */
    pMcuId->McuDevId = (uint16_t)(((uint32_t)(DBGMCU->IDCODE) & DEV_ID_MASK) >> DEV_ID_BIT);
    pMcuId->McuRevId = (uint16_t)(((uint32_t)(DBGMCU->IDCODE) & REV_ID_MASK) >> REV_ID_BIT);
    
    /* Read the unique device ID registers (96 bits) */
    pMcuId->u_id_31_0 = *((uint32_t *)(UNQ_DEV_ID_BASE+0x00));
    pMcuId->u_id_63_32 = *((uint32_t *)(UNQ_DEV_ID_BASE+0x04));
    pMcuId->u_id_95_64 = *((uint32_t *)(UNQ_DEV_ID_BASE+0x08));
    
    /* Read the size of the device Flash memory expressed in Kbytes */
    pMcuId->FlashSize = *((uint16_t *)(FLASH_SIZE_BASE));

    /* Read the package data */
    pMcuId->Package = (uint8_t)(((*((uint16_t *)(PACKAGE_DATA_BASE)))>>8)&0x07);
  }
}

/**
 * @brief  Save vibration parameters values to memory
 * @param pAccelerometer_Parameters Pointer to Accelerometer parameter structure
 * @param pMotionSP_Parameters Pointer to Board parameter structure
 * @retval unsigned char Success/Not Success
 */
unsigned char SaveVibrationParamToMemory(void)
{
  /* ReLoad the Vibration Parameters Values from RAM */
  unsigned char Success=0;

  VibrationParam[0]= CHECK_VIBRATION_PARAM;
  VibrationParam[1]=  (uint16_t)Accelerometer_Parameters.AccOdr;
  VibrationParam[2]=  (uint16_t)Accelerometer_Parameters.FifoOdr;
  VibrationParam[3]=  (uint16_t)Accelerometer_Parameters.fs;
  VibrationParam[4]=  (uint16_t)MotionSP_Parameters.size;
  VibrationParam[5]=  (uint16_t)MotionSP_Parameters.tau;
  VibrationParam[6]=  (uint16_t)MotionSP_Parameters.window;
  VibrationParam[7]=  (uint16_t)MotionSP_Parameters.td_type;
  VibrationParam[8]=  (uint16_t)MotionSP_Parameters.tacq;
  VibrationParam[9]=  (uint16_t)MotionSP_Parameters.ovl;
  VibrationParam[10]= (uint16_t)MotionSP_Parameters.subrange_num;
  
  PREDMNT1_PRINTF("Vibration parameters values will be saved in FLASH\r\n");
  MDM_SaveGMD(GMD_VIBRATION_PARAM,(void *)VibrationParam);
  NecessityToSaveMetaDataManager=1;

  return Success;
}

/**
  * @}
  */

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: PREDMNT1_PRINTF("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1){
  }
}
#endif

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




/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/

/**
  ******************** (C) COPYRIGHT 2019 STMicroelectronics *******************
  * @file    readme.txt
  * @author  Central LAB
  * @version V1.0.0
  * @date    08-Feb-2019
  * @brief   Description of the Application FW.
  ******************************************************************************
  * Attention
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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

Application Description 

 This firmware package includes Components Device Drivers, Board Support Package
 and example application for the following STMicroelectronics elements:
 - X-NUCLEO-IDB05A1 Bluetooth Low energy expansion boards
 - X-NUCLEO-IKS01A2 Expansion board for four MEMS sensor devices:
       HTS221, LPS22HB, LSM6DSL, LSM303AGR
	   DIL24 with ISM330DLC
 - X-NUCLEO-CCA02M1 Digital MEMS microphones expansion board
 - NUCLEO-F446RE NUCLEO boards
 - MotionSP software provides real-time vibration analysis in time domain and frequency domain.

 The Example application initializes all the Components and Library creating 4 Custom Bluetooth services:
 - The first service exposes all the HW characteristics related to MEMS sensor devices: Temperature, Humidity, Pressure, Magnetometer, 
   Gyroscope, Accelleromenter and Microphones Signal Noise dB level.
 - The second service exposes the SW characteristic: the Frequency Domain Processing using the MotionSP algorithm.
 - The third Service exposes the console services where we have stdin/stdout and stderr capabilities
 - The last Service is used for configuration purpose
 
 For NUCLEO boards the example application allows the user to control the initialization phase via UART.
 Launch a terminal application and set the UART port to 115200 bps, 8 bit, No Parity, 1 stop bit.
 
 This example must be used with the related BlueMS Android/iOS application available on Play/itune store (Version 4.1.0 or higher),
 in order to read the sent information by Bluetooth Low Energy protocol
 
                   ---------------------------------------------
                   | Important Hardware Additional Information |
			       ---------------------------------------------
 Before to connect X-NUCLEO-IKS01A2 with X-NUCLEO-CCAM02M1 expansion board through the Arduino UNO R3 extension connector,
 on to X-NUCLEO-IKS01A2 board remove these 0-ohm resistor: SB25, SB26 and SB27

                              --------------------
                              | VERY IMPORTANT : |
                              --------------------
 1) This example support the Firmware-Over-The-Air (FOTA) update using the BlueMS Android/iOS application (Version 3.0.0 and above)
 2) This example must run starting at address 0x08004000 in memory and works ONLY if the BootLoader 
    is saved at the beginning of the FLASH (address 0x08000000)
 3) For each IDE (IAR/µVision/System Workbench) and for NUCLEO-F446RE platform,
    there are some scripts *.bat and *.sh that makes the following operations:
     - Full Flash Erase
     - Load the BootLoader on the rigth flash region
     - Load the Program (after the compilation) on the rigth flash region (This could be used for a FOTA)
     - Dump back one single binary that contain BootLoader+Program that could be
       flashed at the flash beginning (address 0x08000000) (This COULD BE NOT used for FOTA)
     - Reset the board
                                   ----------
                                   | ISSUE: |
                                   ----------
 - A compiler warning is generated in IAR IDE from ism330dlc reg for typedef name redeclared (conflict with arm math library).
   It doesn't affect library performances.

                              ----------------------
                              | Known Limitations: |
                              ----------------------
 - ODR frequency limitation for Nucleo-F446RE (maximum value is 1660 Hz without optimize code)
 - MotionSP API do not available with lsm6dsl component  


 Inside the Binary Directory there are the following binaries:
Binary/
+-- STM32F446RE-Nucleo
¦   +-- NUCLEO-F446RE_ConditionMonitoring_v1.0.0.bin    (Program without BootLoader. COULD BE USED     for FOTA)
¦   +-- NUCLEO-F446RE_ConditionMonitoring_v1.0.0_BL.bin (Program with BootLoader.    COULD NOT BE USED for FOTA)

@par Hardware and Software environment

  - This example runs on Sensor expansion board attached to STM32F446RE devices
    can be easily tailored to any other supported device and development board.
    
  - This example must be used with the related BlueMS Android/iOS application (Version 4.1.0 or higher) available on Play/itune store,
    in order to read the sent information by Bluetooth Low Energy protocol
    
@par STM32Cube packages:
  - STM32F4xx drivers from STM32CubeF4 V1.21.0
@par X-CUBE packages:
  - X-CUBE-BLE1 V4.2.0
  - X-CUBE-MEMS1 V5.2.1
  - X-CUBE-MEMSMIC1 V4.0.0

@par How to use it ? 

This package contains projects for 3 IDEs viz. IAR, µVision and System Workbench. 
In order to make the  program work, you must do the following:
 - WARNING: before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.

For IAR:
 - Open IAR toolchain (this firmware has been successfully tested with Embedded Workbench V8.20.2).
 - Open the IAR project file STM32F446RE-Nucleo\Demonstrations\Acoustic_Analysis\EWARM\AcousticAnalysis.eww.
 - Rebuild all files and run CleanPREDMNT1_IAR_F446.bat script that you find on the same directory.

For µVision:
 - Open µVision toolchain (this firmware has been successfully tested with MDK-ARM Professional Version: 5.24.2)
 - Open the µVision project file STM32F446RE-Nucleo\Demonstrations\Acoustic_Analysis\Project.uvprojx.
 - Rebuild all files and run CleanPREDMNT1_MDK_ARM_F446.bat script that you find on the same directory.
		
For System Workbench:
 - Open System Workbench for STM32 (this firmware has been successfully tested with System Workbench for STM32 Version 2.7.2.201812190825)
 - Set the default workspace proposed by the IDE (please be sure that there are not spaces in the workspace path).
 - Press "File" -> "Import" -> "Existing Projects into Workspace"; press "Browse" in the "Select root directory" and choose the path where the System
   Workbench project is located (it should be STM32F446RE-Nucleo\Demonstrations\Acoustic_Analysis\SW4STM32\STM32F446RE-Nucleo\. 
 - Rebuild all files and and run these script that you find on the same directory:
   - if you are on windows and you had installed the STM32 ST-Link utility:
		- For Nucleo F446: CleanPREDMNT1_SW4STM32_F446.bat
   - Otherwise (Linux/iOS or Windows without the STM32 ST-Link Utility):
		- For Nucleo F446: CleanPREDMNT1_SW4STM32_F446.sh
		
 /******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/

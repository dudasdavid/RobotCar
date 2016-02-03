/**
  ******************************************************************************
  * @file    stm32f3_discovery_accelerometer.c
  * @author  MCD Application Team
  * @version V2.1.2
  * @date    13-November-2015
  * @brief   This file provides a set of functions needed to manage the ACCELEROMETER
  *          MEMS available on STM32F3-Discovery Kit.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
#include "stm32f3_discovery_accelerometer.h"

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup STM32F3_DISCOVERY
  * @{
  */ 

/** @addtogroup STM32F3_DISCOVERY_ACCELEROMETER
  * @{
  */


/** @defgroup STM32F3_DISCOVERY_ACCELERO_Private_TypesDefinitions STM32F3_DISCOVERY_ACCELERO_Private_TypesDefinitions
  * @{
  */
  
/**
  * @}
  */

/** @defgroup STM32F3_DISCOVERY_ACCELERO_Private_Defines STM32F3_DISCOVERY_ACCELERO_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F3_DISCOVERY_ACCELERO_Private_Macros STM32F3_DISCOVERY_ACCELERO_Private_Macros
  * @{
  */

/**
  * @}
  */ 
  
/** @defgroup STM32F3_DISCOVERY_ACCELERO_Private_Variables STM32F3_DISCOVERY_ACCELERO_Private_Variables
  * @{
  */ 
static volatile int16_t rawTemp;
static ACCELERO_DrvTypeDef *AccelerometerDrv;

/**
  * @}
  */

/** @defgroup STM32F3_DISCOVERY_ACCELERO_Private_FunctionPrototypes STM32F3_DISCOVERY_ACCELERO_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32F3_DISCOVERY_ACCELERO_Private_Functions STM32F3_DISCOVERY_ACCELERO_Private_Functions
  * @{
  */

/**
  * @brief  Set LSM303DLHC Mag Initialization.
  * @param  LSM303DLHC_InitStruct: pointer to a LSM303DLHC_MagInitTypeDef structure 
  *         that contains the configuration setting for the LSM303DLHC.
  * @retval None
  */
void LSM303DLHC_MagInit(LACCELERO_InitTypeDef *LSM303DLHC_InitStruct)
{  
  uint8_t cra_regm = 0x00, crb_regm = 0x00, mr_regm = 0x00;
  
  /* Configure the low level interface ---------------------------------------*/
  //LSM303DLHC_LowLevel_Init();
  
  /* Configure MEMS: temp and Data rate */
  cra_regm |= (uint8_t) (LSM303DLHC_InitStruct->Temperature_Sensor | LSM303DLHC_InitStruct->MagOutput_DataRate);
    
  /* Configure MEMS: full Scale */
  crb_regm |= (uint8_t) (LSM303DLHC_InitStruct->MagFull_Scale);
      
  /* Configure MEMS: working mode */
  mr_regm |= (uint8_t) (LSM303DLHC_InitStruct->Working_Mode);
                    
  /* Write value to Mag MEMS CRA_REG regsister */
  COMPASSACCELERO_IO_Write(MAG_I2C_ADDRESS, LSM303DLHC_CRA_REG_M, cra_regm);
  
  /* Write value to Mag MEMS CRB_REG regsister */
  COMPASSACCELERO_IO_Write(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, crb_regm);

  /* Write value to Mag MEMS MR_REG regsister */
  COMPASSACCELERO_IO_Write(MAG_I2C_ADDRESS, LSM303DLHC_MR_REG_M, mr_regm);
}

void MAGNET_Init(void)
{
  LACCELERO_InitTypeDef LSM303DLHC_MAG_InitStructure;
  
  /* Configure MEMS magnetometer main parameters: temp, working mode, full Scale and Data rate */
  LSM303DLHC_MAG_InitStructure.Temperature_Sensor = LSM303DLHC_TEMPSENSOR_ENABLE;
  LSM303DLHC_MAG_InitStructure.MagOutput_DataRate =LSM303DLHC_ODR_30_HZ ;
  LSM303DLHC_MAG_InitStructure.MagFull_Scale = LSM303DLHC_FS_4_0_GA;
  LSM303DLHC_MAG_InitStructure.Working_Mode = LSM303DLHC_CONTINUOS_CONVERSION;
  LSM303DLHC_MagInit(&LSM303DLHC_MAG_InitStructure);
}

/**
  * @brief  calculate the magnetic field Magn.
* @param  pfData: pointer to the data out
  * @retval None
  */
void LSM303DLHC_MagReadXYZ (float* pfData)
{
  static uint8_t buffer[6] = {0};
  uint8_t CTRLB = 0;
  uint16_t Magn_Sensitivity_XY = 0, Magn_Sensitivity_Z = 0;
  uint8_t i =0;
  
  //buffer[0] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A); 
  //buffer[1] = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_H_A);
  
  //LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, &CTRLB, 1);
  CTRLB = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M);
  
  //LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M, buffer, 1);
  //LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_L_M, buffer+1, 1);
  //LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_H_M, buffer+2, 1);
  //LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_L_M, buffer+3, 1);
  //LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_H_M, buffer+4, 1);
  //LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_L_M, buffer+5, 1);
  /* Read output register X, Y & Z magnetic value */
  buffer[0] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_L_M); 
  buffer[1] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M);
  buffer[2] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_L_M);
  buffer[3] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_H_M);
  buffer[4] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_L_M);
  buffer[5] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_H_M);
  
  /* Switch the sensitivity set in the CRTLB*/
  switch(CTRLB & 0xE0)
  {
  case LSM303DLHC_FS_1_3_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_3Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_3Ga;
    break;
  case LSM303DLHC_FS_1_9_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_9Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_9Ga;
    break;
  case LSM303DLHC_FS_2_5_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_2_5Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_2_5Ga;
    break;
  case LSM303DLHC_FS_4_0_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4Ga;
    break;
  case LSM303DLHC_FS_4_7_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4_7Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4_7Ga;
    break;
  case LSM303DLHC_FS_5_6_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_5_6Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_5_6Ga;
    break;
  case LSM303DLHC_FS_8_1_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_8_1Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_8_1Ga;
    break;
  }
  
  for(i=0; i<2; i++)
  {
    pfData[i]=(float)((int16_t)(((uint16_t)buffer[2*i+1] << 8) + buffer[2*i]))/Magn_Sensitivity_XY*1000;
  }
  pfData[2]=(float)((int16_t)(((uint16_t)buffer[5] << 8) + buffer[4]))/Magn_Sensitivity_Z*1000;
}

void LSM303DLHC_MagReadTemp (float* pfData)
{
  //static int16_t tempC;
  //static int16_t rawTemp;
  static uint8_t buffer[2] = {0};
  buffer[0] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303DLHC_TEMP_OUT_L_M); 
  buffer[1] = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303DLHC_TEMP_OUT_H_M);

  rawTemp=(((uint16_t)buffer[1] << 8) + buffer[0]);
  
  //rawTemp = (((buffer[1] << 8) + buffer[0]) >> 4); //temperature raw
  //tempC = rawTemp / 8 / 8 + 25; //temperature in celsuis + offset
  
  pfData[0] = rawTemp / 8.0 / 8.0;
  //pData[0] = (int16_t)((buffer[1] << 4) + (buffer[0] >> 4));
}

/**
  * @brief  Set ACCELEROMETER Initialization.
  * @retval ACCELERO_OK if no problem during initialization
  */
uint8_t BSP_ACCELERO_Init(void)
{  
  uint8_t ret = ACCELERO_ERROR;
  static uint16_t ctrl = 0x0000;
  ACCELERO_InitTypeDef LSM303DLHC_InitStructure;
  ACCELERO_FilterConfigTypeDef LSM303DLHC_FilterStructure;
 
  if(Lsm303dlhcDrv.ReadID() == I_AM_LMS303DLHC)
  {
    /* Initialize the gyroscope driver structure */
    AccelerometerDrv = &Lsm303dlhcDrv;
  
  /* MEMS configuration ------------------------------------------------------*/
   /* Fill the accelerometer structure */
    LSM303DLHC_InitStructure.Power_Mode = LSM303DLHC_NORMAL_MODE;
    LSM303DLHC_InitStructure.AccOutput_DataRate = LSM303DLHC_ODR_50_HZ;
    LSM303DLHC_InitStructure.Axes_Enable= LSM303DLHC_AXES_ENABLE;
    LSM303DLHC_InitStructure.AccFull_Scale = LSM303DLHC_FULLSCALE_2G;
    LSM303DLHC_InitStructure.BlockData_Update = LSM303DLHC_BlockUpdate_Continous;
    LSM303DLHC_InitStructure.Endianness=LSM303DLHC_BLE_LSB;
    LSM303DLHC_InitStructure.High_Resolution=LSM303DLHC_HR_ENABLE;
    
    /* Configure MEMS: data rate, power mode, full scale and axes */
    ctrl |= (LSM303DLHC_InitStructure.Power_Mode | LSM303DLHC_InitStructure.AccOutput_DataRate | \
                       LSM303DLHC_InitStructure.Axes_Enable);
    
    ctrl |= ((LSM303DLHC_InitStructure.BlockData_Update | LSM303DLHC_InitStructure.Endianness | \
                      LSM303DLHC_InitStructure.AccFull_Scale | LSM303DLHC_InitStructure.High_Resolution) << 8);
    
  /* Configure the accelerometer main parameters */
    AccelerometerDrv->Init(ctrl);
  
  /* Fill the accelerometer LPF structure */
    LSM303DLHC_FilterStructure.HighPassFilter_Mode_Selection =LSM303DLHC_HPM_NORMAL_MODE;
    LSM303DLHC_FilterStructure.HighPassFilter_CutOff_Frequency = LSM303DLHC_HPFCF_16;
    LSM303DLHC_FilterStructure.HighPassFilter_AOI1 = LSM303DLHC_HPF_AOI1_DISABLE;
    LSM303DLHC_FilterStructure.HighPassFilter_AOI2 = LSM303DLHC_HPF_AOI2_DISABLE;
    
    /* Configure MEMS: mode, cutoff frquency, Filter status, Click, AOI1 and AOI2 */
    ctrl = (uint8_t) (LSM303DLHC_FilterStructure.HighPassFilter_Mode_Selection |\
                      LSM303DLHC_FilterStructure.HighPassFilter_CutOff_Frequency|\
                      LSM303DLHC_FilterStructure.HighPassFilter_AOI1|\
                      LSM303DLHC_FilterStructure.HighPassFilter_AOI2);

  /* Configure the accelerometer LPF main parameters */
    AccelerometerDrv->FilterConfig(ctrl);

    ret = ACCELERO_OK;
  }  
  else
  {
    ret = ACCELERO_ERROR;
  }

  return ret;
}

/**
  * @brief  Reboot memory content of ACCELEROMETER      
  * @retval None
  */
void BSP_ACCELERO_Reset(void)
  {
  if(AccelerometerDrv->Reset != NULL)
{
    AccelerometerDrv->Reset();
}
}

/**
  * @brief  Get XYZ acceleration
  * @param pDataXYZ Pointeur on 3 angular accelerations 
  *                 pDataXYZ[0] = X axis, pDataXYZ[1] = Y axis, pDataXYZ[2] = Z axis
* @retval None
*/
void BSP_ACCELERO_GetXYZ(float *pfDataXYZ)
{
  if(AccelerometerDrv->GetXYZ!= NULL)
  {   
    AccelerometerDrv->GetXYZ(pfDataXYZ);
  }
}

void CompassLED(float HeadingValue) {
  if (((HeadingValue < 25.0f)&&(HeadingValue >= 0.0f))||((HeadingValue >=340.0f)&&(HeadingValue <= 360.0f))) {
    BSP_LED_On(LED3);
    BSP_LED_Off(LED10);
    BSP_LED_Off(LED6);
    BSP_LED_Off(LED7);
    BSP_LED_Off(LED4);
    BSP_LED_Off(LED8);
    BSP_LED_Off(LED9);
    BSP_LED_Off(LED5);
  }
  else  if ((HeadingValue <70.0f)&&(HeadingValue >= 25.0f)) {
    BSP_LED_On(LED5);
    BSP_LED_Off(LED6);
    BSP_LED_Off(LED10);
    BSP_LED_Off(LED3);
    BSP_LED_Off(LED8);
    BSP_LED_Off(LED9);
    BSP_LED_Off(LED4);
    BSP_LED_Off(LED7);
  } 
  else  if ((HeadingValue < 115.0f)&&(HeadingValue >= 70.0f)) {
    BSP_LED_On(LED7);
    BSP_LED_Off(LED3);
    BSP_LED_Off(LED4);
    BSP_LED_Off(LED9);
    BSP_LED_Off(LED10);
    BSP_LED_Off(LED8);
    BSP_LED_Off(LED6);
    BSP_LED_Off(LED5);
  }
  else  if ((HeadingValue <160.0f)&&(HeadingValue >= 115.0f)) {
    BSP_LED_On(LED9);
    BSP_LED_Off(LED6);
    BSP_LED_Off(LED10);
    BSP_LED_Off(LED8);
    BSP_LED_Off(LED5);
    BSP_LED_Off(LED7);
    BSP_LED_Off(LED4);
    BSP_LED_Off(LED3);
  } 
  else  if ((HeadingValue <205.0f)&&(HeadingValue >= 160.0f)) {
    BSP_LED_On(LED10);
    BSP_LED_Off(LED6);
    BSP_LED_Off(LED4);
    BSP_LED_Off(LED8);
    BSP_LED_Off(LED9);
    BSP_LED_Off(LED5);
    BSP_LED_Off(LED3);
    BSP_LED_Off(LED7);
  } 
  else  if ((HeadingValue <250.0f)&&(HeadingValue >= 205.0f)) {
    BSP_LED_On(LED8);
    BSP_LED_Off(LED6);
    BSP_LED_Off(LED10);
    BSP_LED_Off(LED4);
    BSP_LED_Off(LED9);
    BSP_LED_Off(LED5);
    BSP_LED_Off(LED3);
    BSP_LED_Off(LED7);
  } 
  else  if ((HeadingValue < 295.0f)&&(HeadingValue >= 250.0f)) {
    BSP_LED_On(LED6);
    BSP_LED_Off(LED9);
    BSP_LED_Off(LED10);
    BSP_LED_Off(LED8);
    BSP_LED_Off(LED3);
    BSP_LED_Off(LED5);
    BSP_LED_Off(LED4);
    BSP_LED_Off(LED7);
  }        
  else  if ((HeadingValue < 340.0f)&&(HeadingValue >= 295.0f)) {
    BSP_LED_On(LED4);
    BSP_LED_Off(LED6);
    BSP_LED_Off(LED10);
    BSP_LED_Off(LED7);
    BSP_LED_Off(LED9);
    BSP_LED_Off(LED3);
    BSP_LED_Off(LED8);
    BSP_LED_Off(LED5);
  }

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
  

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/     

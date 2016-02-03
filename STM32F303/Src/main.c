/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <math.h>
#include "main.h"
#include "filter.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

osThreadId defaultTaskHandle;
osThreadId sensorTaskHandle;
osThreadId motorCtrlTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static const uint16_t inputAxis[10]     = {0,  1, 5, 10, 20, 30, 50, 70, 85, 100};
static const float outCurve[10] = {0, 0.1, 0.25, 0.35, 0.5, 0.55, 0.65, 0.9, 1.3, 2.0};  // komfort

uint32_t msTime = 0;
uint32_t uart4RxTimeStamp = 0;

static volatile int16_t rawAngleLeft = 0;
static volatile int16_t rawAngleRight = 0;
static float toMeterFactor = 0.0082;
static float posLeft = 0.0;
static float posRight = 0.0;
static float posLeft_prev = 0.0;
static float posRight_prev = 0.0;
static volatile float spdLeft = 0.0;
static volatile float spdRight = 0.0;
static float spdLeftFiltered = 0.0;
static float spdRightFiltered = 0.0;
//static float accLeft = 0.0;
//static float accRight = 0.0;
//static float accLeftFiltered = 0.0;
//static float accRightFiltered = 0.0;
static volatile float gyroX,gyroY,gyroZ = 0;
static float compassX,compassY,compassZ = 0;
static volatile float compassX_Max,compassY_Max,compassZ_Max = 0;
static volatile float compassX_Min,compassY_Min,compassZ_Min = 0;
static volatile float compassX_Range,compassY_Range,compassZ_Range = 0;
static volatile float compassX_Norm,compassY_Norm,compassZ_Norm = 0;
static volatile float compassX_NormFiltered,compassY_NormFiltered,compassZ_NormFiltered = 0;
static volatile float compassX_NormFilteredTilted,compassY_NormFilteredTilted = 0.0f;
static volatile float roll,pitch,heading = 0;

static volatile float temperature,temperatureGyro = 0;

static volatile float fNormAcc,fSinRoll,fCosRoll,fSinPitch,fCosPitch = 0.0f;

static volatile float accX,accY,accZ = 0;
static volatile float accXFiltered,accYFiltered,accZFiltered = 0;
static volatile float accXFilteredKalman,accYFilteredKalman,accZFilteredKalman = 0;
static volatile float accXFilteredLPF,accYFilteredLPF,accZFilteredLPF = 0;

static int16_t forceLeft = 0;
static int16_t forceLeftBeforeSaturation = 0;
static int16_t absForceLeft = 0;
static int16_t forceRight = 0;
static int16_t forceRightBeforeSaturation = 0;
static int16_t absForceRight = 0;
static int16_t hillHolderForceLeft = 0;
static int16_t hillHolderForceRight = 0;
static int16_t summaForceLeft = 0;
static int16_t summaForceRight = 0;


static uint16_t UserButtonPressed = 0;
static uint8_t enaSwitch = 1;

static volatile uint16_t rawADC = 0;
static volatile float batteryVoltage = 0;

static volatile uint32_t rawFrontDistance;
static volatile float frontDistance;


static uint8_t receiveBuffer[3]={0};
static float referenceSpeedLeft = 0;
static float referenceSpeedRight = 0;
static volatile float referenceSpeedLeftPrev = 0;
static volatile float referenceSpeedRightPrev = 0;
static float referenceSpeedLeftQualified = 0;
static float referenceSpeedRightQualified = 0;
static uint8_t lastSyncBranch = 2;
static uint8_t syncBranch = 2;
static int8_t syncCorruption = 0;

static volatile float spdErrorLeft = 0;
static volatile float spdErrorLeftBefore = 0;
static volatile float spdIntErrorLeft = 0;
static volatile float spdDerivatedErrorLeft = 0;
static volatile float spdErrorRight = 0;
static volatile float spdErrorRightBefore = 0;
static volatile float spdIntErrorRight = 0;
static volatile float spdDerivatedErrorRight = 0;

static volatile float ctrlP = 40;
static volatile float ctrlI = 0.04;
static volatile float ctrlD = 3;
static volatile float antiWindup = 1;

static volatile float hillHolderGainFW = -0.75;
static volatile float hillHolderGainBW = -0.45;

volatile long int ultrasonicDelay = 0;
static volatile uint16_t echoTimeStamp = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartSensorTask(void const * argument);
void StartMotorCtrllTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define PI     3.14159
#define _2PI    6.28318
#define _180_PI 57.2958

#define COMPASS_X_MAX -687//569
#define COMPASS_X_MIN -1751//-560
#define COMPASS_Y_MAX 1055//491
#define COMPASS_Y_MIN -38//-653
#define COMPASS_Z_MAX 517//540
#define COMPASS_Z_MIN -585//-570
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIMEx_PWMN_Start(&htim2, TIM_CHANNEL_4);
  
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
  
  HAL_ADC_Start(&hadc1);
  
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
  
  TIM6->EGR = 1;           // Generate an update event
  TIM6->CR1 = 1;           // Enable the counter
  
  
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI); 
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED5);
  BSP_LED_Init(LED7);
  BSP_LED_Init(LED9);
  BSP_LED_Init(LED10);
  BSP_LED_Init(LED8);
  BSP_LED_Init(LED6);
  
  BSP_GYRO_Init();
  BSP_ACCELERO_Init();
  MAGNET_Init();
  
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of sensorTask */
  osThreadDef(sensorTask, StartSensorTask, osPriorityNormal, 0, 128);
  sensorTaskHandle = osThreadCreate(osThread(sensorTask), NULL);

  /* definition and creation of motorCtrlTask */
  osThreadDef(motorCtrlTask, StartMotorCtrllTask, osPriorityHigh, 0, 128);
  motorCtrlTaskHandle = osThreadCreate(osThread(motorCtrlTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = OVR_DATA_OVERWRITTEN;
  HAL_ADC_Init(&hadc1);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

    /**Configure Analogue filter 
    */
  HAL_I2CEx_AnalogFilter_Config(&hi2c1, I2C_ANALOGFILTER_ENABLED);

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLED;
  HAL_SPI_Init(&hspi1);

}

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_IC_InitTypeDef sConfigIC;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 480;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_IC_Init(&htim1);

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  HAL_TIM_SlaveConfigSynchronization(&htim1, &sSlaveConfig);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2400;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);

  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  HAL_TIM_Encoder_Init(&htim3, &sConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}

/* TIM4 init function */
void MX_TIM4_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  HAL_TIM_Encoder_Init(&htim4, &sConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

}

/* TIM6 init function */
void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  HAL_TIM_Base_Init(&htim6);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);

}

/* UART4 init function */
void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart4);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA2_CLK_ENABLE();
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA11   ------> USB_DM
     PA12   ------> USB_DP
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOE_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin 
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin 
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin 
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin 
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin 
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin 
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Ultrasonic_trigger_Pin */
  GPIO_InitStruct.Pin = Ultrasonic_trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(Ultrasonic_trigger_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Ultrasonic_echo_Pin */
  GPIO_InitStruct.Pin = Ultrasonic_echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Ultrasonic_echo_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DM_Pin DP_Pin */
  GPIO_InitStruct.Pin = DM_Pin|DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF14_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void saturateInteger(int16_t* i, int16_t min, int16_t max) {
  int16_t val = *i;
  if (val < min) val = min;
  else if (val > max) val = max;
  *i = val;
}
void saturateFloat(float* i, float min, float max) {
  float val = *i;
  if (val < min) val = min;
  else if (val > max) val = max;
  *i = val;
}

float absFloat(float num){
  if (num >= 0) return num;
  else return num*-1;
}

float filter2 (float avg, float input, float alpha) {
  avg = (alpha * input) + (1.0 - alpha) * avg;
  return avg;
}

float calcSpdRef_LUT(int16_t inForce){
  float outForce = 0;
  uint8_t i;
  
  for (i = 0; i < 10; i++) {
    if (inForce <= inputAxis[i]) {
      if (inForce == inputAxis[i]) {
        outForce = outCurve[i];
        break;
      }
      else {
        outForce = outCurve[i-1] + ((outCurve[i] - outCurve[i-1])*(inForce-inputAxis[i-1]))/(inputAxis[i]-inputAxis[i-1]);
        break;
      }
    }
    else {
      outForce = outCurve[9];
    }
  }
  
  return outForce;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin==USER_BUTTON_PIN)
  {
    compassX_Max = -1000;
    UserButtonPressed++;
    if (!(UserButtonPressed % 2)) enaSwitch = 1;
    else enaSwitch = 0;
    referenceSpeedLeft = 2;
    

    TIM6->EGR = 1;           // Generate an update event
  }
  else if (GPIO_Pin==GPIO_PIN_8)
  {
    __asm("NOP");
    echoTimeStamp = TIM6->CNT;
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* Get the Input Capture value */
  rawFrontDistance = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

  // SystemCoreClock = APB1 timer clock = 48 MHz 
  // Timer1 prescaler = 4800
  // => 1 tick = 100 us
  
  // Distance calculation from HC-SR04 datasheet:  uS / 58 = centimeters
  frontDistance = rawFrontDistance * 100.0 / 58 / 10;

}
/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  uint8_t * p;
  char textBuffer[100];
  
  /* Infinite loop */
  for(;;)
  {
    if (enaSwitch) CompassLED(heading);  
    osDelay(20);
    //HAL_UART_Transmit(&huart4,"kaka\r\n",6,10);
    //char lo = value & 0xFF;
    //char hi = value >> 8;
    int i = 0;
    p = (uint8_t*)&msTime;
    textBuffer[i++]=p[3];textBuffer[i++]=p[2];textBuffer[i++]=p[1];textBuffer[i++]=p[0];
    
    p = (uint8_t*)&posLeft;
    textBuffer[i++]=p[3];textBuffer[i++]=p[2];textBuffer[i++]=p[1];textBuffer[i++]=p[0];

    p = (uint8_t*)&posRight;
    textBuffer[i++]=p[3];textBuffer[i++]=p[2];textBuffer[i++]=p[1];textBuffer[i++]=p[0];
    
    p = (uint8_t*)&frontDistance;
    textBuffer[i++]=p[3];textBuffer[i++]=p[2];textBuffer[i++]=p[1];textBuffer[i++]=p[0];
    
    p = (uint8_t*)&roll;
    textBuffer[i++]=p[3];textBuffer[i++]=p[2];textBuffer[i++]=p[1];textBuffer[i++]=p[0];
    
    p = (uint8_t*)&pitch;
    textBuffer[i++]=p[3];textBuffer[i++]=p[2];textBuffer[i++]=p[1];textBuffer[i++]=p[0];
    
    p = (uint8_t*)&heading;
    textBuffer[i++]=p[3];textBuffer[i++]=p[2];textBuffer[i++]=p[1];textBuffer[i++]=p[0];
    
    p = (uint8_t*)&batteryVoltage;
    textBuffer[i++]=p[3];textBuffer[i++]=p[2];textBuffer[i++]=p[1];textBuffer[i++]=p[0];
    
    p = (uint8_t*)&spdLeftFiltered;
    textBuffer[i++]=p[3];textBuffer[i++]=p[2];textBuffer[i++]=p[1];textBuffer[i++]=p[0];
    
    p = (uint8_t*)&spdRightFiltered;
    textBuffer[i++]=p[3];textBuffer[i++]=p[2];textBuffer[i++]=p[1];textBuffer[i++]=p[0];
    
    textBuffer[i++]=receiveBuffer[0];
    textBuffer[i++]=receiveBuffer[1];
    textBuffer[i++]=receiveBuffer[2];
    
    textBuffer[i++]='k';textBuffer[i++]='u';textBuffer[i++]='k';textBuffer[i++]='i';
    
    //sprintf(textBuffer, "%c%c%c%c%c%c%c%c\r\n", (msTime >> 24) & 0xFF, (msTime >> 16) & 0xFF, (msTime >> 8) & 0xFF, msTime & 0xFF, (posLeft >> 24) & 0xFF, (posLeft >> 16) & 0xFF, (posLeft >> 8) & 0xFF, posLeft & 0xFF);
    HAL_UART_Transmit_DMA(&huart4,(uint8_t*)textBuffer, i);// sizeof(textBuffer)/sizeof(textBuffer[0]));
    
    //HAL_UART_Receive_DMA(&huart4,receiveBuffer,3);
    HAL_UART_Receive_IT(&huart4,receiveBuffer,3);
    
    referenceSpeedLeftPrev = referenceSpeedLeft;
    referenceSpeedRightPrev = referenceSpeedRight;
    lastSyncBranch = syncBranch;
    if (msTime - uart4RxTimeStamp < 200){
      if ((receiveBuffer[2] == 0xFF) && (receiveBuffer[1] <=200) && (receiveBuffer[0] <=200)){
        if (receiveBuffer[0] - 100 >= 0) referenceSpeedLeft = calcSpdRef_LUT(receiveBuffer[0] - 100);
        else referenceSpeedLeft = -1*calcSpdRef_LUT(-1*(receiveBuffer[0] - 100));
        if (receiveBuffer[1] - 100 >= 0) referenceSpeedRight = calcSpdRef_LUT(receiveBuffer[1] - 100);
        else referenceSpeedRight = -1*calcSpdRef_LUT(-1*(receiveBuffer[1] - 100));
        syncBranch = 2;
      }
      else if ((receiveBuffer[1] == 0xFF) && (receiveBuffer[2] <=200) && (receiveBuffer[0] <=200)){
        if (receiveBuffer[2] - 100 >= 0) referenceSpeedLeft = calcSpdRef_LUT(receiveBuffer[2] - 100);
        else referenceSpeedLeft = -1*calcSpdRef_LUT(-1*(receiveBuffer[2] - 100));
        if (receiveBuffer[0] - 100 >= 0) referenceSpeedRight = calcSpdRef_LUT(receiveBuffer[0] - 100);
        else referenceSpeedRight = -1*calcSpdRef_LUT(-1*(receiveBuffer[0] - 100));
        syncBranch = 1;
      }
      else if ((receiveBuffer[0] == 0xFF) && (receiveBuffer[1] <=200) && (receiveBuffer[2] <=200)){
        if (receiveBuffer[1] - 100 >= 0) referenceSpeedLeft = calcSpdRef_LUT(receiveBuffer[1] - 100);
        else referenceSpeedLeft = -1*calcSpdRef_LUT(-1*(receiveBuffer[1] - 100));
        if (receiveBuffer[2] - 100 >= 0) referenceSpeedRight = calcSpdRef_LUT(receiveBuffer[2] - 100);
        else referenceSpeedRight = -1*calcSpdRef_LUT(-1*(receiveBuffer[2] - 100));
        syncBranch = 0;
      }
      else {
        referenceSpeedLeft = 0;
        referenceSpeedRight = 0;
      }
      if (lastSyncBranch != syncBranch) syncCorruption = 3;
      else {
        syncCorruption--;
        if (syncCorruption < 0) syncCorruption = 0;
      }
    }
    else {
      referenceSpeedLeft = 0;
      referenceSpeedRight = 0;
      syncCorruption = 0;
    }
    if (syncCorruption == 0) referenceSpeedLeftQualified = referenceSpeedLeft*1;
    if (syncCorruption == 0) referenceSpeedRightQualified = referenceSpeedRight*1;
  }
  /* USER CODE END 5 */ 
}

/* StartSensorTask function */
void StartSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartSensorTask */
  float gyroBuffer[3];
  float compassBuffer[3];
  float accelerometerBuffer[3] = {0};
  float temperatureBuffer[1] = {0};
  float temperatureBufferGyro[1] = {0};
  uint16_t taskCounter = 0;
  /* Infinite loop */
  for(;;)
  {
    taskCounter++;
    rawAngleLeft = TIM3->CNT;
    rawAngleRight = TIM4->CNT;
    
    posLeft_prev = posLeft;
    posRight_prev = posRight;
    
    posLeft = rawAngleLeft*toMeterFactor;
    posRight = rawAngleRight*toMeterFactor;
    
    spdLeft = (posLeft - posLeft_prev) * 50;
    spdRight = (posRight - posRight_prev) * 50;
    spdLeftFiltered = filter2(spdLeftFiltered, spdLeft, 0.2);
    spdRightFiltered = filter2(spdRightFiltered, spdRight, 0.2);
    
    /* Read Gyro Angular data */
    BSP_GYRO_GetXYZ(gyroBuffer);
    
    gyroX = gyroBuffer[0];
    gyroY = gyroBuffer[1];
    gyroZ = gyroBuffer[2];
    
    BSP_GYRO_GetTemp(temperatureBufferGyro);
    temperatureGyro = temperatureBufferGyro[0];
    
    /* Read Acceleration*/
    BSP_ACCELERO_GetXYZ(accelerometerBuffer);

    accX = accelerometerBuffer[0];
    accY = accelerometerBuffer[1];
    accZ = accelerometerBuffer[2];
    
    accXFiltered = filter2(accXFiltered, accX, 0.2);
    accYFiltered = filter2(accYFiltered, accY, 0.2);
    accZFiltered = filter2(accZFiltered, accZ, 0.2);
    
    accZFilteredKalman = kalman_single(accZ, 60, 10);
    accZFilteredLPF    = LPF(accZ,1,40);
    //roll  = atan2(accYFiltered, sqrt(accXFiltered*accXFiltered + accZFiltered*accZFiltered));
    //pitch = atan2(accXFiltered, sqrt(accYFiltered*accYFiltered + accZFiltered*accZFiltered));
    

    /* Read Magnetometer*/
    LSM303DLHC_MagReadXYZ(compassBuffer);
    compassX = compassBuffer[0];
    compassY = compassBuffer[1];
    compassZ = compassBuffer[2];
    
    
    if (compassX > compassX_Max) compassX_Max = compassX;
    if (compassY > compassY_Max) compassY_Max = compassY;
    if (compassZ > compassZ_Max) compassZ_Max = compassZ;
    if (compassX < compassX_Min) compassX_Min = compassX;
    if (compassY < compassY_Min) compassY_Min = compassY;
    if (compassZ < compassZ_Min) compassZ_Min = compassZ;
    
    
    //The code you quoted first subtracts the minimum raw value from the reading,
    //then divides it by the range between the minimum and maximum raw values,
    //which puts the reading into the range of 0 to 1.
    //It then multiplies the reading by 2 and subtracts 1 to put it in the range of -1 to 1.
    saturateFloat(&compassX, COMPASS_X_MIN, COMPASS_X_MAX);
    saturateFloat(&compassY, COMPASS_Y_MIN, COMPASS_Y_MAX);
    saturateFloat(&compassZ, COMPASS_Z_MIN, COMPASS_Z_MAX);
    compassX_Norm = (compassX - COMPASS_X_MIN) / (COMPASS_X_MAX - COMPASS_X_MIN) * 2 - 1.0;
    compassY_Norm = (compassY - COMPASS_Y_MIN) / (COMPASS_Y_MAX - COMPASS_Y_MIN) * 2 - 1.0;
    compassZ_Norm = (compassZ - COMPASS_Z_MIN) / (COMPASS_Z_MAX - COMPASS_Z_MIN) * 2 - 1.0;
    
    compassX_NormFiltered = filter2(compassX_NormFiltered, compassX_Norm, 0.3);
    compassY_NormFiltered = filter2(compassY_NormFiltered, compassY_Norm, 0.3);
    compassZ_NormFiltered = filter2(compassZ_NormFiltered, compassZ_Norm, 0.3);
    
    
    fNormAcc = sqrt((accXFiltered*accXFiltered)+(accYFiltered*accYFiltered)+(accZFiltered*accZFiltered));
      
    fSinRoll = -accYFiltered/fNormAcc;
    fCosRoll = sqrt(1.0-(fSinRoll * fSinRoll));
    fSinPitch = accXFiltered/fNormAcc;
    fCosPitch = sqrt(1.0-(fSinPitch * fSinPitch));
      
    if ( fSinRoll >0) {
      if (fCosRoll>0) roll = acos(fCosRoll)*180/PI;
      else            roll = acos(fCosRoll)*180/PI + 180;
    }
    else {
      if (fCosRoll>0) roll = acos(fCosRoll)*180/PI + 360;
      else            roll = acos(fCosRoll)*180/PI + 180;
    }
     
    if ( fSinPitch >0) {
      if (fCosPitch>0) pitch = acos(fCosPitch)*180/PI;
      else             pitch = acos(fCosPitch)*180/PI + 180;
    }
    else {
      if (fCosPitch>0) pitch = acos(fCosPitch)*180/PI + 360;
      else             pitch = acos(fCosPitch)*180/PI + 180;
    }

    if (roll >=360)  roll = 360 - roll;
    if (pitch >=360) pitch = 360 - pitch;
    
    compassX_NormFilteredTilted = compassX_NormFiltered*fCosPitch+compassZ_NormFiltered*fSinPitch;
    compassY_NormFilteredTilted = compassX_NormFiltered*fSinRoll*fSinPitch+compassY_NormFiltered*fCosRoll-compassY_NormFiltered*fSinRoll*fCosPitch;
    
    heading = (float) ((atan2f((float)compassY_NormFilteredTilted,(float)compassX_NormFilteredTilted))*180)/PI;
    heading += 180;
    
    if (heading < 0) heading = heading + 360;    
    
    
    /*
    heading = atan2f(compassX_Norm, compassY_Norm);          // atan2 to calculate heading vector in x-y plane
    if(heading < 0) heading += _2PI;           // heading should be between 0 and 2PI
    else if(heading > _2PI) heading -= _2PI;
    heading *= _180_PI;                        // convert to degrees
    
    roll *= _180_PI;                        // convert to degrees
    pitch *= _180_PI;                        // convert to degrees
    */
    LSM303DLHC_MagReadTemp(temperatureBuffer);
    temperature = temperatureBuffer[0];
    
    rawADC = HAL_ADC_GetValue(&hadc1);
    batteryVoltage = rawADC/419.0;
    
    if (taskCounter % 2 == 0){
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
      
      ultrasonicDelay = 100;
      while (--ultrasonicDelay);
      
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    }
    osDelay(20);
  }
  /* USER CODE END StartSensorTask */
}

/* StartMotorCtrllTask function */
void StartMotorCtrllTask(void const * argument)
{
  /* USER CODE BEGIN StartMotorCtrllTask */
  /* Infinite loop */
  for(;;)
  {
    spdErrorLeftBefore = spdErrorLeft;
    spdErrorLeft = referenceSpeedLeftQualified - spdLeftFiltered;
    spdIntErrorLeft = spdIntErrorLeft + spdErrorLeft + antiWindup * (forceLeft - forceLeftBeforeSaturation);
    spdDerivatedErrorLeft = spdErrorLeft - spdErrorLeftBefore;
      
    forceLeft = (int)(ctrlP * spdErrorLeft + ctrlI * spdIntErrorLeft + ctrlD * spdDerivatedErrorLeft);
    forceLeftBeforeSaturation = forceLeft;
    
    spdErrorRightBefore = spdErrorRight;
    spdErrorRight = referenceSpeedRightQualified - spdRightFiltered;
    spdIntErrorRight = spdIntErrorRight + spdErrorRight + antiWindup * (forceRight - forceRightBeforeSaturation);
    spdDerivatedErrorRight = spdErrorRight - spdErrorRightBefore;
    
    forceRight = (int)(ctrlP * spdErrorRight + ctrlI * spdIntErrorRight + ctrlD * spdDerivatedErrorLeft);
    forceRightBeforeSaturation = forceRight;
    
    
    if ((absFloat(referenceSpeedLeftQualified) < 0.1) && (absFloat(spdErrorLeft) < 0.1)){
      spdIntErrorLeft-=spdIntErrorLeft*0.05;
    }
    if ((absFloat(referenceSpeedRightQualified) < 0.1) && (absFloat(spdErrorRight) < 0.1)){
      spdIntErrorRight-=spdIntErrorRight*0.05;
    }
    
    //if (spdErrorLeft*spdIntErrorLeft < 0)spdIntErrorLeft-=spdIntErrorLeft*0.1;
    //if (spdErrorRight*spdIntErrorRight < 0)spdIntErrorRight-=spdIntErrorRight*0.1;
    
    if (pitch > 5){
      hillHolderForceLeft = (int)(pitch*hillHolderGainBW);
      hillHolderForceRight = (int)(pitch*hillHolderGainBW);
      saturateInteger(&hillHolderForceLeft, -18, 18);
      saturateInteger(&hillHolderForceRight, -18, 18);
    }
    else if (pitch < -5){
      hillHolderForceLeft = (int)(pitch*hillHolderGainFW);
      hillHolderForceRight = (int)(pitch*hillHolderGainFW);
      saturateInteger(&hillHolderForceLeft, -18, 18);
      saturateInteger(&hillHolderForceRight, -18, 18);
    }
    else {
      hillHolderForceLeft = 0;
      hillHolderForceRight = 0;
    }
    
    summaForceLeft = forceLeft+hillHolderForceLeft;
    summaForceRight = forceRight+hillHolderForceRight;
    
    ////// PWM OUT //////
    saturateInteger(&summaForceLeft, -98, 98);
    absForceLeft = abs(summaForceLeft);
    saturateInteger(&summaForceRight, -98, 98);
    absForceRight = abs(summaForceRight);
    
    //forceLeft = 20;
    //forceRight = 20;
      
    
    if (forceLeft >= 0) {
      TIM2->CCR1 = 0;
      TIM2->CCR2 = absForceLeft*2400/100;
    }
    else {
      TIM2->CCR1 = absForceLeft*2400/100;
      TIM2->CCR2 = 0;
    }
    
    if (forceRight >= 0) {
      TIM2->CCR3 = 0;
      TIM2->CCR4 = absForceRight*2400/100;
    }
    else {
      TIM2->CCR3 = absForceRight*2400/100;
      TIM2->CCR4 = 0;
    }
    
    osDelay(1);
  }
  /* USER CODE END StartMotorCtrllTask */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

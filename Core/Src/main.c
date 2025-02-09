/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
  uint32_t startTick;
  uint32_t targetTick;
} TimerContext;

typedef struct {
  uint32_t upPeriodMs;
  uint32_t downPeriodMs;
  double threshould;
  uint32_t prevTick;
  double accumulateValue;
  int currValue;
} FilterContext;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define VALVE_ON_LEVEL            SensorLevel0
#define VALVE_OFF_LEVEL           SensorLevel4
#define PUMP_ON_TIMEOUT_MS        (1000 * 60)
#define TANK_EMPTY_BLINK_MS       (500)
#define FILTER_UP_PERIOD_MS       (200)
#define FILTER_DOWN_PERIOD_MS     (1000)
#define FILTER_THRESHOULD         (0.2)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

volatile uint16_t sensor0Curr = 0;
volatile uint16_t sensor0Prev = 0;
volatile uint16_t sensor0Value = 0;
FilterContext sensor0Filter;
volatile SensorLevel sensor0Level = 0;

volatile uint16_t sensor1Curr = 0;
volatile uint16_t sensor1Prev = 0;
volatile uint16_t sensor1Value = 0;
FilterContext sensor1Filter;
volatile SensorLevel sensor1Level = 0;

FilterContext sensorEmptyFilter;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

void initCycleTime(TimerContext* context, uint32_t cycleTimeMs) {
  context->startTick = HAL_GetTick();
  context->targetTick = cycleTimeMs;
}

bool isOnCycleTime(TimerContext* context) {
  uint32_t currTick = HAL_GetTick();
  uint32_t interval = currTick - context->startTick;
  if (interval > context->targetTick) {
    uint32_t tickShift = ((interval / context->targetTick) > 1) ? 0 : (interval % context->targetTick);
    context->startTick = currTick - tickShift;
    return true;
  }
  return false;
}

void initAfterTime(TimerContext* context, uint32_t timeMs) {
  context->startTick = HAL_GetTick();
  context->targetTick = timeMs;
}

bool isAfterTime(TimerContext* context) {
  uint32_t currTick = HAL_GetTick();
  uint32_t interval = currTick - context->startTick;
  return interval > context->targetTick;
}

void initSensorFilter(FilterContext* context, uint32_t upPeriodMs, uint32_t downPeriodMs, double threshould) {
  context->upPeriodMs = upPeriodMs;
  context->downPeriodMs = downPeriodMs;
  context->threshould = threshould;
  context->prevTick = HAL_GetTick();
  context->accumulateValue = 0;
  context->currValue = 0;
}

int sensorFilter(FilterContext* context, int value) {
  uint32_t currTick = HAL_GetTick();

  double interval = currTick - context->prevTick;
  double period = (value >= context->accumulateValue) ? context->upPeriodMs : context->downPeriodMs;
  double step = (interval < period) ? (interval / period) : 1.0;
  context->accumulateValue = (context->accumulateValue * (1.0 - step)) + (value * step);

  int currValue = (int) context->accumulateValue;
  if (fabs(currValue - context->accumulateValue) > context->threshould) {
    currValue += (currValue >= 0) ? 1 : -1;
  }
 
  context->prevTick = currTick;
  context->currValue = currValue;

  return currValue;
}

#define ledOn(color)    HAL_GPIO_WritePin(LED_##color##_GPIO_Port, LED_##color##_Pin, GPIO_PIN_RESET);
#define ledOff(color)   HAL_GPIO_WritePin(LED_##color##_GPIO_Port, LED_##color##_Pin, GPIO_PIN_SET);
#define pumpOn()        HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_SET);
#define pumpOff()       HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_RESET);
#define valve0On()      HAL_GPIO_WritePin(VALVE0_GPIO_Port, VALVE0_Pin, GPIO_PIN_SET);
#define valve0Off()     HAL_GPIO_WritePin(VALVE0_GPIO_Port, VALVE0_Pin, GPIO_PIN_RESET);
#define valve1On()      HAL_GPIO_WritePin(VALVE1_GPIO_Port, VALVE1_Pin, GPIO_PIN_SET);
#define valve1Off()     HAL_GPIO_WritePin(VALVE1_GPIO_Port, VALVE1_Pin, GPIO_PIN_RESET);

bool isSensorEmpty() {
  GPIO_PinState state = HAL_GPIO_ReadPin(SENSOR_EMPTY_GPIO_Port, SENSOR_EMPTY_Pin);
  return state == GPIO_PIN_SET;
}

SensorLevel toSensorLevel(uint32_t sensorVal) {
  if ((sensorVal >= 2800) && (sensorVal < 6000)) {
    // 4000
    return SensorLevel0;
  } else if ((sensorVal >= 1200) && (sensorVal < 2800)) {
    // 1600
    return SensorLevel1;
  } else if ((sensorVal >= 600) && (sensorVal < 1200)) {
    // 800
    return SensorLevel2;
  } else if ((sensorVal >= 300) && (sensorVal < 600)) {
    // 400
    return SensorLevel3;
  } else if ((sensorVal >= 100) && (sensorVal < 300)) {
    // 200
    return SensorLevel4;
  }
  return SensorLevelInvalid;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim2) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
      sensor0Curr = htim->Instance->CCR3;
      if (sensor0Curr != sensor0Prev) {
        sensor0Value = sensor0Curr - sensor0Prev;
        SensorLevel level = toSensorLevel(sensor0Value);
        sensor0Level = sensorFilter(&sensor0Filter, level);
      }
      sensor0Prev = sensor0Curr;
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
      sensor1Curr = htim->Instance->CCR4;
      if (sensor1Curr != sensor1Prev) {
        sensor1Value = sensor1Curr - sensor1Prev;
        SensorLevel level = toSensorLevel(sensor1Value);
        sensor1Level = sensorFilter(&sensor1Filter, level);
      }
      sensor1Prev = sensor1Curr;
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  initSensorFilter(&sensor0Filter, FILTER_UP_PERIOD_MS, FILTER_DOWN_PERIOD_MS, FILTER_THRESHOULD);
  initSensorFilter(&sensor1Filter, FILTER_UP_PERIOD_MS, FILTER_DOWN_PERIOD_MS, FILTER_THRESHOULD);
  initSensorFilter(&sensorEmptyFilter, FILTER_UP_PERIOD_MS, FILTER_DOWN_PERIOD_MS, FILTER_THRESHOULD);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_IWDG_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  ledOff(RED);
  ledOff(GREEN);
  ledOff(BLUE);

  //HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

  LOG("AutoPump initialized!");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  bool isTankEmpty = false;
  bool isLedRedOn = false;
  bool isLedGreenOn = true;
  bool isValve0On = false;
  bool isValve1On = false;
  bool isPumpOn = false;
  bool isError = false;

  TimerContext pumpOnTimeout;
  TimerContext tankEmptyCycle;

  TimerContext logCycle;
  initCycleTime(&logCycle, 500);
  while (1)
  {
    int sensorEmptyLevel = sensorFilter(&sensorEmptyFilter, isSensorEmpty() ? 1 : 0);
    bool isPrevTankEmpty = isTankEmpty;
    bool isPrevPumpOn = isPumpOn;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    isTankEmpty = sensorEmptyLevel > 0;
    
    // For debug purpose
    //isTankEmpty = !isTankEmpty;

    if (isError) {
      isLedRedOn = true;
      isLedGreenOn = false;
      isValve0On = false;
      isValve1On = false;
      isPumpOn = false;
    } else if (isTankEmpty) {
      if (!isPrevTankEmpty) {
        initCycleTime(&tankEmptyCycle, TANK_EMPTY_BLINK_MS);
        isLedRedOn = true;
      } else if (isOnCycleTime(&tankEmptyCycle)) {
        isLedRedOn = !isLedRedOn;
      }
      isLedGreenOn = false;
      isValve0On = false;
      isValve1On = false;
      isPumpOn = false;
    } else {
      if (sensor0Level <= VALVE_ON_LEVEL) {
        isValve0On = true;
      } else if(sensor0Level >= VALVE_OFF_LEVEL) {
        isValve0On = false;
      }

      if (sensor1Level <= VALVE_ON_LEVEL) {
        isValve1On = true;
      } else if (sensor1Level >= VALVE_OFF_LEVEL) {
        isValve1On = false;
      }

      isPumpOn = isValve0On || isValve1On;
      if (isPumpOn) {
        // Check previous pump on state
        if (!isPrevPumpOn) {
          // From pump off to pump on, setup timeout
          initAfterTime(&pumpOnTimeout, PUMP_ON_TIMEOUT_MS);
        } else if (isAfterTime(&pumpOnTimeout)) {
          // Pump on timeout, set error flat to keep all valves & pump off
          isError = true;
          continue;
        }
      }

      isLedRedOn = isPumpOn;
      isLedGreenOn = true;
    }

    if (isLedRedOn) {
      ledOn(RED);
    } else {
      ledOff(RED);
    }
    if (isLedGreenOn) {
      ledOn(GREEN);
    } else {
      ledOff(GREEN);
    }
    if (isValve0On) {
      valve0On();
    } else {
      valve0Off();
    }
    if (isValve1On) {
      valve1On();
    } else {
      valve1Off();
    }
    if (isPumpOn) {
      pumpOn();
    } else {
      pumpOff();
    }

    if (isOnCycleTime(&logCycle)) {
      LOG("S0[%u] S1[%u] SE[%u] | V0[%u] V1[%u] Pump[%u] | R[%u] G[%u] ", sensor0Level, sensor1Level, sensorEmptyLevel, isValve0On, isValve1On, isPumpOn, isLedRedOn, isLedGreenOn);
    }

    HAL_IWDG_Refresh(&hiwdg);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 1024;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 100;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_RED_Pin|LED_GREEN_Pin|LED_BLUE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PUMP_Pin|VALVE0_Pin|VALVE1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_RED_Pin LED_GREEN_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_GREEN_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SENSOR_EMPTY_Pin */
  GPIO_InitStruct.Pin = SENSOR_EMPTY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SENSOR_EMPTY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PUMP_Pin VALVE0_Pin VALVE1_Pin */
  GPIO_InitStruct.Pin = PUMP_Pin|VALVE0_Pin|VALVE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

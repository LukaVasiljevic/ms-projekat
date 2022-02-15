/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

#define ADC_PORT GPIOA
#define DISPLAY_PORT GPIOB
// X osa jedinice
#define XJ1 GPIO_PIN_0
#define XJ2 GPIO_PIN_1
#define XJ3 GPIO_PIN_2
#define XJ4 GPIO_PIN_3
// X osa desetice
#define XD1 GPIO_PIN_4
#define XD2 GPIO_PIN_5
#define XD3 GPIO_PIN_6
#define XD4 GPIO_PIN_7
// Y osa jedinice
#define YJ1 GPIO_PIN_8
#define YJ2 GPIO_PIN_9
#define YJ3 GPIO_PIN_10
#define YJ4 GPIO_PIN_11
// Y osa desetice
#define YD1 GPIO_PIN_12
#define YD2 GPIO_PIN_13
#define YD3 GPIO_PIN_14
#define YD4 GPIO_PIN_15
// led za belezenje konverzije
#define ADCX GPIO_PIN_1
#define ADCY GPIO_PIN_3
// interrupt pin
#define INTERR_PIN GPIO_PIN_0
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void initAxes(void);
void setAxis(int coordinate, char axis);
void setAxisJ(int partialCoordinate, char axis);
void setAxisD(int partialCoordinate, char axis);

void getADCValues(uint32_t *adcResultX, uint32_t *adcResultY);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ADC_SELECT_CH0(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}
void ADC_SELECT_CH1(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

uint32_t adcResultX, adcResultY;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	initAxes();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		getADCValues(&adcResultX, &adcResultY);

		int xValue = (adcResultX * 100) / 4095;
		setAxis(xValue == 100 ? 99 : xValue, 'X');
		int yValue = (adcResultY * 100) / 4095;
		setAxis(yValue == 100 ? 99 : yValue, 'Y');
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
//  /** Configure Regular Channel
//  */
//  sConfig.Channel = ADC_CHANNEL_2;
//  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure Regular Channel
//  */
//  sConfig.Channel = ADC_CHANNEL_4;
//  sConfig.Rank = ADC_REGULAR_RANK_2;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA3 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB11 PB12 PB13 PB14 
                           PB15 PB3 PB4 PB5 
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void initAxes(void) {
	HAL_GPIO_WritePin(DISPLAY_PORT, XJ1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DISPLAY_PORT, XJ2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DISPLAY_PORT, XJ3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DISPLAY_PORT, XJ4, GPIO_PIN_SET);

	HAL_GPIO_WritePin(DISPLAY_PORT, XD1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DISPLAY_PORT, XD2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DISPLAY_PORT, XD3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DISPLAY_PORT, XD4, GPIO_PIN_SET);

	HAL_GPIO_WritePin(DISPLAY_PORT, YJ1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DISPLAY_PORT, YJ2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DISPLAY_PORT, YJ3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DISPLAY_PORT, YJ4, GPIO_PIN_SET);

	HAL_GPIO_WritePin(DISPLAY_PORT, YD1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DISPLAY_PORT, YD2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DISPLAY_PORT, YD3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DISPLAY_PORT, YD4, GPIO_PIN_SET);

	HAL_Delay(300);

	HAL_GPIO_WritePin(DISPLAY_PORT, XJ1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DISPLAY_PORT, XJ2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DISPLAY_PORT, XJ3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DISPLAY_PORT, XJ4, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(DISPLAY_PORT, XD1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DISPLAY_PORT, XD2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DISPLAY_PORT, XD3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DISPLAY_PORT, XD4, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(DISPLAY_PORT, YJ1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DISPLAY_PORT, YJ2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DISPLAY_PORT, YJ3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DISPLAY_PORT, YJ4, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(DISPLAY_PORT, YD1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DISPLAY_PORT, YD2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DISPLAY_PORT, YD3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DISPLAY_PORT, YD4, GPIO_PIN_RESET);

}

void setAxis(int coordinate, char axis) {
	setAxisJ(coordinate % 10, axis);
	setAxisD(coordinate / 10, axis);
}
void setAxisJ(int partialCoordinate, char axis) {
	if (axis == 'X') {
		HAL_GPIO_WritePin(DISPLAY_PORT, XJ1,
		CHECK_BIT(partialCoordinate, 0) == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
		HAL_GPIO_WritePin(DISPLAY_PORT, XJ2,
		CHECK_BIT(partialCoordinate, 1) == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
		HAL_GPIO_WritePin(DISPLAY_PORT, XJ3,
		CHECK_BIT(partialCoordinate, 2) == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
		HAL_GPIO_WritePin(DISPLAY_PORT, XJ4,
		CHECK_BIT(partialCoordinate, 3) == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
	} else if (axis == 'Y') {
		HAL_GPIO_WritePin(DISPLAY_PORT, YJ1,
		CHECK_BIT(partialCoordinate, 0) == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
		HAL_GPIO_WritePin(DISPLAY_PORT, YJ2,
		CHECK_BIT(partialCoordinate, 1) == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
		HAL_GPIO_WritePin(DISPLAY_PORT, YJ3,
		CHECK_BIT(partialCoordinate, 2) == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
		HAL_GPIO_WritePin(DISPLAY_PORT, YJ4,
		CHECK_BIT(partialCoordinate, 3) == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
	}
}
void setAxisD(int partialCoordinate, char axis) {
	if (axis == 'X') {
		HAL_GPIO_WritePin(DISPLAY_PORT, XD1,
		CHECK_BIT(partialCoordinate, 0) == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
		HAL_GPIO_WritePin(DISPLAY_PORT, XD2,
		CHECK_BIT(partialCoordinate, 1) == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
		HAL_GPIO_WritePin(DISPLAY_PORT, XD3,
		CHECK_BIT(partialCoordinate, 2) == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
		HAL_GPIO_WritePin(DISPLAY_PORT, XD4,
		CHECK_BIT(partialCoordinate, 3) == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
	} else if (axis == 'Y') {
		HAL_GPIO_WritePin(DISPLAY_PORT, YD1,
		CHECK_BIT(partialCoordinate, 0) == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
		HAL_GPIO_WritePin(DISPLAY_PORT, YD2,
		CHECK_BIT(partialCoordinate, 1) == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
		HAL_GPIO_WritePin(DISPLAY_PORT, YD3,
		CHECK_BIT(partialCoordinate, 2) == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
		HAL_GPIO_WritePin(DISPLAY_PORT, YD4,
		CHECK_BIT(partialCoordinate, 3) == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
	}
}
void getADCValues(uint32_t *adcResultX, uint32_t *adcResultY) {

	HAL_GPIO_WritePin(ADC_PORT, ADCX, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ADC_PORT, ADCY, GPIO_PIN_SET);

	HAL_Delay(300);
	ADC_SELECT_CH0();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	*adcResultX = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	ADC_SELECT_CH1();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	*adcResultY = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	HAL_GPIO_WritePin(ADC_PORT, ADCX, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ADC_PORT, ADCY, GPIO_PIN_RESET);
	HAL_Delay(300);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

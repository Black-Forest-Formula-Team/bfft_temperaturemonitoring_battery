/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct sTemp
{
  uint32_t ADC_VAL[4][26];
  float Temp_VAL[4][26];
  const float Avg_Slope;			//Einheit: Kelvin/Bit
}tTemp;

tTemp Temp = {{0}, {0}, 0.08};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define Anzahl_MUX 4			//Für mehr als vier Multiplexer müssen die angeschlossenen GPIO-Pins nach dem folgenden Schema in der .ioc konfiguriert sein: A0_x ... A4_x, EN_x, WR_x, CS_x
#define Anzahl_Sensoren 26		//Maximal 32 Sensoren pro Multiplexer bei dem Modell ADG732 möglich

#define Func_OK 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


uint8_t vADC = 0;
uint8_t *pchangeADC = &vADC;

/**
 * @brief changes ADC channel
 * @param uint8_t* 0...Anzahl_MUX
 * @retval None
 */

void changeCH(uint8_t *changeADC)
{
	uint8_t channel = (*changeADC + 10);
	ADC_ChannelConfTypeDef sConfig;
	switch (channel)
	{
	case 10: sConfig.Channel = ADC_CHANNEL_10; break;
	case 11: sConfig.Channel = ADC_CHANNEL_11; break;
	case 12: sConfig.Channel = ADC_CHANNEL_12; break;
	case 13: sConfig.Channel = ADC_CHANNEL_13; break;				//Für mehr als 4 Multiplexer hier erweitern!!!
	//default: return(void);
	}
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

/*void ADC_Select_CH10 (void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  * Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.

	  sConfig.Channel = ADC_CHANNEL_10;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}*/


static uint16_t muxcontroll[3][Anzahl_MUX] =
{
		{CS_0_Pin, CS_1_Pin, CS_2_Pin, CS_3_Pin},
		{EN_0_Pin, EN_1_Pin, EN_2_Pin, EN_3_Pin},
		{WR_0_Pin, WR_1_Pin, WR_2_Pin, WR_3_Pin}
};

static uint16_t muxaddress[Anzahl_MUX][5] =
{
		{A0_0_Pin, A1_0_Pin, A2_0_Pin, A3_0_Pin, A4_0_Pin},
		{A0_1_Pin, A1_1_Pin, A2_1_Pin, A3_1_Pin, A4_1_Pin},
		{A0_2_Pin, A1_2_Pin, A2_2_Pin, A3_2_Pin, A4_2_Pin},
		{A0_3_Pin, A1_3_Pin, A2_3_Pin, A3_3_Pin, A4_3_Pin}
};

//typedef struct smuxcontroll
//{
//	uint16_t CS_Pin[4];
//	uint16_t EN_Pin[4];
//	uint16_t WR_Pin[4];
//}tmuxcontroll;
//
//struct smuxaddress
//{
//	uint16_t A0[4] = {A0_0_Pin, A0_1_Pin, A0_2_Pin, A0_3_Pin};
//	uint16_t A1[4] = {A1_0_Pin, A1_1_Pin, A1_2_Pin, A1_3_Pin};
//	uint16_t A2[4] = {A2_0_Pin, A2_1_Pin, A2_2_Pin, A2_3_Pin};
//	uint16_t A3[4] = {A3_0_Pin, A3_1_Pin, A3_2_Pin, A3_3_Pin};
//	uint16_t A4[4] = {A4_0_Pin, A4_1_Pin, A4_2_Pin, A4_3_Pin};
//}muxaddress;

/**
 * @brief returns fitting port of corresponding MUX
 * @param uint8_t 0...3
 * @retval GPIO_TypeDef*
 */

GPIO_TypeDef* selectMuxPort(uint8_t port)
{
	switch (port)
	{
		case 0: return(GPIOA); break;
		case 1: return(GPIOC); break;
		case 2: return(GPIOG); break;
		case 3: return(GPIOE); break;								//Port des weiteren Multiplexers hier erweitern
		default: return(NULL);				//default case gibt Null-Zeiger zurück
	}
}

/**
 * @brief Set CS, EN, WR to zero, runs only once
 * @param None
 * @retval None
 */
void initmux()
{
//	tmuxcontroll muxcontroll;
//	muxcontroll.CS_Pin[0]= CS_0_Pin;
//	muxcontroll.CS_Pin[1]= CS_1_Pin;
//	muxcontroll.CS_Pin[2]= CS_2_Pin;
//	muxcontroll.CS_Pin[3]= CS_3_Pin;
//
//	muxcontroll.EN_Pin[4] = {EN_0_Pin, EN_1_Pin, EN_2_Pin, EN_3_Pin};
//	{WR_0_Pin, WR_1_Pin, WR_2_Pin, WR_3_Pin};

	for(int i=0; i<Anzahl_MUX; i++)
	{
		for(int j=0; j<3; j++)
		{
			HAL_GPIO_WritePin(selectMuxPort(i), muxcontroll[j][i], GPIO_PIN_RESET);
		}
	}
}


/**
 * @brief Set A0, A1, A2, A3, A4 according to calculated binary, runs for every change
 * @param uint8_t
 * @retval 1: success
 */

int setMuxAddress(uint8_t pin)				//Adresse wird für jeden Multiplexer gleich gesetzt
{
	uint8_t binary[5] = { 0 };

	for (int i = 0; pin > 0; i++)
	{
		binary[i] = pin % 2;			//Umrechnung Dezimal-Zahl in Binär-Zahl
		pin = pin / 2;
	}

	for (int i = 0; i < Anzahl_MUX; i++)			//Schleife für Multiplexer
	{
		HAL_GPIO_WritePin(selectMuxPort(i), muxcontroll[2][i], GPIO_PIN_RESET);		//setze WR-Pin auf 0 --> Multiplexer ist bereit
		for (int j = 0; j < 5; j++)		//Schleife für Setzen der Adress-Bits
		{
			if(i!=1)
			{
				HAL_GPIO_WritePin(selectMuxPort(i), muxaddress[i][j], binary[j]);
			} else
			{
				HAL_GPIO_WritePin(GPIOF, muxaddress[i][j], binary[j]);			//Ausnahme dokumentieren
			}
		}
		HAL_GPIO_WritePin(selectMuxPort(i), muxcontroll[2][i], GPIO_PIN_SET);			//setze WR-Pin auf 1 --> Multiplexer schaltet Eingang auf Ausgang entsprechend des Adressleitungen
	}

	return Func_OK;

}

/**
 * @brief entry function for collecting temp data
 * @param none
 * @retval 1: success
 */

uint8_t readTempVal()
{
	for (int i = 0; i < Anzahl_Sensoren; i++)
	{
		setMuxAddress(i);

		for(uint8_t j=0; j<Anzahl_MUX; j++)
		{
			changeCH(&j);
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 1000);
			Temp.ADC_VAL[j][i] = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);

			Temp.Temp_VAL[j][i] = (Temp.Avg_Slope*Temp.ADC_VAL[j][i]-168);
		}
	}
	return Func_OK;
}

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

  initmux();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1)
	{
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 25000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, EN_1_Pin|WR_1_Pin|CS_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, A4_1_Pin|A3_1_Pin|A2_1_Pin|A1_1_Pin
                          |A0_1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN_0_Pin|WR_0_Pin|CS_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, A4_0_Pin|A3_0_Pin|A2_0_Pin|A1_0_Pin
                          |A0_0_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, EN_3_Pin|WR_3_Pin|CS_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, A4_3_Pin|A3_3_Pin|A2_3_Pin|A1_3_Pin
                          |A0_3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, WR_2_Pin|CS_2_Pin|EN_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, A4_2_Pin|A3_2_Pin|A2_2_Pin|A1_2_Pin
                          |A0_2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : EN_1_Pin WR_1_Pin CS_1_Pin */
  GPIO_InitStruct.Pin = EN_1_Pin|WR_1_Pin|CS_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : A4_1_Pin A3_1_Pin A2_1_Pin A1_1_Pin
                           A0_1_Pin */
  GPIO_InitStruct.Pin = A4_1_Pin|A3_1_Pin|A2_1_Pin|A1_1_Pin
                          |A0_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_0_Pin WR_0_Pin CS_0_Pin A4_0_Pin
                           A3_0_Pin A2_0_Pin A1_0_Pin A0_0_Pin */
  GPIO_InitStruct.Pin = EN_0_Pin|WR_0_Pin|CS_0_Pin|A4_0_Pin
                          |A3_0_Pin|A2_0_Pin|A1_0_Pin|A0_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_3_Pin WR_3_Pin CS_3_Pin A4_3_Pin
                           A3_3_Pin A2_3_Pin A1_3_Pin A0_3_Pin */
  GPIO_InitStruct.Pin = EN_3_Pin|WR_3_Pin|CS_3_Pin|A4_3_Pin
                          |A3_3_Pin|A2_3_Pin|A1_3_Pin|A0_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : WR_2_Pin CS_2_Pin A4_2_Pin A3_2_Pin
                           A2_2_Pin A1_2_Pin A0_2_Pin EN_2_Pin */
  GPIO_InitStruct.Pin = WR_2_Pin|CS_2_Pin|A4_2_Pin|A3_2_Pin
                          |A2_2_Pin|A1_2_Pin|A0_2_Pin|EN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RXBUFFERSIZE 1   //串口接收缓冲区的长度

// ADC采样参数
#define ADC1_CHANNEL_NUM 2
#define ADC1_SAMPLING_TIMES 3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t PwmValCH1 = 0;
uint16_t PwmValCH2 = 8200;

uint16_t tim6_counter = 0;
uint16_t tim6_desired_time = 10;

uint8_t RXBUFFER[RXBUFFERSIZE]; //串口接收数据数组

// 摇杆ADC采集数组
uint16_t ADC1_ConvertedValue[ADC1_SAMPLING_TIMES][ADC1_CHANNEL_NUM] = {0};
uint32_t ADC1_FilteredValue[ADC1_CHANNEL_NUM] = {0};
float ADC1_Measured_Voltage[ADC1_CHANNEL_NUM] = {0};

//串口发送测试
uint8_t CommInitMessage[] = "Init is ok\r\n";
uint8_t CommOkMessage[] = "Everything is ok\r\n";
uint8_t CommFlag = 0;


// 轮子工作状态枚举类型
typedef enum {
	Direction_Forward,
	Direction_Backward,
	Direction_Stop
} WheelDirection;

// 摇杆读取的数值
struct JoytickAdcValue{
	uint16_t Xaxis;
	uint16_t Yaxis;
};

//__IO uint32_t PWM_OUT ENABLE;
uint16_t DCMotor_PWM_OUT = 1;

//振动马达PWM输出flag
uint16_t Vibration_Motor_PWM_OUT = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
static void Wheel_Rotate_Direction(WheelDirection LeftFrontDirection,
									WheelDirection RightFrontDirection,
									WheelDirection LeftRearDirection,
									WheelDirection RightRearDirection);
struct JoytickAdcValue JoytickReadValue(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  // 定义缓冲区，设置数据接收长度，开启串口中断接收
  HAL_UART_Receive_IT(&huart1, (uint8_t *)RXBUFFER, RXBUFFERSIZE);

  // 测试发送
  HAL_UART_Transmit(&huart1, CommInitMessage, sizeof(CommInitMessage), 1000);

  // 设置pwm的频率为10khz， pwm的频率范围一般要为1khz - 20khz
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);


  HAL_TIM_Base_Start_IT(&htim6);


  /*********小车行进测试************

  // 前进
  Wheel_Rotate_Direction(Direction_Forward, Direction_Forward, Direction_Forward, Direction_Forward);
  HAL_Delay(2000);

  // 后退
  Wheel_Rotate_Direction(Direction_Backward, Direction_Backward, Direction_Backward, Direction_Backward);
  HAL_Delay(2000);

  // 向左
  Wheel_Rotate_Direction(Direction_Backward, Direction_Forward, Direction_Forward, Direction_Backward);
  HAL_Delay(2000);

  // 向右
  Wheel_Rotate_Direction(Direction_Forward, Direction_Backward, Direction_Backward, Direction_Forward);
  HAL_Delay(2000);

  // 电机制动
  Wheel_Rotate_Direction(Direction_Stop, Direction_Stop, Direction_Stop, Direction_Stop);

  *********小车行进测试************/


  HAL_UART_Transmit(&huart1, CommOkMessage, sizeof(CommOkMessage), 1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADC1_ConvertedValue, ADC1_SAMPLING_TIMES * ADC1_CHANNEL_NUM);
	  HAL_Delay(50);

	  struct JoytickAdcValue Value = JoytickReadValue();

	  printf("------------\n");
	  printf("%d, %d\n", Value.Xaxis, Value.Yaxis);

	  if(Value.Xaxis >=0 && Value.Xaxis <=1000)
	  {
		  Wheel_Rotate_Direction(Direction_Backward, Direction_Forward, Direction_Forward, Direction_Backward);
	  }
	  else if(Value.Xaxis >=3095 && Value.Xaxis <=4095)
	  {
		  Wheel_Rotate_Direction(Direction_Forward, Direction_Backward, Direction_Backward, Direction_Forward);
	  }
	  else if(Value.Yaxis >=0 && Value.Yaxis <=1000)
	  {
		  Wheel_Rotate_Direction(Direction_Forward, Direction_Forward, Direction_Forward, Direction_Forward);
	  }
	  else if(Value.Yaxis >=3095 && Value.Yaxis <=4095)
	  {
		  Wheel_Rotate_Direction(Direction_Backward, Direction_Backward, Direction_Backward, Direction_Backward);
	  }
	  else
	  {
		  Wheel_Rotate_Direction(Direction_Stop, Direction_Stop, Direction_Stop, Direction_Stop);
	  }

//	  HAL_Delay(500);

//	  // 串口接收测试
//	  if(RXBUFFER[0] == 0x10)
//	  {
//		  HAL_GPIO_WritePin(PC11_LED_GPIO_Port, PC11_LED_Pin, GPIO_PIN_SET);
//	  }
//	  else
//	  {
//		  HAL_GPIO_WritePin(PC11_LED_GPIO_Port, PC11_LED_Pin, GPIO_PIN_RESET);
//	  }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 41;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 750;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 750;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 9999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 8399;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, FORWARD_IN1_Pin|FORWARD_IN2_Pin|FORWARD_IN3_Pin|FORWARD_IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, BACK_IN1_Pin|BACK_IN2_Pin|BACK_IN3_Pin|BACK_IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Board_LED_GPIO_Port, Board_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PC11_LED_GPIO_Port, PC11_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : FORWARD_IN1_Pin FORWARD_IN2_Pin FORWARD_IN3_Pin FORWARD_IN4_Pin */
  GPIO_InitStruct.Pin = FORWARD_IN1_Pin|FORWARD_IN2_Pin|FORWARD_IN3_Pin|FORWARD_IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_PB0_Pin */
  GPIO_InitStruct.Pin = KEY_PB0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KEY_PB0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BACK_IN1_Pin BACK_IN2_Pin BACK_IN3_Pin BACK_IN4_Pin */
  GPIO_InitStruct.Pin = BACK_IN1_Pin|BACK_IN2_Pin|BACK_IN3_Pin|BACK_IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : Board_LED_Pin */
  GPIO_InitStruct.Pin = Board_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Board_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC11_LED_Pin */
  GPIO_InitStruct.Pin = PC11_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PC11_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//	PwmValCH1 = PwmValCH1 + 50;
//	if(PwmValCH1 > 8400)
//	{
//		PwmValCH1 = 200;
//	}
//	TIM2->CCR1 = PwmValCH1;

//	if(htim == &htim6)
//	{
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);
//		tim6_counter++;
//		if(tim6_counter == tim6_desired_time)
//		{
//			HAL_TIM_Base_Stop(&htim6);
//			tim6_counter = 0;
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
//		}
//	}

//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);


}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//	HAL_Delay(10);
	if (GPIO_Pin == KEY_PB0_Pin)
	{

		HAL_GPIO_TogglePin(Board_LED_GPIO_Port, Board_LED_Pin);

		if(DCMotor_PWM_OUT == 1)
		{
			DCMotor_PWM_OUT = 0;

			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
		}
		else
		{
			DCMotor_PWM_OUT = 1;
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
		}

		if (Vibration_Motor_PWM_OUT == 1)
		{
			Vibration_Motor_PWM_OUT = 0;
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);

			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
		}
		else
		{
			Vibration_Motor_PWM_OUT = 1;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		}
	}
}



/**********************************************************
 *
 * 函数功能：麦克纳姆轮四个轮子方向的转动控制函数
 * 输入参数：WheelDirection枚举类型
 * 返回参数：无
 *
 *************************************************************/
static void Wheel_Rotate_Direction(WheelDirection LeftFrontDirection,
									WheelDirection RightFrontDirection,
									WheelDirection LeftRearDirection,
									WheelDirection RightRearDirection)
{
	//左前轮
	switch(LeftFrontDirection)
	{
		case Direction_Forward:
			HAL_GPIO_WritePin(FORWARD_IN1_GPIO_Port, FORWARD_IN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(FORWARD_IN2_GPIO_Port, FORWARD_IN2_Pin, GPIO_PIN_RESET);
			break;
		case Direction_Backward:
			HAL_GPIO_WritePin(FORWARD_IN1_GPIO_Port, FORWARD_IN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(FORWARD_IN2_GPIO_Port, FORWARD_IN2_Pin, GPIO_PIN_SET);
			break;
		case Direction_Stop:
			HAL_GPIO_WritePin(FORWARD_IN1_GPIO_Port, FORWARD_IN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(FORWARD_IN2_GPIO_Port, FORWARD_IN2_Pin, GPIO_PIN_RESET);
			break;
		default:
			break;
	}

	//右前轮
	switch(RightFrontDirection)
	{
		case Direction_Forward:
			HAL_GPIO_WritePin(FORWARD_IN3_GPIO_Port, FORWARD_IN3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(FORWARD_IN4_GPIO_Port, FORWARD_IN4_Pin, GPIO_PIN_RESET);
			break;
		case Direction_Backward:
			HAL_GPIO_WritePin(FORWARD_IN3_GPIO_Port, FORWARD_IN3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(FORWARD_IN4_GPIO_Port, FORWARD_IN4_Pin, GPIO_PIN_SET);
			break;
		case Direction_Stop:
			HAL_GPIO_WritePin(FORWARD_IN3_GPIO_Port, FORWARD_IN3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(FORWARD_IN4_GPIO_Port, FORWARD_IN4_Pin, GPIO_PIN_RESET);
			break;
		default:
			break;
	}

	//左后轮
	switch(LeftRearDirection)
	{
		case Direction_Forward:
			HAL_GPIO_WritePin(BACK_IN1_GPIO_Port, BACK_IN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BACK_IN2_GPIO_Port, BACK_IN2_Pin, GPIO_PIN_SET);
			break;
		case Direction_Backward:
			HAL_GPIO_WritePin(BACK_IN1_GPIO_Port, BACK_IN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(BACK_IN2_GPIO_Port, BACK_IN2_Pin, GPIO_PIN_RESET);
			break;
		case Direction_Stop:
			HAL_GPIO_WritePin(BACK_IN1_GPIO_Port, BACK_IN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BACK_IN2_GPIO_Port, BACK_IN2_Pin, GPIO_PIN_RESET);
			break;
		default:
			break;
	}

	//右后轮
	switch(RightRearDirection)
	{
		case Direction_Forward:
			HAL_GPIO_WritePin(BACK_IN3_GPIO_Port, BACK_IN3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(BACK_IN4_GPIO_Port, BACK_IN4_Pin, GPIO_PIN_RESET);
			break;
		case Direction_Backward:
			HAL_GPIO_WritePin(BACK_IN3_GPIO_Port, BACK_IN3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BACK_IN4_GPIO_Port, BACK_IN4_Pin, GPIO_PIN_SET);
			break;
		case Direction_Stop:
			HAL_GPIO_WritePin(BACK_IN3_GPIO_Port, BACK_IN3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BACK_IN4_GPIO_Port, BACK_IN4_Pin, GPIO_PIN_RESET);
			break;
		default:
			break;
	}
}


/**********************************************************
 *
 * 函数功能：串口中断服务函数
 *
 *************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//接收完指定长度的数据之后需要重新开启接收中断
	HAL_UART_Receive_IT(&huart1, (uint8_t *)RXBUFFER, RXBUFFERSIZE);
}



/**********************************************************
 *
 * 函数功能：通过重定向putchar函数重定向printf函数实现串口打印
 *
 *************************************************************/
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}



/***********************************************
 *
函数功能：读取单个PS2摇杆模块的X轴和Y轴的AD电压值
入口值： 无
返回值： 结构体 JoytickAdcValue Value
*
 ***********************************************/
struct JoytickAdcValue JoytickReadValue(void)
{
	for(uint8_t i = 0; i < ADC1_CHANNEL_NUM; i++)
	{
	  for(uint8_t j = 0; j < ADC1_SAMPLING_TIMES; j++)
	  {
		  ADC1_FilteredValue[i] += ADC1_ConvertedValue[j][i];
	  }
	  ADC1_FilteredValue[i] /= ADC1_SAMPLING_TIMES;
	}

	struct JoytickAdcValue Value;
	Value.Xaxis = ADC1_FilteredValue[0];
	Value.Yaxis = ADC1_FilteredValue[1];

//	printf("------\n");
//
//	printf("%d, %d, %d, %d, %d, %d \n", ADC1_ConvertedValue[0][0], ADC1_ConvertedValue[0][1],
//		  ADC1_ConvertedValue[1][0],ADC1_ConvertedValue[1][1], ADC1_ConvertedValue[2][0], ADC1_ConvertedValue[2][1]);
//
//	printf("%lu, %lu \n", ADC1_FilteredValue[0], ADC1_FilteredValue[1]);

	HAL_ADC_Stop_DMA(&hadc1);

	memset(ADC1_FilteredValue, 0, sizeof(ADC1_FilteredValue));

//	for(uint8_t i = 0; i < ADC1_CHANNEL_NUM; i++)
//	{
//	  ADC1_FilteredValue[i] = 0;
//	}
	return Value;
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

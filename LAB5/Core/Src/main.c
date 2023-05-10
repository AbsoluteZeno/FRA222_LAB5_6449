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
#include "string.h"
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
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
uint8_t RxBuffer;
uint8_t TxBuffer[1000];

char UserInput;
uint8_t LEDstatus = 1;
uint8_t ButtonModestatus = 0;
uint8_t freq = 5;

uint16_t size;
uint8_t last_button = 1;
uint8_t current_button;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void UARTITConfig();
void UIState();
void UI_MainMenu();
void UI_LEDMenu();
void UI_SpeedUp();
void UI_SpeedDown();
void UI_OnOff();
void UI_ButtonMenu();
void WrongInput();
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  sprintf((char*)TxBuffer, "\r\n========================================\r\n"
  							   "|               Main Menu              |\r\n"
  							   "========================================\r\n"
  							   "        [0] LED Control\r\n"
  							   "        [1] Button status\r\n"
  							   "----------------------------------------\r\n"
  							   "User Input>> \0");
  HAL_UART_Transmit_DMA(&huart2, TxBuffer, strlen((char*)TxBuffer));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  static uint32_t LEDtimestamp = 0;
	  static uint32_t Buttontimestamp = 0;
	  current_button = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

	  UARTITConfig();

	  //LED status
	  if (LEDstatus == 1)
	  {
		  if(HAL_GetTick() >= LEDtimestamp)
		  {
			  LEDtimestamp = HAL_GetTick() + (1000.0/freq);
			  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		  }
	  }
	  else
	  {
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	  }

	  //Button Mode
	  if (ButtonModestatus == 1)
	  {
		  if (HAL_GetTick() >= Buttontimestamp)
		  {
			  Buttontimestamp = HAL_GetTick() + 500;
			  if (last_button == 1 && current_button == 0)
			  {
				  sprintf((char*)TxBuffer, "Button status : Press\r\n"
										   "User Input>> \0");
				  HAL_UART_Transmit_IT(&huart2, TxBuffer, strlen((char*)TxBuffer));
			  }
			  else if (last_button == 0 && current_button == 1)
			  {
				  sprintf((char*)TxBuffer, "Button status : Unpress\r\n"
										   "User Input>> \0");
				  HAL_UART_Transmit_IT(&huart2, TxBuffer, strlen((char*)TxBuffer));
			  }
			  last_button = current_button;
		  }
	  }


	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void UARTITConfig()
{
	//start UART in IT Mode
	HAL_UART_Receive_IT(&huart2, &RxBuffer, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2)
	{
		UserInput = (char)RxBuffer;
		UIState();

		HAL_UART_Receive_IT(&huart2, &RxBuffer, 1);
	}
}

void UIState()
{
	static enum {Main, LED, Button} state = Main;

	switch(state)
	{
	case Main:
		if (UserInput == '0')
		{
			UI_LEDMenu();
			state = LED;
		}
		else if (UserInput == '1')
		{
			ButtonModestatus = 1;
			UI_ButtonMenu();
			state = Button;
		}
		else
		{
			WrongInput();
		}
		break;

	case LED:
		if (UserInput == 'a')
		{
			freq++;
			UI_SpeedUp();
		}
		else if (UserInput == 's')
		{
			if (freq >= 2)
			{
				freq--;
			}
			UI_SpeedDown();
		}
		else if (UserInput == 'd')
		{
			LEDstatus = (LEDstatus + 1) % 2;
			UI_OnOff();
		}
		else if (UserInput == 'x')
		{
			UI_MainMenu();
			state = Main;
		}
		else
		{
			WrongInput();
		}
		break;

	case Button:
		if (UserInput == 'x')
		{
			ButtonModestatus = 0;
			UI_MainMenu();
			state = Main;
		}
		break;
	}
	size = strlen((char*)TxBuffer);
	HAL_UART_Transmit_DMA(&huart2, TxBuffer, size);
}

void UI_MainMenu()
{
	sprintf((char*)TxBuffer, "%c\r\n========================================\r\n"
							       "|               Main Menu              |\r\n"
							       "========================================\r\n"
							       "        [0] LED Control\r\n"
							       "        [1] Button status\r\n"
							       "----------------------------------------\r\n"
							       "User Input>> \0", UserInput);
}

void UI_LEDMenu()
{
	sprintf((char*)TxBuffer, "%c\r\n========================================\r\n"
						           "|               LED Menu               |\r\n"
						           "========================================\r\n"
							       "    Current blinking speed : %d Hz\r\n"
							       "        [a] Speed Up   (+1 Hz)\r\n"
							       "        [s] Speed Down (-1 Hz)\r\n"
							       "        [d] On/Off\r\n"
							       "        [x] back\r\n"
							       "----------------------------------------\r\n"
							       "User Input>> \0", UserInput, freq);
}

void UI_ButtonMenu()
{
	sprintf((char*)TxBuffer, "%c\r\n========================================\r\n"
							       "|              Button Menu             |\r\n"
							       "========================================\r\n"
							       "        [x] back\r\n"
							       "----------------------------------------\r\n"
							       "User Input>> \0", UserInput);
}

void UI_SpeedUp()
{
	sprintf((char*)TxBuffer, "%c\r\nCurrent blinking speed is now increase to %d Hz\r\n"
							       "User Input>> \0", UserInput, freq);
}

void UI_SpeedDown()
{
	if (freq > 1)
	{
		sprintf((char*)TxBuffer, "%c\r\nCurrent blinking speed is now decrease to %d Hz\r\n"
									   "User Input>> \0", UserInput, freq);
	}
	else
	{
		sprintf((char*)TxBuffer, "%c\r\nCurrent blinking speed is now at the minimum\r\n"
									   "User Input>> \0", UserInput);
	}
}

void UI_OnOff()
{
	if (LEDstatus == 1)
	{
		sprintf((char*)TxBuffer, "%c\r\nLED status : ON\r\n"
									   "User Input>> \0", UserInput);
	}
	else
	{
		sprintf((char*)TxBuffer, "%c\r\nLED status : OFF\r\n"
									   "User Input>> \0", UserInput);
	}
}

void WrongInput()
{
	sprintf((char*)TxBuffer, "%c\r\nInvalid Input\r\n"
							       "User Input>> \0", UserInput);
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

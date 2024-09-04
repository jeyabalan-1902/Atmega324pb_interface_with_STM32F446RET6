/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;

/* USER CODE BEGIN PV */
uint8_t uartRxBuffer;
uint8_t spiRxBuffer[3];
volatile uint8_t dataReceived = 0;
volatile uint8_t light1State;
volatile uint8_t light2State;
volatile uint8_t light3State;
volatile uint8_t light4State;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_SPI1_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  if(HAL_SPI_Receive_IT(&hspi1,spiRxBuffer, sizeof(spiRxBuffer)- 1) != HAL_OK)
  {
	  //Error_Handler();
  }
  if(HAL_UART_Receive_IT(&huart4, &uartRxBuffer, 1) != HAL_OK)
  {

  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Touch_LED_1_Pin|Touch_LED_2_Pin|Touch_LED_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Touch_LED_4_GPIO_Port, Touch_LED_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Touch_LED_1_Pin Touch_LED_2_Pin Touch_LED_3_Pin */
  GPIO_InitStruct.Pin = Touch_LED_1_Pin|Touch_LED_2_Pin|Touch_LED_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Touch_LED_4_Pin */
  GPIO_InitStruct.Pin = Touch_LED_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Touch_LED_4_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi->Instance == SPI1)
	{
		spiRxBuffer[sizeof(spiRxBuffer) - 1] = '\0';
		if(strcmp((char *)spiRxBuffer, "L1") == 0)
		{
			HAL_GPIO_TogglePin(Touch_LED_1_GPIO_Port, Touch_LED_1_Pin);
			uint8_t Newlight1_state = (light1State == 0 ) ? 1 : 0;
			if(Newlight1_state != light1State){
				light1State = Newlight1_state;
				if(light1State == 1){
					uint8_t uartTxData = 'A';
					HAL_UART_Transmit(&huart4, &uartTxData, 1, 10);
				}else{
					uint8_t uartTxData = 'B';
					HAL_UART_Transmit(&huart4, &uartTxData, 1, 10);
				}
			}

		}
		else if(strcmp((char *)spiRxBuffer, "L2") == 0)
		{
			HAL_GPIO_TogglePin(Touch_LED_2_GPIO_Port, Touch_LED_2_Pin);
		    uint8_t Newlight2_state = (light2State == 0) ? 1: 0;
		    if(Newlight2_state != light2State){
		    	light2State = Newlight2_state;
		    	if(light2State == 1){
		    		uint8_t uartTxData = 'C';
		    		HAL_UART_Transmit(&huart4, &uartTxData, 1, 10);
		    	}else{
		    		uint8_t uartTxData = 'D';
		    	    HAL_UART_Transmit(&huart4, &uartTxData, 1, 10);
		    	}
		    }
		}
		else if(strcmp((char *)spiRxBuffer, "L3") == 0)
		{
			HAL_GPIO_TogglePin(Touch_LED_3_GPIO_Port, Touch_LED_3_Pin);
			uint8_t Newlight3_state = (light3State == 0) ? 1 : 0;
			if(Newlight3_state != light3State){
				light3State = Newlight3_state;
				if(light3State == 1){
					uint8_t uartTxData = 'E';
					HAL_UART_Transmit(&huart4, &uartTxData, 1, 10);
				}else{
					uint8_t uartTxData = 'F';
				    HAL_UART_Transmit(&huart4, &uartTxData, 1, 10);
				}
			}
		}
		else if(strcmp((char *)spiRxBuffer, "L4") == 0)
		{
			HAL_GPIO_TogglePin(Touch_LED_4_GPIO_Port, Touch_LED_4_Pin);
			uint8_t Newlight4_state = (light4State == 0) ? 1 : 0;
			if(Newlight4_state != light4State){
				light4State = Newlight4_state;
		        if(light4State == 1){
		        	uint8_t uartTxData = 'G';
		        	HAL_UART_Transmit(&huart4, &uartTxData, 1, 10);
		        }else{
		        	uint8_t uartTxData = 'H';
		        	HAL_UART_Transmit(&huart4, &uartTxData, 1, 10);
		        }
			}
		}
		if (HAL_SPI_Receive_IT(hspi, spiRxBuffer, sizeof(spiRxBuffer) - 1) != HAL_OK)
		{
		    //Error_Handler();
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == UART4){

        if (uartRxBuffer == 'L') {
        	//HAL_GPIO_TogglePin(Touch_LED_1_GPIO_Port, Touch_LED_1_Pin);
            if (HAL_UART_Receive(&huart4, &uartRxBuffer, 1, 10) == HAL_OK) {
                if (uartRxBuffer == '1') {
                    HAL_GPIO_WritePin(Touch_LED_1_GPIO_Port, Touch_LED_1_Pin, GPIO_PIN_SET);    // light1_ON()
                    light1State = 1;
                } else if (uartRxBuffer == '0') {
                    HAL_GPIO_WritePin(Touch_LED_1_GPIO_Port, Touch_LED_1_Pin, GPIO_PIN_RESET);  // light1_OFF()
                    light1State = 0;
                }
            }
        }
        else if (uartRxBuffer == 'M') {
        	//HAL_GPIO_TogglePin(Touch_LED_2_GPIO_Port, Touch_LED_2_Pin);
            if (HAL_UART_Receive(&huart4, &uartRxBuffer, 1, 10) == HAL_OK) {
                if (uartRxBuffer == '1') {
                    HAL_GPIO_WritePin(Touch_LED_2_GPIO_Port, Touch_LED_2_Pin, GPIO_PIN_SET);    // light2_ON()
                    light2State = 1;
                } else if (uartRxBuffer == '0') {
                    HAL_GPIO_WritePin(Touch_LED_2_GPIO_Port, Touch_LED_2_Pin, GPIO_PIN_RESET);  // light2_OFF()
                    light2State = 0;
                }
            }
        }
        else if (uartRxBuffer == 'N') {
        	//HAL_GPIO_TogglePin(Touch_LED_3_GPIO_Port, Touch_LED_3_Pin);
            if (HAL_UART_Receive(&huart4, &uartRxBuffer, 1, 10) == HAL_OK) {
                if (uartRxBuffer == '1') {
                    HAL_GPIO_WritePin(Touch_LED_3_GPIO_Port, Touch_LED_3_Pin, GPIO_PIN_SET);                      // light3_ON()
                    light3State = 1;
                } else if (uartRxBuffer == '0') {
                    HAL_GPIO_WritePin(Touch_LED_3_GPIO_Port, Touch_LED_3_Pin, GPIO_PIN_RESET);                    // light3_OFF()
                    light3State = 0;
                }
            }
        }
        else if (uartRxBuffer == 'O') {
        	//HAL_GPIO_TogglePin(Touch_LED_4_GPIO_Port, Touch_LED_4_Pin);
            if (HAL_UART_Receive(&huart4, &uartRxBuffer, 1, 10) == HAL_OK) {
                if (uartRxBuffer == '1') {
                    HAL_GPIO_WritePin(Touch_LED_4_GPIO_Port, Touch_LED_4_Pin, GPIO_PIN_SET);  // light4_ON()
                    light4State = 1;
                } else if (uartRxBuffer == '0') {
                    HAL_GPIO_WritePin(Touch_LED_4_GPIO_Port, Touch_LED_4_Pin, GPIO_PIN_RESET);  // light4_OFF()
                    light4State = 0;
                }
            }
        }
        HAL_UART_Receive_IT(&huart4, &uartRxBuffer, 1);
	}
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

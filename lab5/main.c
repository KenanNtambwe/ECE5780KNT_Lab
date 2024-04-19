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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

int16_t DataX, DataY;
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
  HAL_Init();
  SystemClock_Config();

	//Enable GPIOC and GPIOB Clock
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
	GPIO_InitTypeDef scl = {GPIO_PIN_13, GPIO_MODE_AF_OD, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, GPIO_AF5_I2C2};
	HAL_GPIO_Init(GPIOB, &scl);
	GPIO_InitTypeDef sda = {GPIO_PIN_11, GPIO_MODE_AF_OD, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL, GPIO_AF1_I2C2};
	HAL_GPIO_Init(GPIOB, &sda);
	
	GPIO_InitTypeDef p0 = {GPIO_PIN_0, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &p0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	GPIO_InitTypeDef pin14 = {GPIO_PIN_14, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL};
	HAL_GPIO_Init(GPIOB, &pin14);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	
	GPIO_InitTypeDef initstr = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
	GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initstr);

	__HAL_RCC_I2C2_CLK_ENABLE();
	
		//100kHz
    I2C2->TIMINGR |= (1<<28) | (1<<0) | (1<<1) | (1<<4) | (1<<8) | (1<<9) | (1<<10) | (1<<11) | 
                                        (1<<17) | (1<<22) ;
    //peripheral enable in CR1
    I2C2->CR1 |= (1<<0);
	
	DataX = 0;
	DataY = 0;
	
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
  I2C2->CR2 |= ((1 << 16) | (0x69 << 1)); // Set SADD = 0x69 and NBYTES = 1
  I2C2->CR2 |= (1 << 10); //Set write mode
  I2C2->CR2 |= (1 << 13); //Set start bit
	
	//Wait for the TXIS flag
	while(1)
	{
		if((I2C2->ISR & (1 << 1))) {
			break;
		}
		
		if((I2C2->ISR & (1 << 4))) {
			while(1) {
			}
		}
	}
	//Transmit data
	I2C2->TXDR = 0x0F;
	//Wait transmit complete flag
	while(!(I2C2->ISR & (1 << 6)))
	{
	}
	
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
  I2C2->CR2 |= ((1 << 16) | (0x69 << 1)); // Set SADD = 0x69 and NBYTES = 1
  I2C2->CR2 |= (1 << 10); //Set write mode
  I2C2->CR2 |= (1 << 13); //Set start bit
	
	//Wait for TXIS flag
	while(1)
	{
		if((I2C2->ISR & (1 << 2))) {
			break;
		}
		if((I2C2->ISR & (1 << 4))) {
			while(1) {
			}
		}
	}

	uint32_t RDataX = I2C2->RXDR;
	
	//Wait transmit complete flag
	while(!(I2C2->ISR & (1 << 6)))
	{
	}
	//Stop bit
	I2C2->CR2 |= (1 << 14);
	//Wait stop condition
  while(I2C2->CR2 & (1 << 14))
  { 
  }
  //Clear stop bit
  I2C2->CR2 &= ~(1 << 14);
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
  I2C2->CR2 |= ((1 << 16) | (0x69 << 1)); // Set SADD = 0x69 and NBYTES = 1
  I2C2->CR2 |= (1 << 10); //Set write mode
  I2C2->CR2 |= (1 << 13); //Set start bit
	
	//Wait for TXIS flag
	while(1)
	{
		if((I2C2->ISR & (1 << 1))) {
			break;
		}
		
		if((I2C2->ISR & (1 << 4))) {
			while(1) {
			}
		}
	}
	
	//Set transmit data
	I2C2->TXDR = 0x20;
	
	//Wait for TXIS flag
	while(1)
	{
		if((I2C2->ISR & (1 << 1))) {
			break;
		}
		
		if((I2C2->ISR & (1 << 4))) {
			while(1) {
			}
		}
	}
	
	I2C2->TXDR = 0x0B;
	
	//Wait for transmit complete
	while(!(I2C2->ISR & (1 << 6)))
	{
		
	}
	
	//Send stop bit
	I2C2->CR2 |= (1 << 14);
	
	//Wait for stop condition 
  while(I2C2->CR2 & (1 << 14))
  {
  }

  //Clear stop bit
  I2C2->CR2 &= ~(1 << 14);
	
  while (1)
  {
			DataX = 0;

	//write
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
  I2C2->CR2 |= ((1 << 16) | (0x69 << 1)); // Set SADD = 0x69 and NBYTES = 1
  I2C2->CR2 |= (1 << 10); //Set write mode
  I2C2->CR2 |= (1 << 13); //Set start bit
	//Wait for TXIS flag
	while(1)
	{
		if((I2C2->ISR & (1 << 1))) {
			break;
		}
		
		if((I2C2->ISR & (1 << 4))) {
			while(1) {
			}
		}
	}
	
	//Set transmit data
	I2C2->TXDR = 0xA8;
	
	//Wait for transmit complete 
	while(!(I2C2->ISR & (1 << 6)))
	{
	}
	
	//read
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
  I2C2->CR2 |= ((1 << 16) | (0x69 << 1)); // Set SADD = 0x69 and NBYTES = 1
  I2C2->CR2 |= (1 << 10); //Set write mode
  I2C2->CR2 |= (1 << 13); //Set start bit
	
	//Wait for RXNE Flag
	while(1)
	{
		if((I2C2->ISR & (1 << 2))) {
			break;
		}
		if((I2C2->ISR & (1 << 4))) {
			while(1) {
			}
		}
	}
	
	DataX |= I2C2->RXDR;
	
	//Wait for RXNE Flag
	while(1)
	{
		if((I2C2->ISR & (1 << 2))) {
			break;
		}
		
		if((I2C2->ISR & (1 << 4))) {
			while(1) {
			}
		}
	}
	
	DataX |= ((I2C2->RXDR) << 8);
	
	//Wait for the transmit complete flag
	while(!(I2C2->ISR & (1 << 6)))
	{
	}
	//Send stop bit
	I2C2->CR2 |= (1 << 14);
	//Wait for stop condition 
  while(I2C2->CR2 & (1 << 14))
  {
  }

  //Clear the stop bit
  I2C2->CR2 &= ~(1 << 14);
	DataY = 0;

	//write
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
  I2C2->CR2 |= ((1 << 16) | (0x69 << 1)); // Set SADD = 0x69 and NBYTES = 1
  I2C2->CR2 |= (1 << 10); //Set write mode
  I2C2->CR2 |= (1 << 13); //Set start bit
	
	//Wait for TXIS flag
	while(1)
	{
		if((I2C2->ISR & (1 << 1))) {
			break;
		}
		
		if((I2C2->ISR & (1 << 4))) {
			while(1) {
			}
		}
	}
	
	//Set transmit data
	I2C2->TXDR = 0xAA;
	//wait transmit complete
	while(!(I2C2->ISR & (1 << 6)))
	{
	}
	
	//READ
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
  I2C2->CR2 |= ((1 << 16) | (0x69 << 1)); // Set SADD = 0x69 and NBYTES = 1
  I2C2->CR2 |= (1 << 10); //Set write mode
  I2C2->CR2 |= (1 << 13); //Set start bit
	
	//Wait for RXNE Flag
	while(1)
	{
		if((I2C2->ISR & (1 << 2))) {
			break;
		}
		
		if((I2C2->ISR & (1 << 4))) {
			while(1) {
			}
		}
	}
	
	DataY |= I2C2->RXDR;
	
	//Wait for the RXNE Flag
	while(1)
	{
		if((I2C2->ISR & (1 << 2))) {
			break;
		}
		
		if((I2C2->ISR & (1 << 4))) {
			while(1) {
			}
		}
	}
	
	DataY |= ((I2C2->RXDR) << 8);
	
	//Wait transmit complete
	while(!(I2C2->ISR & (1 << 6)))
	{
	}
	
	//Send the stop bit
	I2C2->CR2 |= (1 << 14);
	
	//Wait for stop condition
  while(I2C2->CR2 & (1 << 14))
  {    
  }
	
  //Clear the stop bit
  I2C2->CR2 &= ~(1 << 14);
	
		if(DataX > 5000) {
			GPIOC->ODR |= (1 << 9);
			GPIOC->ODR &= ~(1 << 8);
		} else if(DataX < -5000) {
			GPIOC->ODR |= (1 << 8);
			GPIOC->ODR &= ~(1 << 9);
		} else {
			GPIOC->ODR &= ~(1 << 8);
			GPIOC->ODR &= ~(1 << 9);
		}
		if(DataY > 5000) {
			GPIOC->ODR |= (1 << 6);
			GPIOC->ODR &= ~(1 << 7);
		} else if(DataY < -5000) {
			GPIOC->ODR |= (1 << 7);
			GPIOC->ODR &= ~(1 << 6);
		} else {
			GPIOC->ODR &= ~(1 << 6);
			GPIOC->ODR &= ~(1 << 7);
		}
		
		HAL_Delay(100);
  }
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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
volatile int32_t count;
volatile char recieved;
/* USER CODE BEGIN PV */

/* USER CODE END PV */



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void Transmit_char(char x);

void Transmit_string(char x[]);
void USART3_4_IRQnHandler(void);
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
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable GPIOC clock for LEDs pins.
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable GPIOB clock for USART pins.

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
	
	//set pb10 and pb11 to af mode
	GPIOB->MODER |= (1 << 21) | (1 << 23);
	
	//set pb10 and pb11 to af4 in afrh
	GPIOB->AFR[1] |= (1<<10) | (1 << 14);
	
	//enable sys clock to uart3 in rcc
	RCC->APB1ENR |= (1<<18);
	
	//enable transmiter and reciever AND enable usart at bit 0!
	USART3->CR1 |= (1<<2) | (1<<3) | (1 << 0);
	
	//n the USART initialization, enable the receive register not empty interrupt.
	USART3->CR1 |= (1<<5);
	
	//enable red led
	GPIOC->MODER = (1<<14) | (1<<12) | (1<<16) | (1<<18);

	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 111);
	count = 0;
	
	USART3->BRR =HAL_RCC_GetHCLKFreq()/115200;
	
	int counter = 0;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	
		char recieved1 = 0;		
		char recieved2 = 0;
		int check = 0;
		while(check == 0) { 
			check = USART3->ISR &= (1<<5);
			HAL_Delay(10);
		}
		check = 0;
		recieved1 = USART3->RDR; 
		while(check == 0) { 
			check = USART3->ISR &= (1<<5);
			HAL_Delay(10);
		}
		check = 0;
		recieved2 = USART3->RDR; // read user input
		if(recieved1== 'r'){
			if(recieved2 == '1'){ // turn on
				GPIOC->ODR |= (1<<6);
			}
			if(recieved2 == '2'){
				GPIOC->ODR &=~ (1<<6); // turn off 
			}
		}
		else if(recieved1== 'b'){
			if(recieved2 == '1'){ 
				GPIOC->ODR |= (1<<7);
			}
			if(recieved2 == '2'){
				GPIOC->ODR &=~ (1<<7);
			}
		}
		else if(recieved1== 'o'){
			if(recieved2 == '1'){ 
				GPIOC->ODR |= (1<<8);
			}
			if(recieved2 == '2'){
				GPIOC->ODR &=~ (1<<8); 
			}
		}
		else if(recieved1== 'g'){
			if(recieved2 == '1'){ 
				GPIOC->ODR |= (1<<9);
			}
			if(recieved2 == '2'){
				GPIOC->ODR &=~ (1<<9);
			}		
		}
		recieved1 =0;
		recieved2 =0;
		HAL_Delay(10);
	}
		
  }
  /* USER CODE END 3 */


 void USART3_4_IRQnHandler(void) {
	 if(USART3->ISR & (1<<5)) {
		 recieved = (USART3 -> RDR);
		 count++;
	 }
}

void Transmit_char(char x) {
	while(!(USART3->ISR & (1 << 7))) {		
	}
	USART3->TDR = x;
}

void Transmit_string(char x[]) {
	int i = 0;
	while(!(x[i] == '\0')){
		Transmit_char(x[i]);
		i++;
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



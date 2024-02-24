/* USER CODE BEGIN Header */
/**
  **********
  * @file           : main.c
  * @brief          : Main program body
  **********
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **********
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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
	 // Function to read a character from USART3
char USART3_ReadChar(void) {
    while (!(USART3->ISR & USART_ISR_RXNE));  // Wait for RXNE (Receive data register not empty)
    return USART3->RDR;  // Read the received data
}
void USART3_WriteChar(char ch) {
    while (!(USART3->ISR & USART_ISR_TXE));  // Wait for TXE (Transmit data register empty)
    USART3->TDR = ch;  // Transmit the character
}
// Function to send a string via USART3
void USART3_SendString(const char *str) {
    while (*str) {
        USART3_WriteChar(*str++);
    }
} 

#include "main.h"
#include "stdio.h"

int main(void)
{
  HAL_Init();
	/* Configure the system clock */
  SystemClock_Config();
	
	// Enable GPIOA clock
  __HAL_RCC_GPIOC_CLK_ENABLE();
	
  // Enable GPIOC
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// Enable USART3 clock
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	//Enabling pins 6 and 7 (red and blue)
	GPIO_InitTypeDef initc67 = {GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW,GPIO_NOPULL};
  HAL_GPIO_Init(GPIOC, &initc67);
	
	//Enabling pins 8 and 9 (green and orange)
	GPIO_InitTypeDef initc89 = {GPIO_PIN_8 | GPIO_PIN_9, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW,GPIO_NOPULL};
  HAL_GPIO_Init(GPIOC, &initc89);
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
  // Set PC4 and PC5 to Alternate Function mode
  GPIOC->MODER &= ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
	GPIOC->MODER |= (GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1);
	
	// Set alternate function to USART3 (AF1 for PC4 and PC5 on STM32F072)
  GPIOC->AFR[0] &= ~(GPIO_AFRL_AFRL4 | GPIO_AFRL_AFRL5);
  GPIOC->AFR[0] |= (1 << (4 * 4)) | (1 << (5 * 4));
	
	// Setting baud rate.
	USART3->BRR = 8000000/115200;
	
	USART3->CR1 = USART_CR1_TE | USART_CR1_RE; // Enable TX and RX
	
  USART3->CR1 |= USART_CR1_UE; // Enable USART3
	
  while (1)
  {
		USART3_SendString("\nCmd?\n");
    char ch = USART3_ReadChar();  // Read a character from USART3
 
    // Process the received character
		
		if(ch == 'o' || ch == 'O'){
			GPIOC->ODR ^= GPIO_ODR_8; USART3_SendString("\nORANGE LED TURN ON\n");}  // Through If condition if the character entered is 'o' or 'O' the orange LED glows
		else if(ch == 'g' || ch == 'G'){
			GPIOC->ODR ^= GPIO_ODR_9; USART3_SendString("\nGREEN LED TURN ON\n");} // Through elseIf condition if the character entered is 'g' or 'G' the green LED glows
		else if(ch == 'r' || ch == 'R'){
			GPIOC->ODR ^= GPIO_ODR_6; USART3_SendString("\nRED LED TURN ON\n");} // Through elseIf condition if the character entered is 'g' or 'G' the green LED glows
		else if(ch == 'b' || ch == 'B'){
			GPIOC->ODR ^= GPIO_ODR_7; USART3_SendString("\nBLUE LED TURN ON\n");} // Through elseIf condition if the character entered is 'g' or 'G' the green LED glows
		else
		 USART3_SendString("Error: Unrecognized command.\n\n"); 
}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{RCC_OscInitTypeDef RCC_OscInitStruct = {0};
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
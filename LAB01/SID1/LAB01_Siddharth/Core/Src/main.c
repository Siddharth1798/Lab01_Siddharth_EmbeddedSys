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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
	
	
	   HAL_Init();
    // Configure the system clock SystemClock_Config();
    SystemClock_Config();
    // Initialize the HAL GPIO Library

    uint32_t debouncer = 0;
	
	  // Enable GPIOC and GPIOA clocks

    RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN;
     
	  // Configure-PC6-and-PC7-as-output

    GPIOC->MODER |= (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0); // Output mode
    GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7); // Push-pull 
	  GPIOC->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR6 | GPIO_OSPEEDR_OSPEEDR7); // High speed 
	  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7); // No pull-up, no pull-down
	
	  //Configure PA0 as input
	  GPIOA->MODER &= ~(GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1); // Input node 
		GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR0_1); // No pull-up, no pull-down
		
	  // Set PC6 high and PC7 low
	  GPIOC->BSRR = GPIO_BSRR_BS_6;
		GPIOC->BRR = GPIO_BRR_BR_7;
				
		while (1) {

    debouncer = (debouncer << 1);

    // Read PAO input

    if (GPIOA->IDR & GPIO_IDR_0) {
			debouncer |= 0x01;

    }



     if (debouncer == 0xFFFFFFFF) {

        // Triggered when button is steady high }
		 }
     
		 if (debouncer == 0x00000000) {

          // Triggered when button is steady low

     }

     if (debouncer == 0x7FFFFFFF) { 
			 // Triggered once when transitioning to steady high
			 GPIOC->ODR ^= GPIO_ODR_6 | GPIO_ODR_7; 
			 HAL_Delay(3); // Delay 3 milliseconds
		}
	}
}

/*
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

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
int main(void)
{
  // Initialize the system
    HAL_Init();

    // Enable GPIOC and GPIOA clocks
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN;
  	// Enable the SYSCFG clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Configure PC6,PC7,PC8 and PC9 as output
    GPIOC->MODER |= (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0); // Output mode
    GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9); // Push-pull
    GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR6 | GPIO_OSPEEDR_OSPEEDR7 | GPIO_OSPEEDR_OSPEEDR8 | GPIO_OSPEEDR_OSPEEDR9); // Low speed
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7 | GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9); // No pull-up, no pull-down

    // Configure PA0 as input (digital mode)
    GPIOA->MODER &= ~(GPIO_MODER_MODER0);  // Clear bits for PA0
    
	  // Set low-speed setting for PA0
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0);
    
		// Enable pull-down resistor for PA0
    GPIOA->PUPDR |= (2);
		
		// Connect EXTI0 to PA0
    SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI0);

    // Configure EXTI0 to trigger on the rising edge
    EXTI->RTSR |= EXTI_RTSR_TR0;

    // Enable EXTI0 interrupt
    EXTI->IMR |= EXTI_IMR_MR0;

    // Enable EXTI0 interrupt in NVIC (replace IRQn with the appropriate value)
    NVIC_EnableIRQ(EXTI0_1_IRQn);
/*	
    //Part 2.1: Change Priorities of  EXTI interrupt handler and systick to resolve starving interrupt issue
    NVIC_SetPriority(EXTI0_1_IRQn, 1);  
		//NVIC_SetPriority(EXTI0_1_IRQn, 3); // for second part change priority to 3.
	
		
	  //Part 2.2 - Set appropriate priority to systick handler interrupt
  	NVIC_SetPriority(SysTick_IRQn, 2);
*/
		// Set PC9 high
    GPIOC->BSRR = GPIO_BSRR_BS_9;

    while (1) 
			{
        // Toggle PC6 (Red LED)
        GPIOC->ODR ^= GPIO_ODR_6;
        HAL_Delay(650); // Delay 650 milliseconds
      }
		}
volatile uint32_t i;
void EXTI0_1_IRQHandler(void) 	{
	  GPIOC->ODR ^= GPIO_ODR_8 | GPIO_ODR_9;
	/*  for(i=0; i<1500000;i++)
	  {}
	  // Toggle green and orange LEDs
	  GPIOC->ODR ^= GPIO_ODR_8 | GPIO_ODR_9;
	  // Clear flag for input line 0 in the EXTI pending register  */
	  EXTI->PR |= EXTI_PR_PR0;
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

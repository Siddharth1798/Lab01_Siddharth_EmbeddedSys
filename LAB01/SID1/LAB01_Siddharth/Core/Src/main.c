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
	
    // Initialize the system
    HAL_Init();
	  __HAL_RCC_GPIOC_CLK_ENABLE();
    // Enable GPIOC and GPIOA clocks
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN;
  	// Enable the SYSCFG clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  	// Enable TIM2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
			// Enable TIM3 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
		
	  
	  // Configure TIM2 to generate an interrupt at 4 Hz
    TIM2->PSC = 7999;  // Prescaler, results in a timer clock of 4 kHz (8000000 Hz / (7999 + 1))
    TIM2->ARR = 250;   // Auto-reload value, results in an interrupt every 250 ms (1000 / 4 Hz )

	  TIM2->DIER  |= TIM_DIER_UIE; 
  
  	TIM2->CR1  |= TIM_CR1_CEN; 
	
	  GPIOC->ODR |=(1<<8);
		
	  NVIC_EnableIRQ(TIM2_IRQn);
		
				
    // Configure PC6,PC7,PC8 and PC9 as output
   GPIO_InitTypeDef initc89 = {GPIO_PIN_8 | GPIO_PIN_9, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW,GPIO_NOPULL};
	 HAL_GPIO_Init(GPIOC, &initc89);

    

    // 2. Configure GPIO pins for TIM3 CH1 and CH2 as alternate function (AF1 for STM32F0)
		GPIO_InitTypeDef initc67 = {GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_LOW,GPIO_NOPULL};

		HAL_GPIO_Init(GPIOC, &initc67);

    GPIOC->AFR[0]=0;
		
		
    // Configure TIM3 without enabling or starting the timer
    TIM3->PSC = 7;           // Prescaler, adjust for your requirements
    TIM3->ARR = 1250;          // Auto-reload value, determines PWM frequency (800 Hz)

    TIM3->CCMR1 &= ~(TIM_CCMR1_CC1S);
		TIM3->CCMR1 &= ~(TIM_CCMR1_CC2S);
		
		TIM3->CCMR1 |= (TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);
		TIM3->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);

    // Configure output channels for PWM mode
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE;  // PWM Mode 2 for channel 1
   // TIM3->CCMR1 |= 0;                    // PWM Mode 1 for channel 2
    TIM3->CCMR1 |= TIM_CCMR1_OC2PE;    // Enable output compare preload for channels 1 & 2
   
  	// Enable output channels
    TIM3->CCER |= TIM_CCER_CC1E;
    TIM3->CCER |= TIM_CCER_CC2E;

    // Set duty cycle (e.g., 20% of ARR)
    TIM3->CCR1 = 1125;  // 20% duty cycle for CH1
    TIM3->CCR2 = 112; // 20% duty cycle for CH2

	    // Enable TIM3 counter
    TIM3->CR1 |= TIM_CR1_CEN;

    while (1) 
			{
        /*// Toggle PC6 (Red LED)
        GPIOC->ODR ^= GPIO_ODR_6;
        HAL_Delay(650); // Delay 650 milliseconds
				*/
      }
		}
		void TIM2_IRQHandler(void)
		{
			 if (TIM2->SR & TIM_SR_UIF) { // Check update interrupt flag
        GPIOC->ODR ^= (1 << 8) | (1 << 9); // Toggle green (PC8) and orange (PC9) LEDs
        TIM2->SR &= ~TIM_SR_UIF; // Clear update interrupt flag
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

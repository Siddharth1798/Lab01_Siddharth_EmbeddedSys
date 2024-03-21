/* USER CODE BEGIN Header */
/**
  **************************
  * @file           : main.c
  * @brief          : Main program body
  **************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **************************
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
 
#include "stdio.h"

#include "stm32f0xx_hal.h"


// Gyroscope registers
#define CTRL_REG1 0x20
#define OUT_X_L   0x28
#define OUT_X_H   0x29
#define OUT_Y_L   0x2A
#define OUT_Y_H   0x2B

// LED pins
#define LED_GREEN GPIO_PIN_9
#define LED_ORANGE GPIO_PIN_8
#define LED_RED GPIO_PIN_7
#define LED_BLUE GPIO_PIN_6

I2C_HandleTypeDef hi2c1;

void Gyro_Init(void) {
    uint8_t ctrl_reg1;

    // Enable X and Y axes
    ctrl_reg1 = 0x38;
    HAL_I2C_Mem_Write(&hi2c1, 0xD6, CTRL_REG1, I2C_MEMADD_SIZE_8BIT, &ctrl_reg1, 1, 1000);

    // Set normal mode
    ctrl_reg1 = 0x08;
    HAL_I2C_Mem_Write(&hi2c1, 0xD6, CTRL_REG1, I2C_MEMADD_SIZE_8BIT, &ctrl_reg1, 1, 1000);
}

void Read_Gyro(int16_t *x_data, int16_t *y_data) {
    uint8_t x_low, x_high, y_low, y_high;

    // Read X axis data
    HAL_I2C_Mem_Read(&hi2c1, 0xD6, OUT_X_L, I2C_MEMADD_SIZE_8BIT, &x_low, 1, 1000);
    HAL_I2C_Mem_Read(&hi2c1, 0xD6, OUT_X_H, I2C_MEMADD_SIZE_8BIT, &x_high, 1, 1000);
    *x_data = (x_high << 8) | x_low;

    // Read Y axis data
    HAL_I2C_Mem_Read(&hi2c1, 0xD6, OUT_Y_L, I2C_MEMADD_SIZE_8BIT, &y_low, 1, 1000);
    HAL_I2C_Mem_Read(&hi2c1, 0xD6, OUT_Y_H, I2C_MEMADD_SIZE_8BIT, &y_high, 1, 1000);
    *y_data = (y_high << 8) | y_low;
}
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

 
  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */
  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  // Enable GPIOB
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
 
  // Enable GPIOC
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	 // Configure PB11 in alternate function mode
    GPIOB->MODER &= ~(GPIO_MODER_MODER11);  // Clear the bits
    GPIOB->MODER |= (GPIO_MODER_MODER11_1); // Set to alternate function mode

    // Configure PB11 as open-drain output
    GPIOB->OTYPER |= GPIO_OTYPER_OT_11;

    // Configure the alternate function for PB11 (assuming it's AF4 for I2C2_SDA, check your datasheet/reference manual)
    GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL11; // Clear the bits
    GPIOB->AFR[1] |= (1 << GPIO_AFRH_AFSEL11_Pos); // Set AF4 for I2C2_SDA

    // Configure PB13 in alternate function mode
    GPIOB->MODER &= ~(GPIO_MODER_MODER13);  // Clear the bits
    GPIOB->MODER |= (GPIO_MODER_MODER13_1); // Set to alternate function mode

    // Configure PB11 as open-drain output
    GPIOB->OTYPER |= GPIO_OTYPER_OT_13;

    // Configure the alternate function for PB13 (assuming it's AF4 for I2C2_SDA, check your datasheet/reference manual)
    GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL13; // Clear the bits
    GPIOB->AFR[1] |= (5 << GPIO_AFRH_AFSEL13_Pos); // Set AF4 for I2C2_SCL
 
    // Configure PB14 in general purpose output mode
    GPIOB->MODER &= ~GPIO_MODER_MODER14;  // Clear the bits
    GPIOB->MODER |= GPIO_MODER_MODER14_0; // Set to general purpose output mode

    // Configure PB14 as push-pull output
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT_14;

    // Initialize/set PB14 high
    GPIOB->BSRR |= GPIO_BSRR_BS_14;
		
		// Configure PC0 in general purpose output mode
    GPIOC->MODER &= ~GPIO_MODER_MODER0;  // Clear the bits
    GPIOC->MODER |= GPIO_MODER_MODER0_0; // Set to general purpose output mode

    // Configure PC0 as push-pull output
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT_0;

    // Initialize/set PC0 high
    GPIOC->BSRR |= GPIO_BSRR_BS_0;

    // Enable I2C2
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	  // Set the parameters for 100 kHz standard-mode in TIMINGR register
    // Assuming a 16 MHz system clock, adjust the values accordingly
    I2C2->TIMINGR = (0x1 << 28) | (0x13 << 0) | (0xF << 8) | (0x2 << 16) | (0X4 << 20);

    // Enable the I2C2 peripheral using the PE bit in the CR1 register
    I2C2->CR1 |= I2C_CR1_PE;
		
	 // Configure-PC6-and-PC7-as-output

    GPIOC->MODER |= (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0); // Output mode
    GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7); // Push-pull 
	  GPIOC->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR6 | GPIO_OSPEEDR_OSPEEDR7); // High speed 
	  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7); // No pull-up, no pull-down
	
  	// Configure-PC8-and-PC9-as-output

    GPIOC->MODER |= (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0); // Output mode
    GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9); // Push-pull 
	  GPIOC->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR8 | GPIO_OSPEEDR_OSPEEDR9); // High speed 
	  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9); // No pull-up, no pull-down
	
    GPIOC->ODR ^= GPIO_ODR_8;
	  GPIOC->ODR ^= GPIO_ODR_9;
    GPIOC->ODR ^= GPIO_ODR_6;
	  GPIOC->ODR ^= GPIO_ODR_7;	
		
		
		
		// Step 1: Set transaction parameters in CR2 register
    I2C2->CR2 = (0x69 << 1) | (1 << 16) | I2C_CR2_START;
     
    // Step 2: Wait until either TXIS or NACKF flags are set
    while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))) {
      // Wait
    }

    // If NACKF flag is set, there might be an error
    if (I2C2->ISR & I2C_ISR_NACKF) {
     GPIOC->ODR ^= GPIO_ODR_6;   // Handle error (e.g., print error message or set LED pattern)   
    }

    // Step 3: Write the address of the "WHO_AM_I" register into TXDR
    I2C2->TXDR = 0x0F;

    // Step 4: Wait until TC flag is set
    while (!(I2C2->ISR & I2C_ISR_TC)) {
        // Wait
    }

    // Step 5: Reload CR2 register with read operation parameters and set START bit
    I2C2->CR2 = (0x69 << 1) | (1 << 16) | I2C_CR2_RD_WRN | I2C_CR2_START;

    // Step 6: Wait until either RXNE or NACKF flags are set
    while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF))) {
        // Wait
    }

    // If NACKF flag is set, there might be an error
    if (I2C2->ISR & I2C_ISR_NACKF) {
        GPIOC->ODR ^= GPIO_ODR_8; // Handle error (e.g., print error message or set LED pattern)
    }

    // Step 7: Wait until TC flag is set
    while (!(I2C2->ISR & I2C_ISR_TC)) {
        // Wait
    }

    // Step 8: Check the contents of RXDR register
    uint8_t received_data = I2C2->RXDR;
    if (received_data == 0xD3) {
        // WHO_AM_I register value matches the expected value
       GPIOC->ODR ^= GPIO_ODR_7;  // Proceed with further actions (e.g., print message or set LED pattern)
    } else {
        // WHO_AM_I register value doesn't match the expected value
       GPIOC->ODR ^= GPIO_ODR_9;  // Handle error (e.g., print error message or set LED pattern)
    }

    // Step 9: Set the STOP bit in CR2 register
    I2C2->CR2 |= I2C_CR2_STOP;  

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
		
   int16_t x_data, y_data;
    uint32_t led_state = 0;

// Initialize gyroscope
    Gyro_Init();
		
   // Initialize I2C
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00300F38; // Set appropriate timing value
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        // Error handling
			  Error_Handler();
    }
   // Enable I2C peripheral clock
    __HAL_RCC_I2C1_CLK_ENABLE();

    while (1) {
        Read_Gyro(&x_data, &y_data);
 // Clear previous LED state
        led_state = 0;

        // Update LED state based on gyroscope data
        if (x_data > 500) {
            led_state |= LED_ORANGE;
        } else if (x_data < -500) {
            led_state |= LED_RED;
        }

        if (y_data > 500) {
            led_state |= LED_GREEN;
        } else if  (y_data < -500) {
            led_state |= LED_BLUE;
        }

        // Set LED state
        GPIOC->BSRR = (led_state << 16) | (~led_state & 0);

        // Delay for 100 ms
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
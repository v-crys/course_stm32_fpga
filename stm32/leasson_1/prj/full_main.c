
#include "main.h"
#include "usb_device.h"

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

void led_indic(int type);
char check_btn();

int main(void)
{

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USB_DEVICE_Init();

  led_indic(1);
  HAL_Delay(1000);
  while (1)
  {
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);

	  HAL_Delay(1000);
	  led_indic(0);

	  if ((check_btn() & 1) != 0) led_indic(2);
	  if ((check_btn() & 2) != 0) led_indic(1);
  }

}

char check_btn() {
	char res = 0;
	char tmp = 0;

	tmp |= ~HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) & 1;
	tmp |= (~HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) & 1) << 1;
	res = tmp;
	HAL_Delay(5);
	tmp ^= ~HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) & 1;
	tmp ^= (~HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) & 1) << 1;

	res &= ~tmp;

	return res;
}

void led_indic(int type) {
	static int loc_type = 1;
	static int last_led = 0;


	static int led_arr[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	if (type != 0)	loc_type	=	type;

	switch (loc_type){
	case 1:
		if (type) {
			for ( int i = 0; i < 8; i++ ) led_arr[i] = 0;
			led_arr[ 0 ] = 1;
		} else {
			int tmp = led_arr[ 0 ];
			for ( int i = 0; i < 7; i++ )
				led_arr[ i ] = led_arr[i + 1 ];
			led_arr[7] = tmp;
		}

		break;

	case 2:
		if (type) {
			for ( int i = 0; i < 8; i++ ) led_arr[i] = 1;
			led_arr[ 0 ] = 0;
		} else {
			int tmp = led_arr[ 0 ];
			for ( int i = 0; i < 7; i++ )
				led_arr[ i ] = led_arr[i + 1 ];
			led_arr[7] = tmp;
		}
		break;
	}

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, led_arr[0]);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, led_arr[1]);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, led_arr[2]);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, led_arr[3]);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, led_arr[4]);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, led_arr[5]);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, led_arr[6]);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, led_arr[7]);
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 
                           PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 PC9 
                           PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
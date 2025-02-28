/*
 *  INA226 Driver Source File
 *
 *  Creates current based on shunt and bus voltage based on calibration register.
 *
 *  Bryan Gonzalez
 *
 *
 */
/* Includes ------------------------------------------------------------------*/
#include <INA226.h>
#include <math.h>



/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
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
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00B07CB4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_Test_GPIO_Port, LED_Test_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ACC_INT_Pin */
  GPIO_InitStruct.Pin = ACC_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ACC_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Test_Pin */
  GPIO_InitStruct.Pin = LED_Test_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Test_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */


uint8_t INA226_Initialize(INA226 *dev, I2C_HandleTypeDef *i2cHandle, uint16_t maxCurrent,uint16_t shuntResistance ){
	//commented out register ig I don't care what they are
	dev->i2cHandle = i2cHandle;
	//dev->i2cAddress;
	dev->shuntVoltage = 0.0f;
	dev->busVoltage = 0.0f;
	dev->power = 0.0f;
	dev->current = 0.0f;
	//Current LSB = (Maximum Expected Current)/2^15
	dev->current_LSB = (maxCurrent)/pow(2,15);
	dev->rShunt = shuntResistance;

	//store # of errors to check for issues
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	//Check device manufacturing and DIE ID

	uint16_t regData;


	//CHECK IF CORRECT PART

	status = INA226_ReadRegister(dev,INA226_MANUF_ID_REG , regData);
	//Check the possible statuses if necessary
	errNum += (status != HAL_OK);

	if(regData != INA226_MANUF_ID){
		//leave since ID doesn't match
		return 255;
	}

	status = INA226_ReadRegister(dev,INA226_DIE_ID_REG , regData);
	//Check the possible statuses if necessary
	errNum += (status != HAL_OK);

	if(regData != INA226_DIE_ID){
		//leave since ID doesn't match
		return 255;
	}


	//INITIALIZE NECESSARY COMPONENT REGISTERS

	//Configutation Register: Sets different measuring parameters (Page 22-23)
	  // Bit(B)15 -> reset, B11-B9 -> determines average # of samples taken, B8-6 -> Bus Voltage Conversion Time(CT)
	  // B5-B3 -> Shunt Voltage CT, B2-B0 -> Operating Mode (probing timeframe)

	//starts as 0b0100000100100111;


	//Calibration Register starts at 0x0

	//calibration register value (page 15)
	regData = 0.00512/(current_LSB * Rshunt);
	status = INA226_WriteRegister(dev, INA226_CALIB_REG, regData);
	errNum += (status != HAL_OK);
}



//Low Level Functions

HAL_StatusTypeDef INA226_ReadRegister(INA226 *dev, uint8_t reg, uint16_t *data){
	//Contains: Handler,register which is getting read, memory address size, pointer where to store data,how many bytes to read, timeout duration.
	return HAL_I2C_Mem_Read(dev -> i2cHandle, INA226_I2C_ADDR, reg,I2C_MEMADD_SIZE_8BIT,data,1,HAL_MAX_DELAY);
}

HAL_StatusTypeDef INA226_ReadRegisters(INA226 *dev, uint8_t reg, uint16_t *data, uint8_t length){
	//The same as rearRegister but reads multiply bytes.
	return HAL_I2C_Mem_Read(dev -> i2cHandle, INA226_I2C_ADDR, reg,I2C_MEMADD_SIZE_8BIT,data,length,HAL_MAX_DELAY);
}
HAL_StatusTypeDef INA226_WriteRegister(INA226 *dev, uint8_t reg, uint16_t *data){
	return HAL_I2C_Mem_Write(dev -> i2cHandle, INA226_I2C_ADDR, reg,I2C_MEMADD_SIZE_8BIT,data,1,HAL_MAX_DELAY);
}


HAL_StatusTypeDef INA226_READ_CONFIG_REG(ina226 *dev){
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;
	uint16_t regData;
	status = INA226_ReadRegister(dev,INA226_CONFIG_REG , regData);
	errNum += (status != HAL_OK);
	return regData;
}
HAL_StatusTypeDef INA226_READ_SHUNT_VOLT_REG(ina226 *dev){
	HAL_StatusTypeDef status;
	uint16_t regData;
	status = INA226_ReadRegister(dev,INA226_SHUNT_VOLT_REG , regData);
	return regData;
}
HAL_StatusTypeDef INA226_READ_BUS_VOLT_REG(ina226 *dev){
	HAL_StatusTypeDef status;
	uint16_t regData;
	status = INA226_ReadRegister(dev,INA226_BUS_VOLT_REG , regData);
	return regData;
}
HAL_StatusTypeDef INA226_READ_POWER_REG(ina226 *dev){
	HAL_StatusTypeDef status;
	uint16_t regData;
	status = INA226_ReadRegister(dev,INA226_POWER_REG , regData);
	return regData;
}
HAL_StatusTypeDef INA226_READ_CURRENT_REG(ina226 *dev){
	HAL_StatusTypeDef status;
	uint16_t regData;
	status = INA226_ReadRegister(dev,INA226_CURRENT_REG, regData);
	return regData;
}
HAL_StatusTypeDef INA226_READ_CALIB_REG(ina226 *dev){
	HAL_StatusTypeDef status;
	uint16_t regData;
	status = INA226_ReadRegister(dev,INA226_CALIB_REG , regData);
	return regData;
}
HAL_StatusTypeDef INA226_READ_MASK_EN_REG(ina226 *dev){
	HAL_StatusTypeDef status;
	uint16_t regData;
	status = INA226_ReadRegister(dev,INA226_MASK_EN_REG , regData);
	return regData;
}
HAL_StatusTypeDef INA226_READ_ALERT_LIMIT_REG(ina226 *dev){
	HAL_StatusTypeDef status;
	uint16_t regData;
	status = INA226_ReadRegister(dev,INA226_ALERT_LIMIT_REG, regData);
	return regData;
}
//HAL_StatusTypeDef INA226_READ_MANUF_ID_REG(ina226 *dev);
//HAL_StatusTypeDef INA226_READ_DIE_ID_REG(ina226 *dev);

// return current value after multiplication
uint16_t getCurrentAmp(INA226 *dev){
	HAL_StatusTypeDef status;
	uint16_t regData;
	uint16_t currentData;
	status = INA226_ReadRegister(dev,INA226_CALIB_REG , regData);
	currentData = regData * dev->current_LSB;
	return currentData;
}



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

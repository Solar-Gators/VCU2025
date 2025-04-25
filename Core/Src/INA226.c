/*
 *  INA226 Driver Source File
 *
 *  Calculate current and power based on bus/shunt voltages. Calculate Calibration
 *  register value and current_LSB, both of which are used to find current and power.
 *  Use current_LSB to recalculate register value is Amperes or Watss respectively.
 *
 *  Bryan Gonzalez
 */
/* Includes ------------------------------------------------------------------*/
#include <INA226.h>
#include <stdio.h>
#include <math.h>

//Initialize INA226 Component using max current expected and resistance of shunt resistor.
HAL_StatusTypeDef  INA226_Initialize(INA226_t *dev, I2C_HandleTypeDef *i2cHandle, float maxCurrent, float shuntResistance ){
	dev->i2cHandle = i2cHandle;
	dev->config = 0;
	dev->shuntVoltage = 0; //max is 81.92mV
	dev->busVoltage = 0;
	dev->power = 0;
	dev->current = 0;
	dev->calibration = 0;

	//Current LSB = (Maximum Expected Current)/2^15
	dev->current_LSB = (maxCurrent) / pow(2, 15); // 10A
	dev->rShunt = shuntResistance; //20 mOHM

	//store # of errors to check for issues
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	uint16_t regData;
	//float CAL, we are truncating for register so it doesn't matter.
	uint16_t CAL;


	//Check device manufacturing and DIE ID


	status = INA226_ReadRegister(dev,INA226_MANUF_ID_REG , &regData);

	if(regData != INA226_MANUF_ID){
		//leave since ID doesn't match
		return 255;
	}

	status = INA226_ReadRegister(dev,INA226_DIE_ID_REG, &regData);

	if(regData != INA226_DIE_ID){
		//leave since ID doesn't match
		return 255;
	}


	//INITIALIZE NECESSARY COMPONENT REGISTERS

	//Configutation Register: Sets different measuring parameters (Page 22-23)
	  // Bit(B)15 -> reset, B11-B9 -> determines average # of samples taken, B8-6 -> Bus Voltage Conversion Time(CT)
	  // B5-B3 -> Shunt Voltage CT, B2-B0 -> Operating Mode (probing timeframe)

	status = INA226_ReadRegister(dev,INA226_CONFIG_REG , &regData);
	errNum += (status != HAL_OK);
	dev->config = regData;


	//calibration register value (page 15)
	CAL = (0.00512)/(dev->current_LSB * shuntResistance);
	status = INA226_WriteRegister(dev, INA226_CALIB_REG, &CAL);
	status = INA226_ReadRegister(dev,INA226_CALIB_REG , &regData);
	errNum += (status != HAL_OK);
	dev->calibration = regData;

	return HAL_OK;
}



//Low Level Functions

HAL_StatusTypeDef INA226_ReadRegister(INA226_t *dev, uint8_t reg, uint16_t *data){
    // Read 2 bytes (16 bits) from the register
	uint8_t temp[2];
	HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(dev->i2cHandle, INA226_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,temp, 2, HAL_MAX_DELAY);
    //temp is full rn
    uint16_t alldata;

    //Index first, beacause it reads LSB first
    alldata = (uint16_t)temp[0];

    //shift regdata left 8 so # is xxxxxxxx00000000
    alldata = (alldata << 8);
    alldata = alldata | (uint16_t)temp[1];
    *data = alldata;
    return status;
}

HAL_StatusTypeDef INA226_WriteRegister(INA226_t *dev, uint8_t reg, uint16_t *data){
    // Write 2 bytes (16 bits) to the specified register
	uint16_t passData = ((*data >> 8) | (*data << 8));
	HAL_StatusTypeDef status;



	//Pass in a pointer to the 16 bit # as an 8 bit pointer, but use length 2 to write 2 bits.
    status = HAL_I2C_Mem_Write(dev->i2cHandle, INA226_I2C_ADDR, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&passData, 2, HAL_MAX_DELAY);
    return status;
}

// return current value after multiplication
uint16_t getCurrentAmp(INA226_t *dev){
	uint16_t regData;
	uint16_t currentData;
	float rawVoltage;
	INA226_ReadRegister(dev, INA226_SHUNT_VOLT_REG, &regData);
	rawVoltage = (float)regData * 81.82 / 32768;
	currentData = (uint16_t)(rawVoltage/0.02);
	return currentData;
}

// return power value after multiplication
uint16_t getPowerWatt(INA226_t *dev){
	uint16_t regData;
	uint16_t powerData;
	INA226_ReadRegister(dev, INA226_POWER_REG,&regData);
	powerData = regData * dev->current_LSB;
	return powerData;
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

/*
 *  INA226 Driver Header File
 *
 *  Bryan Gonzalez
 *
 *
 */

#ifndef INA226_I2C_DRIVER_H
#define INA226_I2C_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h" //Needed for I2C

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define INA226_I2C_ADDR (0b1000100 << 1)// A0, A1 pins -> 0x40 for GND (page 18)
	//left shift address because address is 7 bits, w/ 8th bit being R/W bit, which has to be free.
#define INA226_MANUF_ID 0x5449
#define INA226_DIE_ID 0x2260
	//register map (page 22), current register is listed.


//Registers (page 22)

#define INA226_CONFIG_REG 0x00
#define INA226_SHUNT_VOLT_REG 0x01
#define INA226_BUS_VOLT_REG 0x02
#define INA226_POWER_REG 0x03
#define INA226_CURRENT_REG 0x04
#define INA226_CALIB_REG 0x05
#define INA226_MASK_EN_REG 0x06
#define INA226_ALERT_LIMIT_REG 0x07
#define INA226_MANUF_ID_REG 0xFE
#define INA226_DIE_ID_REG 0xFF

typedef struct{
	I2C_HandleTypeDef *i2cHandle;
	double current_LSB;
	float rShunt;
	uint16_t config;
	uint16_t shuntVoltage;
	uint16_t busVoltage;
	uint16_t power;
	uint16_t current;
	uint16_t calibration;
}INA226_t;

//INA226 pointer to struct,pointer of I2C handler
HAL_StatusTypeDef INA226_Initialize(INA226_t *dev, I2C_HandleTypeDef *i2cHandle,float maxCurrent,float shuntResistance);



//Low-Level Functions:

//Parameters: struct pointer, 8-bit address to read from, 16-bit where data is stored
HAL_StatusTypeDef INA226_ReadRegister(INA226_t *dev, uint8_t reg, uint16_t *data);

//last 2 parameters are switched
HAL_StatusTypeDef INA226_WriteRegister(INA226_t *dev, uint8_t reg, uint16_t *data);


//High-Level Functions:
uint16_t getCurrentAmp(INA226_t *dev);
uint16_t getPowerWatt(INA226_t *dev);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ACC_INT_Pin GPIO_PIN_1
#define ACC_INT_GPIO_Port GPIOA
#define ACC_INT_EXTI_IRQn EXTI1_IRQn
#define LED_Test_Pin GPIO_PIN_4
#define LED_Test_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif

#include "ADXL355.h"

uint8_t ADXL355_Initialise( ADXL355 *dev, I2C_HandleTypeDef *i2cHandle ) {

	/* Set struct parameters */
	dev->i2cHandle 		= i2cHandle;

	dev->acc_mps2[0]	= 0.0f;
	dev->acc_mps2[1]	= 0.0f;
	dev->acc_mps2[2]	= 0.0f;

	dev->temp_C			= 0.0f;

	/* Store number of transaction errors (to be returned at end of function) */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	/*
	 * Check device, mems, and part IDs (DATASHEET PAGE 32)
	 */
	uint8_t regData;

	status = ADXL355_ReadRegister( dev, ADXL355_REG_DEVID_AD, &regData );
	errNum += ( status != HAL_OK );

	if ( regData != ADXL355_DEVICE_ID ) {

		return 255;

	}

	status = ADXL355_ReadRegister( dev, ADXL355_REG_DEVID_MST, &regData );
	errNum += ( status != HAL_OK );

	if ( regData != ADXL355_MEMS_ID ) {

		return 255;

	}

	status = ADXL355_ReadRegister( dev, ADXL355_REG_PARTID, &regData );
	errNum += ( status != HAL_OK );

	if ( regData != ADXL355_PART_ID ) {

		return 255;

	}

	/*
	 * Set output data rate (ODR) and digital filters (no high-pass filter, 125 Hz ODR, 31.25 Hz low-pass filter cut-off) (DATASHEET PAGE 37)
	 */
	regData = 0x05;

	status = ADXL355_WriteRegister( dev, ADXL355_REG_FILTER, &regData);
	errNum += ( status != HAL_OK );

	/*
	 * Put sensor into measurement mode (DATASHEET PAGE 38)
	 */
	regData = 0x00;

	status = ADXL355_WriteRegister( dev, ADXL355_REG_POWER_CTL, &regData);
	errNum += ( status != HAL_OK );

	/* Return number of errors (0 if successful initialisation) */
	return errNum;

}

/*
 * DATA ACQUISITION
 */

HAL_StatusTypeDef ADXL355_ReadTemperature( ADXL355 *dev ) {

	/* DATASHEET PAGE 33 */

	/*
	 * Read raw values from temperature registers (16 bits)
	 */
	uint8_t regData[2];

	HAL_StatusTypeDef status = ADXL355_ReadRegisters( dev, ADXL355_REG_TEMP2, regData, 2 );

	/*
	 * Combine register values to give raw temperature reading (12 bits)
	 */
	uint16_t tempRaw = ( ((regData[0] & 0x0F) << 8) | regData[1] );

	/*
	 * Convert to deg C (offset @ 25degC = 1852 LSB, slope = -9.05 LSB/degC)
	 */
	dev->temp_C = -0.11049723756f * ( (float) tempRaw - 1852.0f ) + 25.0f;

	return status;

}

HAL_StatusTypeDef ADXL355_ReadAccelerations( ADXL355 *dev ) {

	/* DATASHEET PAGE 33 and 34 */

	/*
	 * Read raw values from acceleration registers (x, y, z -> 24 bits each)
	 */
	uint8_t regData[9];

	HAL_StatusTypeDef status = ADXL355_ReadRegisters( dev, ADXL355_REG_XDATA3, regData, 9 );

	/*
	 * Combine register valeus to give raw (UNSIGNED) accelerometer readings (20 bits each)
	 */
	uint32_t accRaw[3];

	accRaw[0] = (uint32_t) (((regData[0] << 16) | (regData[1] << 8) |  (regData[2] & 0xF0)) >> 4) & 0x000FFFFF; /* X-axis */
	accRaw[1] = (uint32_t) (((regData[3] << 16) | (regData[4] << 8) |  (regData[5] & 0xF0)) >> 4) & 0x000FFFFF; /* Y-axis */
	accRaw[2] = (uint32_t) (((regData[6] << 16) | (regData[7] << 8) |  (regData[8] & 0xF0)) >> 4) & 0x000FFFFF; /* Z-axis */

	/* Convert to SIGNED integers (two's complement) */
	int32_t accRawSigned[3];

	if ( (accRaw[0] & 0x00080000) == 0x00080000 ) {

		accRawSigned[0] = accRaw[0] | 0xFFF00000;

	} else {

		accRawSigned[0] = accRaw[0];

	}

	if ( (accRaw[1] & 0x00080000) == 0x00080000 ) {

		accRawSigned[1] = accRaw[1] | 0xFFF00000;

	} else {

		accRawSigned[1] = accRaw[1];

	}

	if ( (accRaw[2] & 0x00080000) == 0x000080000 ) {

		accRawSigned[2] = accRaw[2] | 0xFFF00000;

	} else {

		accRawSigned[2] = accRaw[2];

	}

	/* Convert to mps^2 (given range setting of +-2.048g) */
	dev->acc_mps2[0] = 9.81f * 0.00000390625f * accRawSigned[0];
	dev->acc_mps2[1] = 9.81f * 0.00000390625f * accRawSigned[1];
	dev->acc_mps2[2] = 9.81f * 0.00000390625f * accRawSigned[2];

	return status;

}

/*
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef ADXL355_ReadRegister( ADXL355 *dev, uint8_t reg, uint8_t *data ) {

	return HAL_I2C_Mem_Read( dev->i2cHandle, ADXL355_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY );

}

HAL_StatusTypeDef ADXL355_ReadRegisters( ADXL355 *dev, uint8_t reg, uint8_t *data, uint8_t length ) {

	return HAL_I2C_Mem_Read( dev->i2cHandle, ADXL355_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY );

}

HAL_StatusTypeDef ADXL355_WriteRegister( ADXL355 *dev, uint8_t reg, uint8_t *data ) {

	return HAL_I2C_Mem_Write( dev->i2cHandle, ADXL355_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY );

}

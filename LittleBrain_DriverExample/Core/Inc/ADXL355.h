/*
 *
 * ADXL355 Accelerometer I2C Driver
 *
 * Author:	Philip Salmony
 * Created: 10 August 2021
 *
 */

#ifndef ADXL355_I2C_DRIVER_H
#define ADXL355_I2C_DRIVER_H

#include "stm32f4xx_hal.h" /* Needed for I2C */

/*
 * DEFINES
 */
#define ADXL355_I2C_ADDR	(0x1D << 1) /* ASEL = 0 -> 0x1D, ASEL = 1 -> 0x53 (p. 26) */

#define ADXL355_DEVICE_ID	0xAD
#define ADXL355_MEMS_ID		0x1D
#define ADXL355_PART_ID		0xED

/*
 * REGISTERS (p. 31)
 */
#define ADXL355_REG_DEVID_AD		0x00
#define ADXL355_REG_DEVID_MST		0x01
#define ADXL355_REG_PARTID			0x02
#define ADXL355_REG_REVID			0x03
#define ADXL355_REG_STATUS			0x04
#define ADXL355_REG_FIFO_ENTRIES	0x05
#define ADXL355_REG_TEMP2			0x06
#define ADXL355_REG_TEMP1			0x07
#define ADXL355_REG_XDATA3			0x08
#define ADXL355_REG_XDATA2			0x09
#define ADXL355_REG_XDATA1			0x0A
#define ADXL355_REG_YDATA3			0x0B
#define ADXL355_REG_YDATA2			0x0C
#define ADXL355_REG_YDATA1			0x0D
#define ADXL355_REG_ZDATA3			0x0E
#define ADXL355_REG_ZDATA2			0x0F
#define ADXL355_REG_ZDATA1			0x10
#define ADXL355_REG_FIFO_DATA		0x11
#define ADXL355_REG_OFFSET_X_H		0x1E
#define ADXL355_REG_OFFSET_X_L		0x1F
#define ADXL355_REG_OFFSET_Y_H		0x20
#define ADXL355_REG_OFFSET_Y_L		0x21
#define ADXL355_REG_OFFSET_Z_H		0x22
#define ADXL355_REG_OFFSET_Z_L		0x23
#define ADXL355_REG_ACT_EN			0x24
#define ADXL355_REG_ACT_THRESH_H	0x25
#define ADXL355_REG_ACT_THRESH_L	0x26
#define ADXL355_REG_ACT_COUNT		0x27
#define ADXL355_REG_FILTER			0x28
#define ADXL355_REG_FIFO_SAMPLES	0x29
#define ADXL355_REG_INT_MAP			0x2A
#define ADXL355_REG_SYNC			0x2B
#define ADXL355_REG_RANGE			0x2C
#define ADXL355_REG_POWER_CTL		0x2D
#define ADXL355_REG_SELF_TEST		0x2E
#define ADXL355_REG_ESET			0x2F

/*
 * SENSOR STRUCT
 */

typedef struct {

	/* I2C handle */
	I2C_HandleTypeDef *i2cHandle;

	/* Acceleration data (X, Y, Z) in m/s^2 */
	float acc_mps2[3];

	/* Temperature data in deg */
	float temp_C;

} ADXL355;

/*
 * INITIALISATION
 */
uint8_t ADXL355_Initialise( ADXL355 *dev, I2C_HandleTypeDef *i2cHandle );

/*
 * DATA ACQUISITON
 */

HAL_StatusTypeDef ADXL355_ReadTemperature( ADXL355 *dev );
HAL_StatusTypeDef ADXL355_ReadAccelerations( ADXL355 *dev );

/*
 * LOW-LEVEL FUNCTIONS
 */
HAL_StatusTypeDef ADXL355_ReadRegister(  ADXL355 *dev, uint8_t reg, uint8_t *data );
HAL_StatusTypeDef ADXL355_ReadRegisters( ADXL355 *dev, uint8_t reg, uint8_t *data, uint8_t length );

HAL_StatusTypeDef ADXL355_WriteRegister( ADXL355 *dev, uint8_t reg, uint8_t *data );


#endif

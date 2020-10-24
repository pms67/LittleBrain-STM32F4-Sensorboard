#ifndef SPL_PRESSURE_SENSOR_DRIVER_H
#define SPL_PRESSURE_SENSOR_DRIVER_H

#include "stm32f4xx_hal.h"

#define SPL_PRS_B2 		0x00
#define SPL_PRS_B1 		0x01
#define SPL_PRS_B0 		0x02
#define SPL_TMP_B2 		0x03
#define SPL_TMP_B1 		0x04
#define SPL_TMP_B0 		0x05
#define SPL_PRS_CFG 	0x06
#define SPL_TMP_CFG 	0x07
#define SPL_MEAS_CFG 	0x08
#define SPL_CFG_REG 	0x09
#define SPL_INT_STS 	0x0A
#define SPL_FIFO_STS 	0x0B
#define SPL_RESET 		0x0C
#define SPL_ID 			0x0D
#define SPL_COEF 		0x10

typedef struct {

	/* SPI */
	SPI_HandleTypeDef *spiHandle;
	GPIO_TypeDef 	  *csPinBank;
	uint16_t 		   csPin;

	/* Calibration values */
	int16_t c0, c1;
	int32_t c00, c10;
	int16_t c01, c11, c20, c21, c30;

	/* Measurements */
	float pressurePa;
	float temperatureC;

} SPL;

uint8_t SPL_Init(SPL *bar, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPinBank, uint16_t csPin) {

	uint8_t status = 0;

	/* Store peripheral data */
	bar->spiHandle = spiHandle;
	bar->csPinBank = csPinBank;
	bar->csPin	   = csPin;

	/* Clear measurements */
	bar->pressurePa   = 0.0f;
	bar->temperatureC = 0.0f;

	uint8_t txBuf[2] = {0x00, 0x00};
	uint8_t rxBuf[2];

	/* Check device ID */
	uint8_t id;

	txBuf[0] = SPL_ID | 0x80;

	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_RESET);
	status += (HAL_SPI_TransmitReceive(bar->spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_SET);

	id = rxBuf[1];

	/* Make sure device ID matches */
	if (id != 0x10) {

		return 0;

	}

	HAL_Delay(25);

	/* Read calibration coefficients */
	uint8_t calibTxBuf[19];
	calibTxBuf[0] = (SPL_COEF | 0x80);

	uint8_t calibRxBuf[19];

	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_RESET);
	status += (HAL_SPI_TransmitReceive(bar->spiHandle, calibTxBuf, calibRxBuf, 19, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_SET);

	/* Convert raw calibration coefficients to signed integers */
	bar->c0  = ((uint16_t) calibRxBuf[1] << 4) | ((uint16_t) (calibRxBuf[2] & 0xF0) >> 4);
	bar->c0  = (bar->c0 & 1 << 11) ? (0xF000 | bar->c0) : bar->c0;

	bar->c1  = ((uint16_t) (calibRxBuf[2] & 0x0F) << 8) | ((uint16_t) calibRxBuf[3]);
	bar->c1  = (bar->c1 & 1 << 11) ? (0xF000 | bar->c1) : bar->c1;

	bar->c00  = ((uint32_t) calibRxBuf[4] << 12) | ((uint32_t) calibRxBuf[5] << 4) | ((uint16_t) (calibRxBuf[6] & 0xF0) >> 4);
	bar->c00  = (bar->c00 & 1 << 19) ? (0xFFF00000 | bar->c00) : bar->c00;

	bar->c10  = ((uint32_t) (calibRxBuf[6] & 0x0F) << 16) | ((uint32_t) calibRxBuf[7] << 8) | ((uint16_t) calibRxBuf[8]);
	bar->c10  = (bar->c10 & 1 << 19) ? (0xFFF00000 | bar->c10) : bar->c10;

	bar->c01 = (uint16_t) calibRxBuf[9]  << 8 | calibRxBuf[10];
	bar->c11 = (uint16_t) calibRxBuf[11] << 8 | calibRxBuf[12];
	bar->c20 = (uint16_t) calibRxBuf[13] << 8 | calibRxBuf[14];
	bar->c21 = (uint16_t) calibRxBuf[15] << 8 | calibRxBuf[16];
	bar->c30 = (uint16_t) calibRxBuf[17] << 8 | calibRxBuf[18];

	HAL_Delay(25);

	/* Set pressure configuration */
	txBuf[0] = SPL_PRS_CFG;
	txBuf[1] = 0x33;			/* 8 Hz, 8x oversampling */

	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_RESET);
	status += (HAL_SPI_Transmit(bar->spiHandle, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_SET);

	HAL_Delay(25);

	/* Set temperature configuration */
	txBuf[0] = SPL_TMP_CFG;
	txBuf[1] = 0x00; /* 1 Hz, no oversampling */

	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_RESET);
	status += (HAL_SPI_Transmit(bar->spiHandle, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_SET);

	HAL_Delay(25);

	/* Set measurement configuration */
	txBuf[0] = SPL_MEAS_CFG ;
	txBuf[1] = 0xC7;

	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_RESET);
	status += (HAL_SPI_Transmit(bar->spiHandle, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_SET);

	HAL_Delay(50);

	/* Read measurement configuration */
	txBuf[0] = SPL_MEAS_CFG | 0x80;
	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_RESET);
	status += (HAL_SPI_TransmitReceive(bar->spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_SET);

	uint8_t measConf = rxBuf[1];

	return status;

}

void SPL_Read(SPL *bar) {

	uint8_t txBuf[7];
	txBuf[0] = 0x00 | 0x80;

	uint8_t rxBuf[7];

	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(bar->spiHandle, txBuf, rxBuf, 7, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_SET);

	/* Convert raw to uncalibrated pressure and temperature */
	int32_t pres = ((uint32_t) rxBuf[1] << 16) | ((uint32_t) rxBuf[2] << 8) | ((uint32_t) rxBuf[3]);
			pres = (pres & 1 << 23) ? (0xFF000000 | pres) : pres;

	int32_t temp = ((uint32_t) rxBuf[4] << 16) | ((uint32_t) rxBuf[5] << 8) | ((uint32_t) rxBuf[6]);
			temp = (temp & 1 << 23) ? (0xFF000000 | temp) : temp;

	/* Apply calibration */
	float tempRaw = (float) temp / 524288.0f;
	bar->temperatureC = 0.5f * bar->c0 + bar->c1 * tempRaw;

	float presRaw   = (float) pres / 7864320.0f;
	bar->pressurePa = bar->c00 + presRaw * (bar->c10 + presRaw * (bar->c20 + bar->c30 * presRaw))
				    + tempRaw * (bar->c01 + presRaw * (bar->c11 + bar->c21 * presRaw));

}

#endif

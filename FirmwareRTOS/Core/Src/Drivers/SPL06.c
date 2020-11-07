#include "SPL06.h"

/*
 *
 * INITIALISATION
 *
 */
uint8_t SPL06_Init(SPL06 *bar, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPinBank, uint16_t csPin) {

	uint8_t status = 0;

	/* Store peripheral data */
	bar->spiHandle = spiHandle;
	bar->csPinBank = csPinBank;
	bar->csPin	   = csPin;

	/* Sensor requires LOW on CS pin to put into SPI mode (see datasheet 5.3.2) */
	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_RESET);
	HAL_Delay(10);

	/* Clear measurements */
	bar->pressure_Pa   = 0.0f;
	bar->temperature_C = 0.0f;

	uint8_t txBuf[2] = {0x00, 0x00};
	uint8_t rxBuf[2];

	/* Check device ID */
	uint8_t id;

	txBuf[0] = SPL06_ID | 0x80;

	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_RESET);
	status += (HAL_SPI_TransmitReceive(bar->spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_SET);

	id = rxBuf[1];

	/* Make sure device ID matches */
	if (id != 0x10) {

		return 0;

	}
	HAL_Delay(10);

	/* Read calibration coefficients */
	uint8_t calibTxBuf[19];
	calibTxBuf[0] = (SPL06_COEF | 0x80);

	uint8_t calibRxBuf[19];

	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_RESET);
	status += (HAL_SPI_TransmitReceive(bar->spiHandle, calibTxBuf, calibRxBuf, 19, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_SET);

	/* Convert raw calibration coefficients to signed integers */
	bar->c0 = (uint16_t)calibRxBuf[1] << 4 | (uint16_t)calibRxBuf[2] >> 4;
	bar->c0 = (bar->c0 & 1 << 11) ? (0xF000 | bar->c0) : bar->c0;

	bar->c1 = (uint16_t)(calibRxBuf[2] & 0x0f) << 8 | (uint16_t)calibRxBuf[3];
	bar->c1 = (bar->c1 & 1 << 11) ? (0xF000 | bar->c1) : bar->c1;

	bar->c00 = (uint32_t)calibRxBuf[4] << 12 | (uint32_t)calibRxBuf[5] << 4 | (uint16_t)calibRxBuf[6] >> 4;
	bar->c00 = (bar->c00 & 1 << 19) ? (0xFFF00000 | bar->c00) : bar->c00;

	bar->c10 = (uint32_t)(calibRxBuf[6] & 0x0f) << 16 | (uint32_t)calibRxBuf[7] << 8 | (uint32_t)calibRxBuf[8];
	bar->c10 = (bar->c10 & 1 << 19) ? (0xFFF00000 | bar->c10) : bar->c10;

	bar->c01 = (uint16_t) calibRxBuf[9]  << 8 | calibRxBuf[10];
	bar->c11 = (uint16_t) calibRxBuf[11] << 8 | calibRxBuf[12];
	bar->c20 = (uint16_t) calibRxBuf[13] << 8 | calibRxBuf[14];
	bar->c21 = (uint16_t) calibRxBuf[15] << 8 | calibRxBuf[16];
	bar->c30 = (uint16_t) calibRxBuf[17] << 8 | calibRxBuf[18];
	HAL_Delay(25);

	/* Set pressure configuration */
	txBuf[0] = SPL06_PRS_CFG;
	txBuf[1] = 0x33;			/* 8 Hz, 8x oversampling */

	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_RESET);
	status += (HAL_SPI_Transmit(bar->spiHandle, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_SET);

	HAL_Delay(10);

	/* Set temperature configuration */
	txBuf[0] = SPL06_TMP_CFG;
	txBuf[1] = 0xB3; /* 'external' sensor, 8 Hz, 8x oversampling */

	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_RESET);
	status += (HAL_SPI_Transmit(bar->spiHandle, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_SET);

	HAL_Delay(10);

	/* Set measurement configuration */
	txBuf[0] = SPL06_MEAS_CFG ;
	txBuf[1] = 0xFF; /* Continuous pressure and temperature measurement */

	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_RESET);
	status += (HAL_SPI_Transmit(bar->spiHandle, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_SET);

	return status;

}

/*
 *
 * TEMPERATURE AND PRESSURE READ (POLLING)
 *
 */
void SPL06_Read(SPL06 *bar) {

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
	float tempRaw = (float) temp / 7864320.0f;
	bar->temperature_C = 0.5f * bar->c0 + bar->c1 * tempRaw;

	float presRaw   = (float) pres / 7864320.0f;
	bar->pressure_Pa = bar->c00 + presRaw * (bar->c10 + presRaw * (bar->c20 + bar->c30 * presRaw))
				    + tempRaw * (bar->c01 + presRaw * (bar->c11 + bar->c21 * presRaw));

}

/*
 *
 * TEMPERATURE AND PRESSURE READ (DMA)
 *
 */
uint8_t SPL06_ReadDMA(SPL06 *bar) {

	uint8_t txBuf[7];
	txBuf[0] = 0x00 | 0x80;

	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_RESET);
	if (HAL_SPI_TransmitReceive_DMA(bar->spiHandle, txBuf, (uint8_t *) bar->dmaRxBuf, 7) == HAL_OK) {

		bar->reading = 1;
		return 1;

	} else {

		return 0;

	}

}

void SPL06_ReadDMA_Complete(SPL06 *bar) {

	HAL_GPIO_WritePin(bar->csPinBank, bar->csPin, GPIO_PIN_SET);
	bar->reading = 0;

	/* Convert raw to uncalibrated pressure and temperature */
	int32_t pres = ((uint32_t) bar->dmaRxBuf[1] << 16) | ((uint32_t) bar->dmaRxBuf[2] << 8) | ((uint32_t) bar->dmaRxBuf[3]);
			pres = (pres & 1 << 23) ? (0xFF000000 | pres) : pres;

	int32_t temp = ((uint32_t) bar->dmaRxBuf[4] << 16) | ((uint32_t) bar->dmaRxBuf[5] << 8) | ((uint32_t) bar->dmaRxBuf[6]);
			temp = (temp & 1 << 23) ? (0xFF000000 | temp) : temp;

	/* Apply calibration */
	float tempRaw 	   = (float) temp / 7864320.0f;
	bar->temperature_C = 0.5f * bar->c0 + bar->c1 * tempRaw;

	float presRaw    = (float) pres / 7864320.0f;
	bar->pressure_Pa = bar->c00 + presRaw * (bar->c10 + presRaw * (bar->c20 + bar->c30 * presRaw))
				    + tempRaw * (bar->c01 + presRaw * (bar->c11 + bar->c21 * presRaw));

}

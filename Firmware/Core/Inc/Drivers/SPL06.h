#ifndef SPL06_PRESSURE_SENSOR_DRIVER_H
#define SPL06_PRESSURE_SENSOR_DRIVER_H

#include "stm32f4xx_hal.h"

/* Register defines */
#define SPL06_PRS_B2 	0x00
#define SPL06_PRS_B1 	0x01
#define SPL06_PRS_B0 	0x02
#define SPL06_TMP_B2 	0x03
#define SPL06_TMP_B1 	0x04
#define SPL06_TMP_B0 	0x05
#define SPL06_PRS_CFG 	0x06
#define SPL06_TMP_CFG 	0x07
#define SPL06_MEAS_CFG 	0x08
#define SPL06_CFG_REG 	0x09
#define SPL06_INT_STS 	0x0A
#define SPL06_FIFO_STS 	0x0B
#define SPL06_RESET 	0x0C
#define SPL06_ID 		0x0D
#define SPL06_COEF 		0x10

typedef struct {

	/* SPI */
	SPI_HandleTypeDef *spiHandle;
	GPIO_TypeDef 	  *csPinBank;
	uint16_t 		   csPin;

	/* DMA */
	uint8_t dmaRxBuf[7];
	uint8_t reading;

	/* Calibration values */
	int16_t c0, c1;
	int32_t c00, c10;
	int16_t c01, c11, c20, c21, c30;

	/* Measurements */
	float pressure_Pa;
	float temperature_C;

} SPL06;

/*
 *
 * INITIALISATION
 *
 */
uint8_t SPL06_Init(SPL06 *bar, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPinBank, uint16_t csPin);

/*
 *
 * TEMPERATURE AND PRESSURE READ
 *
 */
void SPL06_Read(SPL06 *bar);

/*
 *
 * TEMPERATURE AND PRESSURE READ (DMA)
 *
 */
uint8_t SPL06_ReadDMA(SPL06 *bar);
void 	SPL06_ReadDMA_Complete(SPL06 *bar);

#endif

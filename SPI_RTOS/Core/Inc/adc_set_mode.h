/*
 * adc_set_mode.h
 *
 *  Created on: Nov 20, 2025
 *      Author: RASBI
 *  Optimized on : 2026 04/04 SAT 16:33 KST
 */

#ifndef INC_ADC_SET_MODE_H_
#define INC_ADC_SET_MODE_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern SPI_HandleTypeDef hspi1;

/* ADC SET MODE */
typedef enum {
	ADC_MODE_STOP,
	ADC_MODE_CH1,
	ADC_MODE_CH2,
	ADC_MODE_CH1_AND_CH2,
	BUSY_STATUS
}ADC_Mode_t;

/* SPI Transfer mode (double buffering) */
typedef enum {
	READY_STATUS,
	SPI_HALF_TRANSFER,
	SPI_FULL_TRANSFER
}SPI_Transfer_Mode_t;

/* EXTI Status define */
typedef enum {
	WAITING_STATUS,
	TRIGGERED,
	REFRESH_FLAG = 0xFFFF
}EXTI_Status_t;

void set_adc_mode(ADC_Mode_t mode, uint16_t* buffer, uint32_t buffer_size);
void get_adc_mode(uint16_t *current_flag);

#endif /* INC_ADC_SET_MODE_H_ */

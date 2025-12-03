/*
 * adc_set_mode.h
 *
 *  Created on: Nov 20, 2025
 *      Author: RASBI
 */

#include <stdint.h>

#ifndef INC_ADC_SET_MODE_H_
#define INC_ADC_SET_MODE_H_

typedef enum {
	ADC_MODE_STOP,
	ADC_MODE_CH1,
	ADC_MODE_CH2,
	ADC_MODE_CH1_AND_CH2
}ADC_Mode_t;

void SET_ADC_MODE(ADC_Mode_t mode, uint16_t* buffer, uint32_t buffer_size);


#endif /* INC_ADC_SET_MODE_H_ */

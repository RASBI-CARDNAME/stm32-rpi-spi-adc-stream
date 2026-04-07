/*
 * adc_set_mode.c
 *
 *  Created on: Nov 20, 2025
 *      Author: RASBI
 *
 *  Optimized on : 2026 04/04 SAT 16:33 KST
 */


#include "adc_set_mode.h"

//Change ADC`s Channel if you want :)
#define CHANNEL_1 ADC_CHANNEL_0
#define CHANNEL_2 ADC_CHANNEL_1

void set_adc_mode(ADC_Mode_t mode, uint16_t* buffer, uint32_t buffer_size) {
	/* STOP ADC */
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_ADC_Stop(&hadc2);

	/* SET Parameter */
	ADC_MultiModeTypeDef multimode = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	switch(mode) {
	case ADC_MODE_STOP: //stop ~ we already stopped.
		return;

	case ADC_MODE_CH1:
		hadc1.Init.NbrOfConversion = 1;
		hadc2.Init.NbrOfConversion = 1;

		HAL_ADC_Init(&hadc1);
		HAL_ADC_Init(&hadc2);

		sConfig.Channel = CHANNEL_1;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;

		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		HAL_ADC_ConfigChannel(&hadc2, &sConfig);

		break;

	case ADC_MODE_CH2:
		hadc1.Init.NbrOfConversion = 1;
		hadc2.Init.NbrOfConversion = 1;

		HAL_ADC_Init(&hadc1);
		HAL_ADC_Init(&hadc2);

		sConfig.Channel = CHANNEL_2;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;

		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		HAL_ADC_ConfigChannel(&hadc2, &sConfig);

		break;

	case ADC_MODE_CH1_AND_CH2:
		hadc1.Init.NbrOfConversion = 2;
		hadc2.Init.NbrOfConversion = 2;

		HAL_ADC_Init(&hadc1);
		HAL_ADC_Init(&hadc2);

		//CH1 SET
		sConfig.Channel = CHANNEL_1;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;

		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		HAL_ADC_ConfigChannel(&hadc2, &sConfig);

		//CH2 SET
		sConfig.Channel = CHANNEL_2;
		sConfig.Rank = ADC_REGULAR_RANK_2;

		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		HAL_ADC_ConfigChannel(&hadc2, &sConfig);

		break;

	default:
		//error handle
		return;
	}
	/* Common Code */
	multimode.Mode = ADC_DUALMODE_INTERLFAST;
	HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);

	HAL_ADC_Start(&hadc2);
	HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)buffer, buffer_size);
}

/* receive control signal */
void get_adc_mode(volatile uint16_t *current_flag, volatile uint16_t *previous_flag) {
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_ADC_Stop(&hadc2);
	HAL_SPI_Abort(&hspi1); //Stop SPI

	//Clear DR
	while(__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE)) {
		volatile uint32_t temp = hspi1.Instance -> DR;
		(void)temp; // Read and Clear
	}

	__HAL_SPI_CLEAR_OVRFLAG(&hspi1); //Clear over run flag

	uint16_t rx_cmd = ADC_MODE_STOP;

	if (HAL_SPI_Receive(&hspi1, (uint8_t *)&rx_cmd, 1, 2000) == HAL_OK) {
		*current_flag = rx_cmd; // success
	} else {
		*current_flag = ADC_MODE_STOP;      // if failed, stop.
	}

	*previous_flag = REFRESH_FLAG;
}

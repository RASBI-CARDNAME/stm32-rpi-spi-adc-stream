/*
 * adc_set_mode.c
 *
 *  Created on: Nov 20, 2025
 *      Author: RASBI
 */


#include "adc_set_mode.h"
#include "stm32f1xx_hal.h"
#include <string.h>

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc1;

//Change ADC`s Channel if you want :)
#define CHANNEL_1 ADC_CHANNEL_0
#define CHANNEL_2 ADC_CHANNEL_1

/*
	ADC_MODE_STOP 			= 0
	ADC_MODE_CH1 				= 1
	ADC_MODE_CH2 				= 2
	ADC_MODE_CH1_AND_CH2 	= 3
*/

void SET_ADC_MODE(ADC_Mode_t mode, uint16_t* buffer, uint32_t buffer_size) {

	ADC_MultiModeTypeDef multimode = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	HAL_ADC_Stop_DMA(&hadc1);

	switch(mode) {
	case 0: //stop ~ we already stopped.
		break;

	case 1: //ch1 only
		hadc1.Init.NbrOfConversion = 1;
		hadc2.Init.NbrOfConversion = 1;

		HAL_ADC_Init(&hadc1);
		HAL_ADC_Init(&hadc2);

		sConfig.Channel = ADC_CHANNEL_0;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;

		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		HAL_ADC_ConfigChannel(&hadc2, &sConfig);

		HAL_ADC_Start(&hadc2);
		HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)buffer, buffer_size);



		break;

	case 2: //ch2 only
		hadc1.Init.NbrOfConversion = 1;
		hadc2.Init.NbrOfConversion = 1;

		HAL_ADC_Init(&hadc1);
		HAL_ADC_Init(&hadc2);

		sConfig.Channel = ADC_CHANNEL_1;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;

		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		HAL_ADC_ConfigChannel(&hadc2, &sConfig);

		HAL_ADC_Start(&hadc2);
		HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)buffer, buffer_size);

		break;

	case 3: //ch1+ch2 dual mode
		hadc1.Init.NbrOfConversion = 2;
		hadc2.Init.NbrOfConversion = 2;

		HAL_ADC_Init(&hadc1);
		HAL_ADC_Init(&hadc2);

		multimode.Mode = ADC_DUALMODE_INTERLFAST;
		HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);

		//ch1
		sConfig.Channel = ADC_CHANNEL_0;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;

		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		HAL_ADC_ConfigChannel(&hadc2, &sConfig);

		//ch2
		sConfig.Channel = ADC_CHANNEL_1;
		sConfig.Rank = ADC_REGULAR_RANK_2;

		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		HAL_ADC_ConfigChannel(&hadc2, &sConfig);

		HAL_ADC_Start(&hadc2);
		HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)buffer, buffer_size);
		break;

	default:
		//error handle
		break;
	}
}

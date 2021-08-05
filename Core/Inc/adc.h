/*
 * adc.h
 *
 *  Created on: Aug 4, 2021
 *      Author: yusaku
 */

#pragma once

#include "main.h"

#if __cplusplus
extern "C" {
#endif

void ConfigureADC1(void);
//uint16_t ADC_Self_Cal_Channel(uint32_t channel);
void ADC_SelfCal(void);
void ConfigureADCNormal(void);

#if __cplusplus
}
#endif

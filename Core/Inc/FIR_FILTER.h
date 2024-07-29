/*
 * FIR_FILTER.h
 *
 *  Created on: May 1, 2024
 *      Author: oguzk
 */

#ifndef INC_FIR_FILTER_H_
#define INC_FIR_FILTER_H_


#include <stdint.h>
#include "stm32f4xx.h"

#define FIR_FILTER_LENGHT 16
#define MAV_FILTER_LENGHT 4

typedef struct
{
	float buf[FIR_FILTER_LENGHT];
	uint8_t bufIndex;

	float out;

}FIRFilter;

void FIRFilter_Init(FIRFilter *fir);
float FIRFilter_Update(FIRFilter *fir , float inp);
void MAFilter_Init(FIRFilter *fir);

#endif /* INC_FIR_FILTER_H_ */

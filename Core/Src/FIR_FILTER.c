/*
 * FIR_FILTER.c
 *
 *  Created on: May 1, 2024
 *      Author: oguzk
 */


#include "FIR_FILTER.h"


static float FIR_IMPULSE_RESPONSE [FIR_FILTER_LENGHT] = {0.00015603561669922996,
		-0.00915184625787686,
		-0.005331690430567518,
		0.03391049310666665,
		-0.003580175073408329,
		-0.09939971585617939,
		0.08849695267706374,
		0.4914139445563702,
		0.4914139445563702,
		0.08849695267706374,
		-0.09939971585617939,
		-0.003580175073408329,
		0.03391049310666665,
		-0.005331690430567518,
		-0.00915184625787686,
		0.00015603561669922996

};


static float MAF_IMPULSE_RESPONSE [4] = {0.25f , 0.25f , 0.25f , 0.25f};



void FIRFilter_Init(FIRFilter *fir)
{
	for (uint8_t n = 0; n< FIR_FILTER_LENGHT; n++)
	{
		fir->buf[n] =0.0f; // filtre buffer temizleme

	}

	fir->bufIndex =0; // index reset
	fir->out = 0; // clear output
}




float FIRFilter_Update(FIRFilter *fir , float inp)
{
	// son gelen veriyi buffer'a kaydet
	fir->buf[fir->bufIndex] =inp;

	// buffer indexini arttır
	fir->bufIndex++;

	// index uzunluğu taşması kontrolü

	if(fir->bufIndex == FIR_FILTER_LENGHT) fir->bufIndex=0;

	fir->out =0;

	uint8_t sumIndex = fir->bufIndex;

	for (uint8_t n = 0; n< FIR_FILTER_LENGHT; n++)
	{
		// index kontrol
		if(sumIndex >0) sumIndex --;
		else sumIndex = FIR_FILTER_LENGHT-1;

		// convulution toplam kısmı
		fir->out += FIR_IMPULSE_RESPONSE[n] * fir->buf[sumIndex];

	}

	return fir->out;

}



void MAFilter_Init(FIRFilter *fir)
{
	for (uint8_t n = 0; n< MAV_FILTER_LENGHT; n++)
	{
		fir->buf[n] =0.0f; // filtre buffer temizleme

	}

	fir->bufIndex =0; // index reset
	fir->out = 0; // clear output
}



float MAVFilter_Update(FIRFilter *fir , float inp)
{
	// son gelen veriyi buffer'a kaydet
	fir->buf[fir->bufIndex] =inp;

	// buffer indexini arttır
	fir->bufIndex++;

	// index uzunluğu taşması kontrolü

	if(fir->bufIndex == MAV_FILTER_LENGHT) fir->bufIndex=0;

	fir->out =0;

	uint8_t sumIndex = fir->bufIndex;

	for (uint8_t n = 0; n< MAV_FILTER_LENGHT; n++)
	{
		// index kontrol
		if(sumIndex >0) sumIndex --;
		else sumIndex = MAV_FILTER_LENGHT-1;

		// convulution toplam kısmı
		fir->out += MAF_IMPULSE_RESPONSE[n] * fir->buf[sumIndex];

	}

	return fir->out;

}

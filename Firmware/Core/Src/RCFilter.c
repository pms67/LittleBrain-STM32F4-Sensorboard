#include "RCFilter.h"

void RCFilter_Init(RCFilter *filt, float cutoffFreqHz, float sampleTimeS) {

	/* Compute equivalent 'RC' constant from cut-off frequency */
	float RC = 1.0f / (6.28318530718f * cutoffFreqHz);

	/* Pre-compute filter coefficients for first-order low-pass filter */
	filt->coeff[0] = sampleTimeS / (sampleTimeS + RC);
	filt->coeff[1] = RC / (sampleTimeS + RC);

	/* Clear output buffer */
	filt->out[0] = 0.0f;
	filt->out[1] = 0.0f;

}

float RCFilter_Update(RCFilter *filt, float inp) {

	/* Shift output samples */
	filt->out[1] = filt->out[0];

	/* Compute new output sample */
	filt->out[0] = filt->coeff[0] * inp + filt->coeff[1] * filt->out[1];

	/* Return filtered sample */
	return (filt->out[0]);

}

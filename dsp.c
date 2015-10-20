#include <stdint.h>
#include <math.h>
#include <stdlib.h>
/**
 * Fixed point numbers
 * Num = Value * 2^-SCALE
 */
#define SCALE 4
#define M_PI 3.14159
#define NUM_DATA 128


/* FIXED_COS_TABLE[(k*n)%data_size]
 * ex. FIXED_COS_TABLE[1] = cos(2pi * 1/data_size) = cos(2pi * (1 + data_size)/data_size) * 2^SCALE
 */
int32_t FIXED_COS_TABLE[1] = {16, 15, 15, 15, 15, 15, 15, 15,
		14, 14, 14, 13, 13, 12, 12, 11,
		11, 10, 10, 9, 8, 8, 7, 6,
		6, 5, 4, 3, 3, 2, 1, 0,
		0, 0, -1, -2, -3, -3, -4, -5,
		-6, -6, -7, -8, -8, -9, -10, -10,
		-11, -11, -12, -12, -13, -13, -14, -14,
		-14, -15, -15, -15, -15, -15, -15, -15,
		-16, -15, -15, -15, -15, -15, -15, -15,
		-14, -14, -14, -13, -13, -12, -12, -11,
		-11, -10, -10, -9, -8, -8, -7, -6,
		-6, -5, -4, -3, -3, -2, -1, 0,
		0, 0, 1, 2, 3, 3, 4, 5,
		6, 6, 7, 8, 8, 9, 10, 10,
		11, 11, 12, 12, 13, 13, 14, 14,
		14, 15, 15, 15, 15, 15, 15, 15};
int32_t FIXED_SIN_TABLE[1] = {0, 0, 1, 2, 3, 3, 4, 5,
		6, 6, 7, 8, 8, 9, 10, 10,
		11, 11, 12, 12, 13, 13, 14, 14,
		14, 15, 15, 15, 15, 15, 15, 15,
		16, 15, 15, 15, 15, 15, 15, 15,
		14, 14, 14, 13, 13, 12, 12, 11,
		11, 10, 10, 9, 8, 8, 7, 6,
		6, 5, 4, 3, 3, 2, 1, 0,
		0, 0, -1, -2, -3, -3, -4, -5,
		-6, -6, -7, -8, -8, -9, -10, -10,
		-11, -11, -12, -12, -13, -13, -14, -14,
		-14, -15, -15, -15, -15, -15, -15, -15,
		-16, -15, -15, -15, -15, -15, -15, -15,
		-14, -14, -14, -13, -13, -12, -12, -11,
		-11, -10, -10, -9, -8, -8, -7, -6,
		-6, -5, -4, -3, -3, -2, -1, 0};

void dft_fixed(uint16_t* data, uint32_t data_size, uint32_t* realresults, uint32_t* imagresults, uint32_t* powerresults) {
	uint32_t i = 0;

	//adjust values to fixed point notation
	int32_t fixed_data[128];
	for(i=0;i<data_size;i++) {
		fixed_data[i] = data[i]<<SCALE;
	}

	//calculate mean of data to renormalize
	float sum = 0;
	for(i=0;i<data_size;i++) {
		sum += fixed_data[i];
//		sum += data[i];
	}
	float mean = (float)sum/(float)data_size;

	//calculate DFT coefficients
	uint16_t k;
	for(k=0;k<data_size;k++) {
		int n;
		float real_sum = 0;
		float imag_sum = 0;
		for(n=0;n<data_size;n++) {
			float n_value = (float)fixed_data[i] - mean;
//			float n_value = (int16_t)data[n] - mean;
//			uint32_t cos_value = FIXED_COS_TABLE[(k*n)%data_size];
//			uint32_t sin_value = FIXED_SIN_TABLE[(k*n)%data_size];
			float cos_value = cos(2.0*3.14159*(float)k*(float)n/(float)data_size);
			float sin_value = sin(2.0*3.14159*(float)k*(float)n/(float)data_size);
			real_sum += n_value*cos_value;
			imag_sum += n_value*sin_value;
		}
		//To get floating point value, multiply these numbers by 2^-16. (2^8 scaling for data, 2^8 scaling for sinusoids)
		realresults[k] = real_sum;
		imagresults[k] = imag_sum;
		powerresults[k] = sqrt(real_sum*real_sum + imag_sum*imag_sum);
	}
}

void dft_float(uint16_t* data, uint32_t data_size, float* realresults, float* imagresults, float* powerresults) {
	//calculate DFT coefficients
	int k;
	for(k=0;k<data_size;k++) {
		int n;
		float real_sum = 0;
		float imag_sum = 0;
		for(n=0;n<data_size;n++) {
			float n_value = (float)data[n];
			float cos_value = cos(2.0*3.14159*(float)k*(float)n/(float)data_size);
			float sin_value = sin(2.0*3.14159*(float)k*(float)n/(float)data_size);
			real_sum += n_value*cos_value;
			imag_sum += n_value*sin_value;
		}
		realresults[k] = real_sum;
		imagresults[k] = imag_sum;
		float val = sqrtf(real_sum*real_sum + imag_sum*imag_sum);
		powerresults[k] = val;
	}
}

/* data parameters - data_buff (buffer containing 16bit sampled ADC data and data_size (likely will be equal to NUM_DATA)
 * system paramenters - bin num ... corresponds to f = binnum/N*fs
 * pointers for results - real and imag
 */
void goertzels_float(uint16_t* data_buff, uint32_t data_size,
			   uint32_t bin_num,
			   float* real, float* imag) {

	float w = 2.0*M_PI/128.0*(float)bin_num;
	float cosvalue = cos(w);
	float sinevalue = sin(w);
	float coeff = 2*cosvalue;

	float q0 = 	0;
	float q1 = 0;
	float q2 = 0;

	int i;
	for(i=0;i<NUM_DATA;i++) {
		q0 = coeff*q1 - q2 + data_buff[i];
		q2 = q1;
		q1 = q0;
	}

	*real = (q1 - q2*cosvalue);
	*imag = q2*sinevalue;
}

/* data parameters - data_buff (buffer containing 16bit sampled ADC data and data_size (likely will be equal to NUM_DATA)
 * system paramenters - target_frequency (Hz) and sample_frequency (Hz)
 * pointers for results - real and imag
 */
void goertzels_fixed(uint16_t* data_buff, uint32_t data_size,
			   uint32_t target_frequency, uint32_t sample_frequency,
			   uint32_t* real, uint32_t* imag) {

	uint32_t k = NUM_DATA*(float)target_frequency/(float)sample_frequency;
	uint32_t w = 2.0*M_PI/(float)NUM_DATA*(float)k;
	uint32_t cosvalue = cos(w);
	uint32_t sinevalue = sin(w);
	uint32_t coeff = 2*cosvalue;

	uint32_t q0 = 	0;
	uint32_t q1 = 0;
	uint32_t q2 = 0;

	int i;
	for(i=0;i<NUM_DATA;i++) {
		float value = (float)data_buff[i];
		q0 = coeff*q1 - q2 + value;
		q2 = q1;
		q1 = q0;
	}

	*real = (q1 - q2*cosvalue);
	*imag = q2*sinevalue;
}

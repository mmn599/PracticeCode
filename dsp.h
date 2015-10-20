#ifndef DSP_H_
#define DSP_H_

void dft_fixed(uint16_t* data, uint32_t data_size, uint32_t* realresults, uint32_t* imagresults, uint32_t* powerresults);

void dft_float(uint16_t* data, uint32_t data_size, float* realresults, float* imagresults, float* powerresults);

void goertzels_float(uint16_t* data_buff, uint32_t data_size,
			   uint32_t bin_num,
			   float* real, float* imag);

void goertzels_fixed(uint16_t* data_buff, uint32_t data_size,
			   uint32_t target_frequency, uint32_t sample_frequency,
			   uint32_t* real, uint32_t* imag);

#endif /* DSP_H_ */

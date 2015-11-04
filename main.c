#include "driverlib.h"
#include <math.h>

#define NUM_DATA 128

uint8_t DacTable_270_1_3_5[270] = {
		30, 32, 34, 36, 38, 40, 42, 44,
		45, 47, 48, 49, 50, 51, 51, 52,
		52, 52, 52, 52, 52, 51, 50, 50,
		49, 48, 47, 45, 44, 43, 42, 40,
		39, 38, 37, 36, 35, 34, 33, 32,
		31, 31, 31, 30, 30, 30, 30, 30,
		30, 31, 31, 32, 32, 33, 34, 34,
		35, 36, 36, 37, 38, 38, 39, 39,
		39, 40, 40, 40, 40, 40, 40, 39,
		39, 39, 38, 38, 37, 36, 36, 35,
		34, 34, 33, 32, 32, 31, 31, 30,
		30, 30, 30, 30, 30, 31, 31, 31,
		32, 33, 34, 35, 36, 37, 38, 39,
		40, 42, 43, 44, 45, 47, 48, 49,
		50, 50, 51, 52, 52, 52, 52, 52,
		52, 51, 51, 50, 49, 48, 47, 45,
		44, 42, 40, 38, 36, 34, 32, 30,
		28, 26, 24, 22, 20, 18, 16, 15,
		13, 12, 11, 10, 9, 9, 8, 8,
		8, 8, 8, 8, 9, 10, 10, 11,
		12, 13, 15, 16, 17, 18, 20, 21,
		22, 23, 24, 25, 26, 27, 28, 29,
		29, 29, 30, 30, 30, 30, 30, 30,
		29, 29, 28, 28, 27, 26, 26, 25,
		24, 24, 23, 22, 22, 21, 21, 21,
		20, 20, 20, 20, 20, 20, 21, 21,
		21, 22, 22, 23, 24, 24, 25, 26,
		26, 27, 28, 28, 29, 29, 30, 30,
		30, 30, 30, 30, 29, 29, 29, 28,
		27, 26, 25, 24, 23, 22, 21, 20,
		18, 17, 16, 15, 13, 12, 11, 10,
		10, 9, 8, 8, 8, 8, 8, 8,
		9, 9, 10, 11, 12, 13, 15, 16,
		18, 20, 22, 24, 26, 28};

void ADC_Init() {
	ADC14_enableModule();

    /* Configuring GPIOs (5.5 A0) */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN5,
    GPIO_TERTIARY_MODULE_FUNCTION);

	ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, ADC_NOROUTE);

	ADC14_setResolution(ADC_14BIT);
	ADC14_setSampleHoldTrigger(ADC_TRIGGER_ADCSC, false);
	ADC14_setSampleHoldTime(ADC_PULSE_WIDTH_16, ADC_PULSE_WIDTH_16);

	ADC14_configureSingleSampleMode(ADC_MEM0, false);

	ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A0, false);

	ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);
	ADC14_setPowerMode(ADC_UNRESTRICTED_POWER_MODE);
	ADC14_enableConversion();
	/*
	 * 0b = ADC reference buffer on continuously
	 * BIT 2 (ADC14REFBURST) in ADC14CTL1 register
	 */
	ADC14CTL1 &= ~(BIT2);
}

void CS_Init() {
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ,
            GPIO_PIN3 | GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);
    CS_setExternalClockSourceFrequency(32000,48000000);
    PCM_setPowerState(PCM_AM_LDO_VCORE1);
    PCM_setCoreVoltageLevel(PCM_VCORE1);
    FlashCtl_setWaitState(FLASH_BANK0, 2);
    FlashCtl_setWaitState(FLASH_BANK1, 2);
    CS_startHFXT(false);
    CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_2);
}

const eUSCI_UART_Config uartConfig = {
            EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
			156,                                      // BRDIV = 78
            4,                                       // UCxBRF = 2
            0,                                       // UCxBRS = 0
            EUSCI_A_UART_NO_PARITY,                  // No Parity
            EUSCI_A_UART_LSB_FIRST,                  // MSB First
            EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
            EUSCI_A_UART_MODE,                       // UART mode
			EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};

void UART_Init() {
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    UART_initModule(EUSCI_A0_MODULE, &uartConfig);
    UART_enableModule(EUSCI_A0_MODULE);
}

void GPIO_Init() {
	P4DIR = 0xFF; //make pins 0-5 out
    P4OUT = 0;
}

typedef void ( *sample_function_ptr_t ) (uint16_t*, uint8_t*, uint32_t, uint32_t);
extern sample_function_ptr_t copy_to_ram();
extern void output_and_sample(uint16_t*, uint8_t*, sample_function_ptr_t, uint32_t);
extern void sampleLoop(uint16_t*, uint8_t*, sample_function_ptr_t, uint32_t);

void sendFloatDFTUART(uint16_t* data_buff, float* power_k, uint32_t data_size) {
	int dataIdx = 0;
	for(dataIdx = 0; dataIdx < data_size; dataIdx++) {
		uint32_t value = data_buff[dataIdx];
		int i;
		UART_transmitData(EUSCI_A0_MODULE, 'a');
		for(i=0;i<4;i++) {
	    	uint8_t data_byte = value>>(i*8);
	    	UART_transmitData(EUSCI_A0_MODULE, data_byte);
		}
	}
	for(dataIdx = 0; dataIdx < data_size; dataIdx++) {
		uint32_t value = (uint32_t) power_k[dataIdx];
		int i;
		UART_transmitData(EUSCI_A0_MODULE, 'a');
		for(i=0;i<4;i++) {
		    uint8_t data_byte = value>>(i*8);
		    UART_transmitData(EUSCI_A0_MODULE, data_byte);
		}
	}
}

void dft_float(uint16_t* data, uint32_t data_size, float* realresults, float* imagresults, float* powerresults) {
	int i = 0;
	//calculate mean of data to renormalize
	float sum = 0;
	for(i=0;i<data_size;i++) {
		sum = sum + (float)data[i];
	}
	float mean = sum/(float)data_size;

	//calculate DFT coefficients
	int k;
	for(k=0;k<data_size;k++) {
		int n;
		float real_sum = 0;
		float imag_sum = 0;
		for(n=0;n<data_size;n++) {
			float n_value = (float)data[n] - mean;
			float cos_value = cos(2.0*3.14159*(float)k*(float)n/(float)data_size);
			float sin_value = sin(2.0*3.14159*(float)k*(float)n/(float)data_size);
			real_sum += n_value*cos_value;
			imag_sum += (-1)*n_value*sin_value;
		}
		realresults[k] = real_sum;
		imagresults[k] = imag_sum;
		float val = sqrtf(real_sum*real_sum + imag_sum*imag_sum);
		powerresults[k] = val;
	}
}

//TODO: store const array of w values to improve performance
void goertzels_float(uint16_t* data_buff, uint32_t data_size, uint32_t* bins, uint8_t bin_count, float* real, float* imag) {
	int b;
	for(b=0;b<bin_count;b++) {
		float w = 2.0*3.141592*(float)bins[b]/(float)NUM_DATA;
		float cosvalue = cos(w);
		float sinvalue = sin(w);
		float coeff = 2*cosvalue;

		float q0 = 	0;
		float q1 = 0;
		float q2 = 0;

		int i;
		for(i=0;i<data_size;i++) {
			float value = (float)data_buff[i];
			q0 = coeff*q1 - q2 + value;
			q2 = q1;
			q1 = q0;
		}

		real[b] = (q1*cosvalue - q2);
		imag[b] = q1*sinvalue;
	}
}

/* values calculated for Xpt sample at Fs
 * index k repr esents value corresponding to bin number
 * corresponds to frequency = k*(Fs/X)
 * k=0 ... bin=1
 */
const int32_t COS_NUM[5] = {};
const int32_t COS_DEN[5] = {};
const int32_t SIN_NUM[5] = {};
const int32_t SIN_DEN[5] = {};
const int32_t COEFF_NUM[5] = {};
const int32_t COEFF_DEN[5] = {};

void goertzels_fixed(uint16_t* data, uint32_t data_size, uint32_t bins, uint8_t bin_count, int32_t* real, int32_t* imag) {
    int b;
    for(b=0;b<bin_count;b++) {
        int32_t z0, z1, z2;
        uint32_t n;

        int32_t cos_num = COS_NUM[b];
        int32_t cos_den = COS_DEN[b];
        int32_t sin_num = SIN_NUM[b];
        int32_t sin_den = SIN_DEN[b];
        int32_t coeff_num = COEFF_NUM[b];
        int32_t coeff_den = COEFF_DEN[b];

        z1 = 0;
        z2 = 0;
        for(n=0; n<data_size; n++) {
        	z0 = (coeff_num*z1)/coeff_den - z2 + value;
        	z2 = z1;
        	z1 = z0;
        }

    	real[b] = (z1*cos_num)/cos_den - z2;
    	imag[b] = (z1*sin_num)/sin_den;
    }

}

int main(void) {
    WDT_A_holdTimer();

    CS_Init();
    ADC14CTL0 = 0;
    ADC_Init();
    GPIO_Init();
    UART_Init();

    FPU_enableModule();

    sample_function_ptr_t sampleFunctionPointer = copy_to_ram();

    float imagresults[NUM_DATA];
	float realresults[NUM_DATA];
	uint16_t data_table[NUM_DATA];

    while(1) {
    	sampleLoop(data_table, DacTable_270_1_3_5, sampleFunctionPointer, NUM_DATA);
      	dft_float(data_table, NUM_DATA, realresults, imagresults, powerresults);
      	sendFloatDFTUART(data_table, powerresults, NUM_DATA);
    }
}

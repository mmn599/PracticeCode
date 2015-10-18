#include "driverlib.h"
#include <math.h>

#define NUM_DATA 128

uint8_t DacTable_64[64] = {0x19,0x1b,0x1e,0x20,0x23,0x25,0x27,0x29,
		0x2b,0x2c,0x2e,0x2f,0x30,0x31,0x32,0x32,
		0x32,0x32,0x32,0x31,0x30,0x2f,0x2e,0x2c,
		0x2b,0x29,0x27,0x25,0x23,0x20,0x1e,0x1b,
		0x19,0x17,0x14,0x12,0xf,0xd,0xb,0x9,
		0x7,0x6,0x4,0x3,0x2,0x1,0x0,0x0,
		0x0,0x0,0x0,0x1,0x2,0x3,0x4,0x6,
		0x7,0x9,0xb,0xd,0xf,0x12,0x14,0x17};

uint8_t DacTable_32[32] = {0x19,0x1e,0x23,0x27,0x2b,0x2e,0x30,0x32,
0x32,0x32,0x30,0x2e,0x2b,0x27,0x23,0x1e,
0x19,0x14,0xf,0xb,0x7,0x4,0x2,0x0,
0x0,0x0,0x2,0x4,0x7,0xb,0xf,0x14};

uint8_t DacTable_128_1[128] = {30, 31, 33, 34, 36, 37, 39, 40,
		41, 43, 44, 45, 47, 48, 49, 50,
		51, 52, 53, 54, 55, 56, 56, 57,
		58, 58, 59, 59, 59, 60, 60, 60,
		60, 60, 60, 60, 59, 59, 59, 58,
		58, 57, 56, 56, 55, 54, 53, 52,
		51, 50, 49, 48, 47, 45, 44, 43,
		41, 40, 39, 37, 36, 34, 33, 31,
		30, 29, 27, 26, 24, 23, 21, 20,
		19, 17, 16, 15, 13, 12, 11, 10,
		9, 8, 7, 6, 5, 4, 4, 3,
		2, 2, 1, 1, 1, 0, 0, 0,
		0, 0, 0, 0, 1, 1, 1, 2,
		2, 3, 4, 4, 5, 6, 7, 8,
		9, 10, 11, 12, 13, 15, 16, 17,
		19, 20, 21, 23, 24, 26, 27, 29};

uint8_t DacTable_128_1_3[128] = {30, 33, 36, 39, 41, 44, 46, 48,
		50, 51, 52, 53, 53, 53, 53, 52,
		51, 50, 49, 47, 45, 44, 42, 40,
		38, 36, 35, 33, 32, 31, 31, 30,
		30, 30, 31, 31, 32, 33, 35, 36,
		38, 40, 42, 44, 45, 47, 49, 50,
		51, 52, 53, 53, 53, 53, 52, 51,
		50, 48, 46, 44, 41, 39, 36, 33,
		30, 27, 24, 21, 19, 16, 14, 12,
		10, 9, 8, 7, 7, 7, 7, 8,
		9, 10, 11, 13, 15, 16, 18, 20,
		22, 24, 25, 27, 28, 29, 29, 30,
		30, 30, 29, 29, 28, 27, 25, 24,
		22, 20, 18, 16, 15, 13, 11, 10,
		9, 8, 7, 7, 7, 7, 8, 9,
		10, 12, 14, 16, 19, 21, 24, 27};

uint8_t DacTable_128_1_3_5_7[128] = {30, 36, 41, 46, 49, 51, 52, 51,
		50, 47, 44, 40, 37, 34, 32, 30,
		30, 30, 31, 33, 35, 36, 37, 38,
		38, 38, 37, 35, 34, 32, 31, 30,
		30, 30, 31, 32, 34, 35, 37, 38,
		38, 38, 37, 36, 35, 33, 31, 30,
		30, 30, 32, 34, 37, 40, 44, 47,
		50, 51, 52, 51, 49, 46, 41, 36,
		30, 24, 19, 14, 11, 9, 8, 9,
		10, 13, 16, 20, 23, 26, 28, 30,
		30, 30, 29, 27, 25, 24, 23, 22,
		22, 22, 23, 25, 26, 28, 29, 30,
		30, 30, 29, 28, 26, 25, 23, 22,
		22, 22, 23, 24, 25, 27, 29, 30,
		30, 30, 28, 26, 23, 20, 16, 13,
		10, 9, 8, 9, 11, 14, 19, 24};

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

void sendDataUART(uint16_t* data_buff, float* power_k, uint32_t data_size) {
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

void dft(uint16_t* data, uint32_t data_size, float* realresults, float* imagresults, float* powerresults) {
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
			float n_value = (float)(int16_t)data[n] - mean;
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

float goertzels(uint16_t* data_buff, uint32_t data_size) {
	int k = (int)(0.5 + (float)NUM_DATA*28250.0/915000.0);
	float w = 2.0*3.141592/(float)NUM_DATA*(float)k;
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

	float real = (q1 - q2*cosvalue);
	float imag = q2*sinevalue;
	float result = sqrtf(real*real + imag*imag);

	return result;
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
	float powerresults[NUM_DATA];
	uint16_t data_table[NUM_DATA];

    while(1) {
    	sampleLoop(data_table, DacTable_128_1_3_5_7, sampleFunctionPointer, NUM_DATA);
      	dft(data_table, NUM_DATA, realresults, imagresults, powerresults);
      	sendDataUART(data_table, powerresults, NUM_DATA);
    }
}

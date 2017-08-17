// Standard includes
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

// Driverlib includes
#include "utils.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_types.h"
#include "hw_adc.h"
#include "hw_ints.h"
#include "hw_gprcm.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "uart.h"
#include "pinmux.h"
#include "pin.h"
#include "adc.h"

//#include "adc_userinput.h"
#include "uart_if.h"
#include "gpio.h"

#define NO_OF_SAMPLES 		128
extern unsigned long pulAdcSamples[4096];

void adc (void){

	unsigned long ulSample;
	unsigned int  uiChannel;
	unsigned int  uiIndex=0;

	while(uiIndex < NO_OF_SAMPLES + 4)
	{
		if(MAP_ADCFIFOLvlGet(ADC_BASE, uiChannel))
	    {
			ulSample = MAP_ADCFIFORead(ADC_BASE, uiChannel);

	        pulAdcSamples[uiIndex++] = ulSample;			// Fill data sample buffer
	    }


	        }

}


void adcPrint (void){
	unsigned int  uiIndex=0;
	//
	// Print out ADC samples
	//
	//float sum = 0;

	while(uiIndex < NO_OF_SAMPLES)
	{
		//UART_PRINT("\n\rVoltage is %f\n\r",(((float)((pulAdcSamples[4+uiIndex] >> 2 ) & 0x0FFF))*1.4*2.279)/4096);
	    //uiIndex++;
	    //sum += (((float)((pulAdcSamples[4+uiIndex] >> 2 ) & 0x0FFF))*1.4*2.279)/4096;
	}


	//UART_PRINT("\n\rVoltage is %f\n\r",((pulAdcSamples[4] >> 2 ) & 0x0FFF)*1.4/4096);
	                //UART_PRINT("\n\r");
	                //UART_PRINT("\n\rAverage Voltage Level %f\n\r", (float)(sum/NO_OF_SAMPLES));
}






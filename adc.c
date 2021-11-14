/********************************************************************
 ************ COPYRIGHT (c) 2014 by ke0ff, Taylor, TX   *************
 *
 *  File name: adc.c
 *
 *  Module:    Control
 *
 *  Summary:
 *  Tiva adc support functions
 *
 *******************************************************************/

#include <stdint.h>
#include <ctype.h>
#include "inc/tm4c123gh6pm.h"
#include "typedef.h"
#include "init.h"						// App-specific SFR Definitions
#include "adc.h"

//=============================================================================
// local registers


//=============================================================================
// local Fn declarations


//*****************************************************************************
// adc_init()
//  initializes the processor ADC peripheral
//	returns bitmapped initialize result status as U16
//
//*****************************************************************************
U16 adc_init(void)
{
	volatile uint32_t	ui32Loop;

    // ADC init
	SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;			// enable ADC clock
	ui32Loop = SYSCTL_RCGCADC_R;
	GPIO_PORTE_AFSEL_R |= (AIN9|AIN2);
	GPIO_PORTE_PCTL_R &= 0xFFF0FF0F;

	GPIO_PORTE_AMSEL_R |= (AIN9|AIN2);
	ADC0_CC_R = ADC_CC_CS_PIOSC;					// use PIOSC
	ADC0_PC_R = 0x0001;								// 125KHz samp rate
	// ADC sequencer init
	ADC0_SSPRI_R = 0x0123;							// Sequencer 2 is highest priority
	ADC0_ACTSS_R &= ~0x0004;						// disable sample sequencer 2
	ADC0_EMUX_R &= ~0x0F00;							// seq2 is software trigger
//	ADC0_SSMUX3_R &= ~0x000F;						// clear SS2 field low nyb
	ADC0_SSMUX2_R = 0x0292;							// set channels, ain2, ain9, ain2
	ADC0_SSCTL2_R = 0xE000;							// TS0 is 4th sample, IE0 enabled after 4th samp
	ADC0_IM_R &= ~0x0004;							// disable SS2 interrupts
	ADC0_ACTSS_R |= 0x0004;							// enable sample sequencer 2
	return 0;
}

//*****************************************************************************
// adc_in()
//  starts SS2 and waits for it to finish.  Returns ADC results as U16 placed
//	at pointer.  In the array, even offsets are status, odd offsets are data.
//	offset 1 = IM, offset 3 = AIN9, offset 5 = AIN10, offset 7 = TJ
//	float voltage = rawADC * Vref / maxADC ; Vref = 3.3V, maxADC = 0x1000
//	Vtj = 2.7 - ((TJ + 55) / 75)
//	Vtj = rawADC * 3.3 / 4096
//	TJ = 147.5 - (75 * ((rawADC * 3.3 / 4096)))
//
//*****************************************************************************
U8 adc_in(U16* p)
{
	U8	i;

	ADC0_PSSI_R = 0x0004;							// initiate SS2
	while((ADC0_RIS_R & 0x04) == 0);				// wait for conversion done
	for(i=0; i<4; i++){
		*p++ = ADC0_SSFSTAT2_R & 0xffff;			// get fifo status
		*p++ = ADC0_SSFIFO2_R & 0x0fff;				// read result
	}
	ADC0_ISC_R = 0x04;								// acknowledge completion
	return i;
}

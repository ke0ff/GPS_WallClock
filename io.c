/*********************************************************************
 ************ COPYRIGHT (c) 2018 by ke0ff, Taylor, TX   **************
 *
 *  File name: io.c
 *
 *  Module:    Control
 *
 *  Summary:
 *  This file encapsulates the I/O drivers and high level functions
 *	for the HF Bridge SW.
 *
 *  Project scope revision history:
 *    03-22-15 jmh:  creation date
 *
 *******************************************************************/

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>
#include "inc/tm4c123gh6pm.h"
#include "typedef.h"
#include "init.h"						// App-specific SFR Definitions

#include "cmd_fn.h"						// fn protos, bit defines, rtn codes, etc.
#include "serial.h"
//#include "I2C0.h"
#include "io.h"

//=============================================================================
// local registers

//=============================================================================
// local Fn declarations

//-----------------------------------------------------------------------------
// get_temp() returns temperature reading, chan = 0 is TA, chan = 1 is TH
//	res is:  0 = 0.5, 1 = 0.25, 2 = 0.125, 3 = 0.0625
//-----------------------------------------------------------------------------
float get_temp(U8 chan, U8 res){
	U8	i;					// temp resolution
	U16	k;					// raw data temp

	i = res;
	i <<= 5;
	k = (75 << res) + 75;										// tmeas(max) = 75 * 2^i (ms)
	switch(chan){
	case 0:
//		I2C_Send2(ADDR_TSENS0, PTR_CONFIG, i);
//		I2C_Send1(ADDR_TSENS0, PTR_TSENSE);
		wait(k);
//		k = I2C_Recv2(ADDR_TSENS0);								// get tempA sense data
		break;

	case 1:
//		I2C_Send2(ADDR_TSENS1, PTR_CONFIG, i);
//		I2C_Send1(ADDR_TSENS1, PTR_TSENSE);
		wait(k);
//		k = I2C_Recv2(ADDR_TSENS1);								// get tempH sense data
		break;
	}
	return temp_float(k);
}

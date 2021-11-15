/********************************************************************
 ************ COPYRIGHT (c) 2021 by ke0ff, Taylor, TX   *************
 *
 *  File name: serial.c
 *
 *  Module:    Control
 *
 *  Summary:   This is the serial I/O module for the FF-PGMR11
 *             protocol converter.
 *
 *******************************************************************/


/********************************************************************
 *  File scope declarations revision history:
 *    07-30-14 jmh:  creation date
 *
 *******************************************************************/

#include "inc/tm4c123gh6pm.h"
#include "typedef.h"
#define VERSOURCE
#include "version.h"
#include "stdio.h"
#include "serial.h"
#include "init.h"

//------------------------------------------------------------------------------
// Define Statements
//------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Local Variable Declarations
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Local Fn Declarations
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// dispSWvers() displays SW version
//-----------------------------------------------------------------------------
void dispSWvers(void){
	char buf[30];
	
	puts0("\n\nGPS Slaved Wall Clock Application");
    sprintf(buf,"Vers: %s, Date: %s",version_number,date_code);
    puts0(buf);
   	sprintf(buf,"IPL: %04x\n",getipl());
   	puts0(buf);
   	puts0("'?' for help, '? ?' for detailed help");
}

//-----------------------------------------------------------------------------
// ledSWvers() returns pointer to LED SW version
//-----------------------------------------------------------------------------
unsigned char* ledSWvers(void){

	return led_versnum;
}

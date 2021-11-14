/********************************************************************
 ************ COPYRIGHT (c) 2021 by ke0ff, Taylor, TX   *************
 *
 *  File name: version.h
 *
 *  Module:    Control
 *
 *  Summary:   This header contains the software version number
 *             as a character string.
 *
 *******************************************************************/


/********************************************************************
 *  File scope declarations revision history:
 *    07-30-14 jmh:  0.0 creation date
 *    04-04-15 jmh:  0.1 First field release
 *
 *******************************************************************/

#ifdef VERSOURCE
const S8    version_number[] = {"0.5"};
const S8    date_code[]      = {"14-Nov-2021"};
#endif

//-----------------------------------------------------------------------------
// Public Fn Declarations
//-----------------------------------------------------------------------------
void dispSWvers(void);
void ccmdSWvers(char* sbuf);

#define VERSION_INCLUDED

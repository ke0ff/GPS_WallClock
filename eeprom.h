/********************************************************************
 ************ COPYRIGHT (c) 2015 by ke0ff, Taylor, TX   *************
 *
 *  File name: eeprom.h
 *
 *  Module:    Control
 *
 *  Summary:   defines and global declarations for io.c
 *
 *  Project scope revision history:
 *    03-23-15 jmh:  creation date
 *
 *******************************************************************/

#include "typedef.h"
#include <stdint.h>

#ifndef EEPROM_H
#define EEPROM_H
#endif

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------
#define	EE_FAIL		0xff		// fail return value
#define	EE_OK		0x00		// OK return value
#define	EE_BLOCK	16			// # words/block

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
#define	EE_TIME_ZONE	0x0000		// time zone address in eeprom array ([15:00])
#define	EE_DSTAUTO		0x0000		// DST auto correct mode ([23:16])
#define	EE_DST			0x0000		// DST auto correct mode ([31:24])
#define	EE_MAXBRT		0x0004		// ambient sensor max brt ([31:24])
#define	EE_MINBRT		0x0004		// ambient sensor min brt ([15:00])
#define	EE_SCALP		0x0008		// hourly cal value (S32)

#define	EE_DST_STOR		0x000c		// std month/week/dst_month/week
									// MSB = STD MON..STD WK..DST MON..DST WK

//-----------------------------------------------------------------------------
// Fn prototypes
//-----------------------------------------------------------------------------
U16 eeprom_init(void);
U32 eerd(U16 addr);
U8  eewr(U16 addr, U32 data);

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------

/*************************************************************************
 *********** COPYRIGHT (c) 2018 by Joseph Haas (DBA FF Systems)  *********
 *
 *  File name: init.h
 *
 *  Module:    Control
 *
 *  Summary:   This is the header file for tsip.c
 *
 *******************************************************************/


/********************************************************************
 *  File scope declarations revision history:
 *    10-27-18 jmh:  creation date
 *
 *******************************************************************/

#include "typedef.h"

#ifndef INIT_H
#define INIT_H
#endif

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------
#define	JAN		1								// month of year indecies
#define	FEB		2
#define	MAR		3
#define	APR		4
#define	MAY		5
#define	JUN		6
#define	JUL		7
#define	AUG		8
#define	SEP		9
#define	OCT		10
#define	NOV		11
#define	DEC		12
#define LASTJAN	31								// last day of each month (normal year)
#define	LASTFEB	28
#define	LASTMAR	31
#define	LASTAPR	30
#define	LASTMAY	31
#define	LASTJUN	30
#define	LASTJUL	31
#define	LASTAUG	31
#define	LASTSEP	30
#define	LASTOCT	31
#define	LASTNOV	30
#define	LASTDEC	31
#define	SECPERMIN		60
#define	MINPERHOUR		60
#define	MINPER6HOUR		(MINPERHOUR*6)
#define	MINPER12HOUR	(MINPERHOUR*12)
#define	MINPER18HOUR	(MINPERHOUR*18)
#define	MINPER24HOUR	(MINPERHOUR*24)

// Error SM defines
#define	MAJORERR	0
#define NOFIX		1
#define	NOSURV		2
#define NOERR		3
#define	STALECOUNT	2							// num stale cycles to trigger MAJORERR

// GPS Message defines
// 0x8F:0xAC -> GPS error flags: minoralm0	(bitmapped)
#define	ANT_OPEN		0x02
#define	ANT_SHORT		0x04
#define	NOT_TRACKING	0x08
#define	SURVEY_INPROG	0x20
#define	NO_STORED_POS	0x40
#define	LEAPSEC_PEND	0x80
// GPS error flags: minoralm1	(bitmapped)
#define	TEST_MODE		0x01
#define	POS_CONF_LOW	0x02
#define	NO_ALMANAC		0x08
#define	PPS_GENERATED	0x10
// GPS error flags: gpstat		(value)
#define	DOING_FIXES		0x00
#define	NO_GPS_TIME		0x01
#define	PDOP_TOOHI		0x03
#define	NO_SATS			0x08
#define	ONE_SAT			0x09
#define	TWO_SAT			0x0a
#define	THREE_SAT		0x0b
#define	SAT_UNUSABL		0x0c
#define	FIX_REJECTED	0x10

// 0x8F:0xAB -> GPS time:
// timing flag bitmasks
#define	UTCTIME		1
#define	UTCPPS		2
#define	TIMESET		4
#define	UTCINFO		8
#define	USRTIME		16

//PBSW mode defines
#define PB_POR      0xFF
#define PB_NORM     0
#define UCPC        1

// loop filter defines
#define LINIT       0
#define LCAP        1
#define LRUN        2

// TSIP state machine defines
#define TSTART      0
#define TRUN        1
#define TPROCESS    2
#define DLE         0x10
#define ETX         0x03
#define TSIPBUFLEN  200

// gstat index identifiers
#define	survprog	0				// GPS status: survey progress (%)
#define	minoralm0	1				// GPS status: alarm flags lsby
#define	minoralm1	2				// GPS status: alarm flags msby
#define	gpstat		3				// GPS status: status reg

//-----------------------------------------------------------------------------
// Prototypes
//-----------------------------------------------------------------------------
U8 get_lastday(U8 mon, U16 year);
U8* get_gstat_ptr(void);
U8 tsip_main(U8 iplfl);
U8* get_gtime_ptr(void);
S16 get_utc_offs(void);
U16 get_gyear(void);
U8 get_gotime(U8 clear);
void enable_gps(U8 flag);
void disp_gpstat(char* obuf);
void get_gtemp(char* cptr);
void do_nextday(void);
void set_gpsdate(S16 tzone);
U8 get_smon(void);
U8 get_sday(void);
U16 get_syear(void);
U8 get_ghour(void);
void set_sdate(U8 mon, U8 day, U16 yr);
void set_xdate(S16 tzone, U8 mon, U8 day, U16 yrs, S32 xrtc_time);
U8 get_gpstat(void);
U8 get_tsip_valid(void);
U16 get_wknum(void);
U32 get_tow(void);
void arch_gps(U8 cmd);
U8* get_gtime_arch_ptr(void);
U32 get_secofday(U8* ptr);
U16 get_dayofyr(U8* ptr, U16 yrval);
U32 get_gpset(void);

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------

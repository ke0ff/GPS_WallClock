/********************************************************************
 ******* Changes COPYRIGHT (c) 2017 by KE0FF, Taylor, TX   **********
 *
 *  File name: rtc.h
 *
 *  Module:    Control
 *
 *  Summary:
 *  This is the header file for the real time clock driver.
 *
 *  Project scope declarations revision history:
 *    08-26-15 jmh:  creation date
 *
 *******************************************************************/

/********************************************************************
 *  File scope declarations revision history:
 *    08-26-15 jmh:  creation date
 *
 *******************************************************************/

#ifndef RTC_H_
#define RTC_H_
#include <stdint.h>
#include "typedef.h"

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------

enum dst_enum{ dst_mon, dst_wk, std_mon, std_wk };

#define	RTC_RAM_LEN	64
#define	SEC			0						// BCD seconds (ss)
#define	RTC_OSC_OFF	0x80					// control bit in RTC SEC register to turn off RTC osc
#define	MIN			1						// BCD minutes (mm)
#define	HR			2						// BCD hours (hh)
#define	DOW			3						// day of week (1 - 7)
#define	DATE		4						// BCD date (day of month) (DD)
#define	MONTH		5						// BCD month (MM)
#define	xxYY		6						// BCD 10's and 1's year (xxyy)
#define	CNTL		7						// control reg. [80] = OUT, [10] = SQWE, [03] = [RS1:RS0]
#define	DST			8						// user RAM, assigned as DST, 0 = standard time, 1 = DST (hh = +1)
#define	YYxx		9						// user RAM, assigned as century reg (yyxx)
#define	DELTA_GMT	10						// user RAM, assigned as DELTA_GMT, the # hours from GMT for the current time zone
#define	DST_MON		11						// user RAM, assigned as month (bcd) of dst start
#define	DST_SUN		12						// user RAM, assigned as nth sunday of dst start
#define	STD_MON		13						// user RAM, assigned as month (bcd) of dst end
#define	STD_SUN		14						// user RAM, assigned as nth sunday of dst end
#define	CHKS_H		15						// user RAM, assigned as checksumH of bytes 8 - 14
#define	CHKS_L		16						// user RAM, assigned as checksumH of bytes 8 - 14
#define	RAM_start	17						// user RAM array (TBD)
#define	RAM_end		63

// PCF2129 Register Map
//COMMAND REG format
#define	XR_RW		0x80					// read/writeN
#define	XR_SA		0x20					// fixed subaddress
#define	XR_RA_MASK	0x1f					// register address mask

// XRTC REGISTERS
#define	XR_CNTL1	0x00					// control 1
#define	EXT_TEST	0x80
#define	STOP		0x20
#define	TSF1		0x10
#define	POR_OVRD	0x08
#define	HOUR12		0x04
#define	MI			0x02
#define	SI			0x01

#define	XR_CNTL2	0x01					// control 2
#define	MSF			0x80
#define	WDTF		0x40
#define	TSF2		0x20
#define	AF			0x10
#define	TSIE		0x04
#define	AIE			0x02

#define	XR_CNTL3	0x02					// control 3
#define	PWRMNG2		0x80
#define	PWRMNG1		0x40
#define	PWRMNG0		0x20
#define	BTSE		0x10
#define	BF			0x08
#define	BLF			0x04
#define	BIE			0x02
#define	BLIE		0x01

// XRTC TIME
#define	XR_SEC		0x03					// time is stored as BCD values
#define	OSF			0x80
#define	XR_SEC_MASK	0x7F

#define	XR_MIN		0x04
#define	XR_MIN_MASK	0x7F

#define	XR_HRS		0x05
#define	XR_HRS_MASK	0x3F

#define	XR_DAY		0x06
#define	XR_DAY_MASK	0x3F

#define	XR_DOW		0x07
#define	XR_DOW_MASK	0x07

#define	XR_MON		0x08
#define	XR_MON_MASK	0x1F

#define	XR_YRS		0x09

// ALARMS
#define	XR_SECALM	0x0a
#define	AE_S		0x80

#define	XR_MINALM	0x0b
#define	AE_M		0x80

#define	XR_HRSALM	0x0c
#define	AE_H		0x80

#define	XR_DAYALM	0x0d
#define	AE_D		0x80

#define	XR_DOWALM	0x0e
#define	AE_W		0x80

#define	XR_CNTLCK	0x0f					// control clk
#define	TCR1		0x80
#define	TCR0		0x40
#define	OTPR		0x20
#define	COF2		0x04
#define	COF1		0x02
#define	COF0		0x01
#define	XR_CLKO_1024 (COF2|COF0)

//WATCHDOG
#define	XR_CNTLWD	0x10					// control watchdog
#define	WD_CD		0x80
#define	TI_TP		0x20
#define	TF1			0x02
#define	TF0			0x01

#define	XR_VALWD	0x11					// watchdog value

//TIMESTAMP
#define	XR_CNTLTS	0x12					// control timestamp
#define	TSM			0x80
#define	TSOFF		0x40
#define	TSTMP4		0x10
#define	TSTMP3		0x08
#define	TSTMP2		0x04
#define	TSTMP1		0x02
#define	TSTMP0		0x01

#define	XR_SECTS	0x13					// timestamp time
#define	XR_MINTS	0x14
#define	XR_HRSTS	0x15
#define	XR_DAYTS	0x16
#define	XR_MONTS	0x17
#define	XR_YRSTS	0x18

//AGING OFFSET
#define	XR_AGOFS	0x19					// aging offset
#define	AO3			0x08
#define	AO2			0x04
#define	AO1			0x02
#define	AO0			0x01

#define	XR_END		0x1a

#define	XWRITE		0						// read/write function semaphores
#define	XREAD		1
//-----------------------------------------------------------------------------
// rtc.c Fn prototypes
//-----------------------------------------------------------------------------

U8* get_dst_info(void);
void set_dst_info(U8 dmon, U8 dwk, U8 smon, U8 swk);
void get_ee_dst(void);
void wr_ee_dst(void);

volatile U8* rtc_ram_io(U8 cmd);
void adjust_rtc_local(U8 dst_i);
void adjust_rtc_gmt(void);
void turn_off_rtc(void);
U8	set_time(U8 hh, U8 mm, U8 ss);
U8	set_date(U8 mon, U8 day, U16 year);
U8	isly(U16 year);
U8	isdst(void);
U16 dsny(U8 day, U8 mon, U16 year);
U32 dse(U8 day, U8 mon, U16 year);
char* dow_str(U8 dow);
char* mon_str(U8 month);
U8	dow_calc(U8 day, U8 mon, U16 year);
U8	bcd_int(U8 bcd);
U16	bcd_int16(U16 bcd);
U8	int_bcd(U8 i);
U16	int_bcd16(U16 i);
U16	chks_ram(volatile U8* rptr);
void rtc_spi(U8* txptr, U8 txlen, U8* rxptr);
U8* get_rtcxptr(U8 scratch_ptr_select);
void rtc_rw(U8* mptr, U8 rw);
void rtc_stop(U8 stop);
void rtc_init(void);

#endif /* RTC_H_ */

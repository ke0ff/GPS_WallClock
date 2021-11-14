/********************************************************************
 ************ COPYRIGHT (c) 2018 by ke0ff, Taylor, TX   *************
 *
 *  File name: init.h
 *
 *  Module:    Control
 *
 *  Summary:   defines and global declarations for main.c
 *
 *******************************************************************/

#include "typedef.h"
#include <stdint.h>

#ifndef INIT_H
#define INIT_H
#endif

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------

//#define EXTXTAL           	// un-comment if external xtal is used

#ifdef EXTXTAL
#define XTAL 1
#else
#define XTAL 0
#endif

#define	ADC_MAX	32767								// ADC max positive value

#define OSC_LF 4            						// osc clock selects
#define OSC_HF 0
#define OSC_EXT 1

#define	MS2000   2000           					// timer constants (ms)
#define SEC10MS    10
#define SEC31MS    31
#define SEC50MS    50
#define MS50  SEC50MS
#define SEC75MS    74
#define SEC100MS  100
#define SEC250MS  250
#define SEC500MS  500
#define SEC500MS_1024  (1024/2)
#define SEC750MS  750
#define SEC1     1000
#define ONESEC   SEC1
#define SEC2     2000
#define SEC3     3000
#define SEC5     5000
#define SEC10   10000
#define SEC15   15000
#define SEC30   30000
#define SEC60   60000
#define SEC300 300000L
#define	ONEMIN	(SEC60)
#define	MSMIN	60000L
#define	MSHOUR	3600000L
#define	MSMIN_1024	(60L * 1024L)
#define	MSHOUR_1024	(3600L * 1024L)
#define	REG_WAIT_DLY 200							// 200ms wait limit for register action
#define	MS_COUNT	1000
#define	MS_COUNT_1024	1024
#define END_OF_DAY	(24L * 60L * 60L * 1000L)		// # ms per day
#define END_OF_DAY_1024	(24L * 60L * 60L * 1024L)	// # 1024Hz cycles per day
#define HOUR_23_1024	(23L * 60L * 60L * 1024L)	// # 1024Hz cycles per day
#define HOUR_01_1024	(01L * 60L * 60L * 1024L)	// # 1024Hz cycles per day
#define	CYC_HOUR	(60L * 60L * 1000L)				// #ms per hour
#define	CYC_HOUR_1024	(60L * 60L * 1024L)			// #1024Hz per hour
#define	SEC_MIN_1024	(1024 * 60)					// 1024 hz / min
#define	SEC_MIN			(1000 * 60)					// 1000 hz / min
#define	SEC_WEEK		(7L * 24L * 60L * 60L)		// 7 dy/wk * 24hr/dy * 60 min/hr * 60 sec/min = 604800 sec/wk
#define	GPS_FRESH	5000							// 5 sec to GPS data stale
#define	GERRTCLIM	(30L * 1024L)					// rtc vs gps max error in ms (30 sec ... sec/min ms/sec)
#define	XERRTCLIM	(3L * 60L * 1024L)				// rtc vs ext rtc max error in ms (3 min ... min sec/min ms/sec)
#define	GPSCALTIME_HR	(168L)						// gps cal timer (1wk)
#define	GPSCALTIME	(GPSCALTIME_HR * 60L * 60L * 1024L) // gps cal timer (ms)	[hr min/hr sec/min ms/sec]
#define	GPSCALLIM	(62L)							// GPS cal limit (rtc cycles per wk.  62 ~~ 100ppb)
#define	CAL_PERIOD	7								// period of calibration (days)
#define	ONEHOUR	(60L * 60L * 1024L)					// ms in 1 hour
#define	TWOHOUR	(2L * 60L * 60L * 1024L)			// ms in 2 hours
#define	MAX_SCALP	(60000L)						// Beyond the max concievable SCALP value
#define	IPL_WAIT	7000 //58000							// 58 sec IPL wait for xRTC update

// timer definitions.  Uses EXTXTAL #def to select between ext crystal and int osc
//  for normal mode.
// SYSCLK value in Hz
#define SYSCLKL 10000L
#define SYSCLK	(80000000L)							// sysclk freq (bus clk)
#define PIOCLK	(16000000L)							// internal osc freq
#define TIMER2_PS 50
#define	TPULSE	(100L)								// in usec
#define TMIN	(((SYSCLK / 100) * TPULSE)/10000L)	// minimum pulse width

// NPXL data output defines
#define	NPXL_FREQ	6300000L
#define	NPXL_DVSR	2L								// must be even, in range of 2 <= dvsr <= 254
#define	NPXL_0		0xc0							// a "0" bit
#define	NPXL_1		0xf0							// a "1" bit

// auto-bright state defines
#define	BRIGHT		1
#define	BRT_INIT	0xff
#define	BRT_LOW		0x00
#define	BRT_HI		BRIGHT
#define	DIMU		20	//80
#define	BRTL		4	//45
#define	MIN_BRT		812 //507
#define	MAX_BRT		6 //(624-1)

// AUX display defines
#define	DISP_MAX	6								// number of aux du modes
#define	LCL_DATE	0								// aux du modes
#define	XUTC_DATE	1
#define	GPS_DATE	2
#define	GPS_STAT	3
#define	CLOCK_VERS	4
#define	AUX_BLANK	5
#define	AUX_TITLE_TIME 2000							// duration of aux titles

// PBSW state defines
#define	PBSW_DBOUNCE	15
#define	PBSW_INIT		0
#define	PBSW_11			1
#define	PBSW_10			2
#define	PBSW_01			3
#define	PBSW_00			4
#define	PBSW_11_DB		11
#define	PBSW_10_DB		12
#define	PBSW_01_DB		13
#define	PBSW_00_DB		14
// pbsw action codes
#define	PBSW_NOSW		0xff
#define	PBSW_11_PRS		11		// both SW pressed
#define	PBSW_10_PRS		12		// SW2 pressed
#define	PBSW_01_PRS		13		// SW1 pressed
#define	PBSW_00_PRS		14		// no button pressed (deprecated)
#define	PBSW_11_REL		1		// both released (deprecated)
#define	PBSW_10_REL		2		// SW2 released from SW1/2 pressed
#define	PBSW_01_REL		3		// SW1 released from SW1/2 pressed
#define	PBSW_00_REL		4		// any button released from one pressed


// Port A defines
// segment address feed HC238 MUX.  Outputs from mux drive NFET-PFET chain to switch LED anode supply on/off for each digit
//	blank shuts off all segments.  used during address switch events to blank until address/segment lines settle.
#define	RXD0			0x01						// UART0
#define	TXD0			0x02						// UART0
#define	DU_SPCK			0x04						// SPI clk (aux DU & RTC)
#define	DU_MISO			0x08						// 7-seg digit address (feeds an HC238 to break out 4 digit enables
#define	RTC_CLK			0x10						// RTC clock (about 3 ppm)
#define	DU_MOSI			0x20						// SPI MOSI (aux DU & RTC)
#define	RTC_INTN		0x40						// rtc /INT signal
#define	RTC_CSN			0x80						// RTC /CS
#define PORTA_DIRV		(RTC_CSN|TXD0|DU_SPCK|DU_MOSI)
#define	PORTA_DENV		(RTC_CSN|TXD0|RXD0|DU_SPCK|DU_MOSI|DU_MISO|RTC_CLK|RTC_INTN)
#define	PORTA_PURV		(RTC_INTN)

// Port B defines
// segment outputs.  each output drives an NFET to switch the LED cathodes to GND
#define PORTB_DIRV		(0xff)						// segment outputs PB0 = "a", PB6 = "g", PB7 = dp/colon
#define	PORTB_DENV		(0xff)						// dp for A2 = upper colon, dp for A3 = lower colon
#define	PORTB_PURV		(0)

// Port C defines
// UART1 I/O
#define	RXD1			0x10						// UART1 (GPS)
#define	TXD2			0x20						// UART1
#define	SEG_ADR0		0x40						// digit select0 to the HC238
#define	SEG_ADR1		0x80						// digit select1 to the HC238
#define PORTC_DIRV		(TXD2|SEG_ADR0|SEG_ADR1)
#define	PORTC_DENV		(TXD2|RXD1|SEG_ADR0|SEG_ADR1)

#define	DIG_H			(0)							// Hh:Mm address defines
#define	DIG_h			(SEG_ADR0)
#define	DIG_M			(SEG_ADR1)
#define	DIG_m			(SEG_ADR1|SEG_ADR0)
#define DADDR_MASK		(~(SEG_ADR1|SEG_ADR0))		// digit address mask

// Port D defines
// not used
#define	DU_CSAB			0x04						// aux DU chip selects
#define	DU_CSDE			0x08
#define PORTD_DIRV		(DU_CSAB|DU_CSDE)
#define	PORTD_DENV		(DU_CSAB|DU_CSDE)
#define	PORTD_PURV		(0)

// Port E defines
// AIN9 is the input from the ambient light sensor
// PE5 is 1PWM3, drives HC238 to PWM the segment brightness
#define	AIN2			0x02						// PE1 = analog in (light level)
#define AIN9			0x10						// PE4 = TBD
#define	LED_PWM			0x20						// PE5 = segment PWM
#define PORTE_DIRV		(LED_PWM)
#define	PORTE_DENV		(LED_PWM)
#define	PORTE_PURV		(0)

// Port F defines
// User switch inputs (TBD)
// RGB LED (binary second hand)
//	Tiva launchpad LEDs
#define SW2				0x01						// PF0 = user SW2
#define NPXL_DO			0x02						// PF1 = neopixel data out (via SPI)
//#define LEDG			0x04						// PF2 = green
#define SW3			0x08						// PF3 = blue
#define	SW1				0x10						// PF4 = user SW1
#define PORTF_DIRV		(NPXL_DO)
#define	PORTF_DENV		(NPXL_DO|SW1|SW2|SW3)
#define	PORTF_PURV		(SW1|SW2|SW3)
#define	NOSW			0x00

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

#ifndef MAIN_C
extern U16	app_timer1ms;		// app timer
extern U16	xm_timer;			// xmodem timer
extern char	bchar;				// global bchar storage
extern char	swcmd;				// global swcmd storage
extern S8	handshake;			// xon/xoff enable
extern S8	xoffsent;			// xoff sent
#endif


//-----------------------------------------------------------------------------
// main.c Fn prototypes
//-----------------------------------------------------------------------------

void Init_Device(void);
void process_IO(U8 flag, U8 iplpio);
U8 set_7seg(char c);
void waitpio(U16 waitms);
void wait(U16 waitms);
void wait2(U16 waitms);
U16 getipl(void);
float get_imin(U8 flag);
float get_imax(U8 flag);
U8 wait_reg0(volatile uint32_t *regptr, uint32_t clrmask, U16 delay);
U8 wait_reg1(volatile uint32_t *regptr, uint32_t setmask, U16 delay);
void led_rgb(S16 ledred, S16 ledblu, S16 ledgrn);
void set_timez(S16 zone);
S16 get_timez(void);
S32 get_rtc(void);
void set_cal(S32 scal);
S32 read_cal(void);
void set_tdebug(void);
void set_rtc(S32 time);
void set_tsip_timer(U16 tval);
U16 get_tsip_timer(void);
U16 get_resync(void);
U16 get_adcbuf(U8 idx);
void store_eeprom(void);
void init_eeprom(void);
void set_led_msg(U8 msgflag, char* ptr, U8 dp);
void set_ppsflag(void);

void SSI1_ISR(void);
void Timer2_ISR(void);
void rtc_1024_isr(void);


//void npxl_datas(U32 mask, U8 sending, U32 data);
//U8 npxl_sending(void);

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------

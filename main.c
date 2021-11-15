/********************************************************************
 ************ COPYRIGHT (c) 2021 by ke0ff, Taylor, TX   *************
 *
 *  File name: main.c
 *
 *  Module:    Control
 *
 *  Summary:
 *  This is the main code file for the GPS slaved wall-clock.
 *   This device controls a 4 digit, 7segment LED (with colon).  The segments
 *   are MUX'd and a PWM output drives the anode switches to control brightness.
 *   GPS data comes in on a secondary UART for processing.  The system calculates
 *   a correction factor that is used to trim the clock to improve its free-run
 *   accuracy when GPS data is unavailable.
 *
 *  The Launchpad RGB LED *was* to be used as a "binary counter display" to produce
 *   a relative indication of seconds from 0 to 59.  This has been abandoned in favor
 *   of a NeoPixel, 16 segment ring.  R, G, and B are still the lsb to msb
 *   of the 3 bit count that divides the minute.  However, in lieu of brightness, the
 *	 16 LEDs tick off fine increments of about 1/2 sec (each 16 LED color "segment" is
 *	 7.5 seconds).
 *	By observing the LED color and the transitions, it is possible to determine the current
 *	 seconds value to within a couple of seconds (a bit of practice required).  The clock
 *	 logic "triggers" the seconds hand by using a seconds flag created in the timer2 isr.
 *	 This will move to the 1024Hz TOD interrupt once that sub-system is integrated.
 *  NeoPixel data protocol uses an 800KHz bi-phase format.  SSI1 is used to generate 1
 *   NeoPixel bit for 8 SSI data bits.  Thus, the SSI clock runs at 8x the 800KHz clock
 *   rate or at 6.4 MHz.  Only SSITX is enabled.  All other SSI GPIOs are left dormant
 *   for other GPIO uses.  None of the other SSI signals are needed for the NeoPixel protocol.
 *
 *  The TOD clock self-calibrates on a 12 hour cycle that starts when the clock error is more
 *   than ERRTCLIM.  If GPS is valid at the end of the 12 hour cycle, an hourly correction
 *   factor is calculated.  At the beginning of each hour, this correction factor is added
 *   to the rtc register to improve the free-run accuracy of the system.
 *
 *	The system uses GPS data to initialize the TOD clock on IPL.  The system waits 1 minute
 *	 for GPS.  If no lock, the time is pulled from the external battery-backed RTC chip.
 *	 This sets a flag such that GPS data will be used as soon as it is available regardless
 *	 of the TOD error.
 *
 *  The host system is a Tiva EK-TM4C123XL LaunchPad Evaluation Module that
 *   is attached to a host board via the dual-row headers present on the module.
 *   The host board features a 5V, 1A power supply and connectors for power, serial
 *   data, and LED module.  The system requires 12V (minimum 11.5V) regulated DC to supply
 *   power to the 7-segment displays.
 *
 *  The PC serial connection runs at 115.2 Kbaud via the LP ICDI JTAG I.F.
 *   and the Stellaris USB serial port (using UART0).  This is used for debug and system
 *   configuration.
 *
 *  The Tiva LP board features two user switches <TBD>
 *
 *  Project scope revision history:
 *    <VERSION 0.5>
 *    11-14-21 jmh:  Modified upper threshold of light sensor to reduce the "motor-boating" feedback of the LED
 *    					brightness when the room is dark and the sensor sees the LED light reflected from nearby
 *    					objects.
 *    				 re-worked DOW calculator to correct result errors.  Tested to year 2404 on 1/1, 2/28-29, 3/1,
 *    				 	and 21/31 for several years between 2000 and 2404 (using the "Q" test command via the CLI).
 *    				 Changed reset banner from "GPS Slaved RTC..." to "GPS Slaved Wall Clock..."
 *    				 Fleshed out help screen cmd list.
 *    				 Added LED software version string to version.h and pointer return function, ledSWvers(), to
 *    				 	localize the LED version string with the serial version strings for easier update.
 *
 *    <VERSION 0.3 & 0.4>
 *    -- rev-notes lost --
 *    General recollections (11-13-21):
 *    some work was done to improve the serial comms with the AUX DU.  The MOTO LED driver chip needs a hardware
 *    fix as it cannot be reset if there is a wonky serial message.  Need to try more power supply bypassing and
 *    filtering of the SPI clock.
 *
 *    <VERSION 0.2>
 *    01-08-19 jmh:  Re-worked delrtc math to address GPS midnight issue.
 *    01-02-19 jmh:  Added local_midnight and eod to the debug display to track why rtc exceeded end of day value.
 *    				 Changed local_midnight calc to use #define for end of day (only eod should have scalp
 *    				 	information).
 *    				 Added GPS status display branches to aux du logic tree.
 *    				 Added TSIP valid flag that goes false when no data is received (GPS dead).
 *    01-02-19 jmh:  Changed how scalp works.  Added "eod" register...rtc_scalp is added to END_OF_DAY_1024
 *    					to set the value for eod.  Begining of day (e.g., lcl_midnight) now operates on "0"
 *    					reference.  This removes the paradox conflicts if rtc_scalp is the "wrong" polarity.
 *    01-01-19 jmh:  Several bug fixes.  I've forgotten them by now, unfortunately...
 *    12-30-18 jmh:  Added SW3 support to cycle thru AUX DU display modes.
 *    				 Added AUX DU display modes: GPS, UTX (xrtc data), Local, GPS status, and SW version.
 *    12-29-18 jmh:  Connected XRTC /INT pin to PA6.  Added code to config xrtc to generate a 1sec pulse
 *    					to be used to pace the display update when GPS is off-line.  Added code to trap
 *    					falling edge of PA6 and trigger display updates.
 *    				 Cleaned up xrtc code to use #defines for register addresses.
 *    				 set_sec_ring() still had errors.  Corrected and checked - now working.
 *    				 Restructured xrtc process check to align to the xrtc INT pulse to keep synch.
 *    				 Moved colon toggle to xrtc ISR to better synchronize with seconds transition.
 *    				 Changed minflag = 1 to be in the process_IO loop and thus synchronized with the xrtc
 *    				 	or GPS sec register == 0.
 *    12-28-18 jmh:  Debugged auto-bright logic.  Several latent issues cleared up.
 *    				 Added synch-interlock for send_npxl() to clear up occasional random flicker in main
 *    				 	7seg display.
 *    				 Added aux du update.  Displays GPS/UTC date/year/time.
 *    12-27-18 jmh:  Completed latest round of HDWR mods including NPXL driver, light sensor on FP,
 *    					and PBSW cabling. NPXL ring is mounted to the main clock panel and is operational.
 *    				 Added support to the du.c code to allow the brightness Fn to also set/clr the DP
 *    				 	segments.
 *    				 Brightness controls now function for all LEDs
 *    				 Clock now running on 1024Hz ISR.  ms ISR is for non-timekeeping functions.
 *    12-17-18 jmh:  Added support for NeoPixel string (a 16 LED ring) using SSI1TX
 *    				 Removed RGB pwm support
 *    				 Produced ring seconds pattern and implemented using sec_tic_flag and sec_tic() Fn.
 *    <VERSION 0.1>
 *    11-25-18 jmh:  Added FF89UX LEDU as auxiliary DU.  Required some re-mapping of GPIO to get
 *    				 SPI to line up (using a BB SPI for now).  The AUX DU offers a large 7-digit field
 *    				 	(red), a medium 3-digit field (amber), a medium 6 digit field (amber), a small
 *						4-digit field (green), and a single indicator (amber) (spfn).
 *    				 All displays now perform a 3-sec lamp test on power-up.
 *    11-17-18 jmh: *Need to see what are the effects of hourly CAL updates on the sec-hand
 *    					--> +3673 scalp does not disrupt sec-hand process at top of hour.
 *    				 DST activities:
 *    					copied ACU rtc.c/rtc.h sources and began modifying to work with GPS values
 *    					added CLI cmd to set start/stop params
 *    					--> trap spots to determine DST status, IFF auto dst is enabled:
 *    						- GPS RE-SYNC
 *    						- manual set-time
 *    						- at 01:00 LCL
 *    						- at 02:00 LCL
 *    				 	debugged isdst().  This can be called from each place to set/clear dst, which will
 *    					cause 1 or 0 to be added with the time zone calc.
 *    				 	need to create registers to track MM/DD/YYYY when GPS is off-line.  These "shadow"
 *    						registers would run always, and are the primary source for DST determination.
 *    				 Created actions for SW1/SW2:
 *    				 	SW1 = dst config loop (auto/man, dst = 1/0, time zone, save to eeprom)
 *    				 	SW2 = GPS status loop.
 *    				 	SW1+SW2 = resync GPS (displays resync count...if SW1 pressed during count disp, a resync is executed).
 *    				 Modified colon flash to be coincident with GPS PPS when GPS valid.  ppsflag
 *    				 	is set by the tsip code when the primary timing packet is received.  This
 *    				 	is used to trigger the colon to come on.  the colon timer then turns it off
 *    				 	500ms later.
 *    11-15-18 jmh:  digit PWM (PE5) debugged and now works
 *    				 blank signal (PA7) implemented during digit switch events - HOWEVER, there
 *    				 	are likely capacitive effects on the N and PFETS that are causing some
 *    				 	observable ghosting.  NEED TO LOOK AT HARDWARE CHANGES FOR THIS
 *    				 AIN9 (PE4) and AIN2 (PE1) are now captured in SW.  sensor to PE1.  Range is small,
 *    				 	roughly 0 to 110.  Added auto brt/dim state machine to process_IO().
 *    				 Added SW1/SW2 closure detect.
 *    				 Added EEPROM I/O for critical variables.
 *    				 Provided a "msg mode" to the display and debugged the alpha segment codes.
 *    11-09-18 jmh:  Basic debug complete.  Need to check run accuracy over several days.
 *    					initial check looks good.
 *    10-26-18 jmh:  Adapted from LIB project.  Removed most of the irrelevant code and
 *    				  #defines.  S-Record and Xmodem code remains since these elements are
 *    				  difficult to sever.
 *    				 TA and TH code is drawn directly from the HFB, the ADS1014 code is
 *    				  new.
 *    10-16-15 jmh:  creation date
 *
 *******************************************************************/

/********************************************************************
 *  File scope declarations revision history:
 *    10-01-15 jmh:  creation date
 *
 *******************************************************************/

//-----------------------------------------------------------------------------
// main.c
//  Receives CLI entries and dispatches to the command line processor.
//  UART0 is used for user interface and data download at 115,200 baud
//	(with xon/xoff handshaking).
//
//  CLI is a simple maintenance/debug port with the following core commands:
//      VERS - interrogate SW version.
//		See "cmd_fn.c" for details
//
//  Interrupt Resource Map:
//      Timer2 int provides 1ms, count-down-from-N-and-halt application timers
//      UART0 is host CLI serial port (via the ICDI USB<->SERIAL circuit)
//		UART1 is the GPS serial port (RX only)
//
//  I/O Resource Map:
//      See "init.h"
//
//	Phase I SW: Basic functions:
//	1) PortE PWM: 1 PWM output controls 4, 7-segment LEDs via PFET switches.
//	2) PortF PWM: 1 SSI output controls a neopixel 16 LED ring
//	3) CLI via UART0 allows user to query system variables and execute test functions.
//	4) UART1 receives TSIP GPS data
//	5) PB[7:0] = segment drive, PA[6:5] = segment address, and PA7 = digit blank
//		Drives a scanned, mux'd 7-seg LED display (x4 digits) plus colon (dp's).
//
//	process_IO() handles TSIP processing and monitors internal rtc registers against the GPS time.
//	automatically calculates cal values for internal rtc and re-sets internal rtc if the error
//	against GPS time exceeds a pre-set limit.
//
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
// compile defines

#define	NORTC_DEBUG 0		// debug system ("1" = no RTC installed; "0" = RTC installed)

#define MAIN_C
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include <stdio.h>
#include <string.h>
#include "init.h"
#include "typedef.h"
#include "version.h"
#include "serial.h"
#include "cmd_fn.h"
#include "tiva_init.h"
//#include "I2C0.h"
#include "io.h"
#include "eeprom.h"
#include "tsip.h"
#include "adc.h"
#include "rtc.h"
#include "du.h"

//-----------------------------------------------------------------------------
// Definitions
//-----------------------------------------------------------------------------

//  see init.h for main #defines
#define	MAX_REBUF	4		// max # of rebufs
#define	MAX_BARS	7		// max # of led bars
#define	GETS_INIT	0xff	// gets_tab() static initializer signal
#define DAC_LDAC 20			// dac spi defines
#define DAC_PWRUP 21
#define DAC_CLR 22

enum gps_idx{gsec,gmin,ghour,gmonth,gday,gtref,gpref,guinfo,guoffsH,guoffsL};

//-----------------------------------------------------------------------------
// Local Variables
//-----------------------------------------------------------------------------

// Processor I/O assignments
//bitbands
#define DUT_ON  (*(volatile uint32_t *)(0x40058000 + (0x04 * 4)))
#define DUT_OFF (*(volatile uint32_t *)(0x40058000 + (0x08 * 4)))
#define DUT_SCK (*(volatile uint32_t *)(0x40058000 + (0x10 * 4)))
#define DUT_D   (*(volatile uint32_t *)(0x40058000 + (0x20 * 4)))
#define DUT_Q   (*(volatile uint32_t *)(0x40058000 + (0x40 * 4)))
#define DUT_CS  (*(volatile uint32_t *)(0x40058000 + (0x80 * 4)))

//-----------------------------------------------------------------------------
// Global variables (extern conditionals are in init.h)
//-----------------------------------------------------------------------------
U16		auxdu_title_timer;				// aux du timer
U16		xm_timer;						// xmodem timer
U16		tsip_timer;                     // tsip com timeout
char	bchar;							// break character trap register - traps ESC ascii chars entered at terminal
char	swcmd;							// software command flag
S8		handshake;						// xon/xoff enable
S8		xoffsent;						// xoff sent

//-----------------------------------------------------------------------------
// Local variables in this file
//-----------------------------------------------------------------------------
U16		procio_timer1ms;				// process IO timer
U32		abaud;							// 0 = 115.2kb (the default)
U8		iplt2;							// timer2 ipl flag
U8		iplgps;							// gps not yet valid on IPL
U16		waittimer;						// gp wait timer
U16		gpsvalid_timer;					// gps stale timer
U32		gpscal_timer;					// GPS cal timer
U8		brt_timer;						// auto brt pacing timer
U8		pbsw_timer;						// pbsw state timer
U16		msg_timer;						// msg progress timer
U16		ipl_timer;						// ipl timer...counts down to zero from reset
U16		colonrate;						// prescales the ms rate to the 7seg LED colon flash rate
S8		err_led_stat;					// err led status
uint8_t idx;
U16		ipl;							// initial power on state
U16		kupper;							// ADC min/max tracking regs
U16		klower;
S16		tzone;							// time zone
S32		local_midnight;					// time when rtc (GPS time) register reaches local midnight (for shadow calendar)
										// = tzone * 3600000L
S8		dst;							// =+1 when DST is in effec, otherwise, = 0.
U8		dst_auto;						// dst autocorrect flag, 1 = autocorrect enabled, 0 = manual correct required
U16		minbrt;							// ambient light min/max
U16		maxbrt;
S32		rtcgsp;							// real-time clock GPS setpoint (milli-seconds since midnight)
S32		rtc;							// real-time clock (milli-seconds since midnight)
S32		rrtc;							// interlocked read real-time clock register (milli-seconds since midnight)
U8		read_rtc;						// rtc read interlock flag
U8		write_rtc;						// rtc write interlock flag
S32		rtc_scalp;						// once-per-day correction factor (+/-).  This is the value loaded into rtc at the start of the day.
U8		calday;							// cal day counter
S32		eod;							// end of day reg = END_OF_DAY_1024 + rtc_scalp
U8		led_string[4];					// string holding the LED segment codes
U8		minflag;						// minute boundary flag
U8		gsod;							// gps-start-of-day valid flag
U8		gpscal_flag;					// gpscal = true flag
U8		tdebug;							// time debug flag
U16		resync;							// resync counter
U16		adc_buf[8];						// ADC raw input buffer
U8		led_msg;						// led message flag
U8		msg_state;						// message state timer
U8		ppsflag;						// GPS pps align flag
U8		auxupd_flag;					// aux du update flag
U8		aux_mode;						// aux du mode (state)
U8		abuf[11];						// aux du strings
U8		sec_flag;
#define	NPXL_ARRAY_LEN	16
U32		npxl_array[NPXL_ARRAY_LEN];		// neopixel data array
U32		npxl_data;						// tx data reg
U8		np_idx;							// neopixel array index
U32		np_mask;						// neopixel serial bit mask
U8		np_sending;						// neopixel sending status
U8		np_state;						// neopixel bit state
U8		np_stop;						// timer regs
U8		npxl_flag;						// NPXL send interlock
#if	(NORTC_DEBUG == 1)
#define	SEC_TIC_RATE (75 * 1000 / 160)	// #ms per ring tic
#else
#define	SEC_TIC_RATE (75 * 1024 / 160)	// #ms per ring tic
#endif
U8		sec_tic_flag;					// sec tic flag
//		Count sequence is msb = blue, lsb = red.  Binary count at abt 0.5 sec rate.
//		ring count colors:	0x--ggrrbb
//						off  red     green     yel       blue  pink    cyan      white
U32		sec_ring[8];					// = { 0x0, 0x2000, 0x200000, 0x202800, 0x20, 0x2020, 0x140028, 0x202020 };
U8		sec_idx;
U16		sec_prscl;
U8		xbuf[11];		// ext RTC/GPS bufer temp

//-----------------------------------------------------------------------------
// Local Prototypes
//-----------------------------------------------------------------------------

void process_LEDMSG(void);
void Timer_Init(void);
void Timer_SUBR(void);
char *gets_tab(char *buf, char *save_buf[3], int n);
void isdst_auto(void);
void set_lcl_midnight(void);
void force_resync(void);
void send_npxl(void);
void sec_tic(void);
void set_sec_tic(U16 secs);
void sec_brt(U8 bright);
void clear_auxdu(void);

//*****************************************************************************
// main()
//  The main function runs a forever loop in which the main application operates.
//	Prior to the loop, Main performs system initialization and boot status
//	announcement.
//	The loop calls gets_tab() which polls for user input and runs the process loops (if any).
//	When gets_tab() returns, it means that the user has pressed "Enter" and their input
//	is ready to be processed by the cmd_fn routines.
//	The loop also processes baud rate changes trapped by gets_tab().
//
//	The CLI maintains and processes re-do buffers (4 total) that are accessed by the
//	TAB key.  This allows the last 4 valid command lines to be recalled and executed
//	(after TAB'ing to desired recall command, press ENTER to execute, or ESC to
//	clear command line).
//	Autobaud rate reset allows user to select alternate baud rates after reset:
//		115,200 baud is default.  A CR entered after reset at 57600, 38400,
//		19200, or 9600 baud will reset the system baud rate and prompt for user
//		acceptance.  Once accepted, the baud rate is frozen.  If rejected, baud rate
//		returns to 115200.  The first valid command at the default rate will also
//		freeze the baud rate.  Once frozen, the baud rate can not be changed until
//		system is reset.
//*****************************************************************************

int main(void){

	volatile uint32_t ui32Loop;
	U8		i;						// temp loop var
    uint8_t bar_state = 0;			// indicates the current LED state (0-7)
//    uint8_t	tempi;					// tempi
    char	buf[80];				// command line buffer
    char	rebuf0[80];				// re-do buffer#1
    char	rebuf1[80];				// re-do buffer#2
    char	rebuf2[80];				// re-do buffer#3
    char	rebuf3[80];				// re-do buffer#4
    char	got_cmd = FALSE;		// first valid cmd flag (freezes baud rate)
    U8		argn;					// number of args
    char*	cmd_string;				// CLI processing ptr
    char*	args[ARG_MAX];			// ptr array into CLI args
    char*	rebufN[4];				// pointer array to re-do buffers
    U16		offset = 0;				// srecord offset register
    U16		cur_baud = 0;			// current baud rate

    ipl = proc_init();								// initialize the processor I/O
    proc_init_NVIC_en(UART_NVIC|TIMER2_NVIC);		// enable interrupts
    ipl_timer = IPL_WAIT;							// hold off xRTC resync for GPS to get a chance
	GPIO_PORTA_DATA_R |= RTC_CSN;	 				// bring RTC CS hi
    for(i=0;i<4;i++){								// lamp test leds
    	led_string[i] = 0xff;
    }
    rebufN[0] = rebuf0;								// init CLI re-buf pointers
	rebufN[1] = rebuf1;
	rebufN[2] = rebuf2;
	rebufN[3] = rebuf3;
//	I2C_Init();										// init I2C system
	dispSWvers(); 									// display reset banner
	wait(10);										// a bit of delay..
	rebuf0[0] = '\0';								// clear cmd re-do buffers
	rebuf1[0] = '\0';
	rebuf2[0] = '\0';
	rebuf3[0] = '\0';
	sec_brt(0xff);									// init sec hand
	bcmd_resp_init();								// init bcmd response buffer
	wait(10);										// a bit more delay..
	while(gotchr()) getchr();						// clear serial input in case there was some POR garbage
	gets_tab(buf, rebufN, GETS_INIT);				// initialize gets_tab()
	process_IO(0xff,0xff);							// init process_io
	swcmd = 0;										// init SW command
	tzone = (S16)(eerd(EE_TIME_ZONE) & 0x0000ffffL); // init tzone from eeprom
	dst_auto = (U8)(eerd(EE_DSTAUTO) >> 16);		// init dst mode from eeprom
	dst = (U8)(eerd(EE_DSTAUTO) >> 24);				// init dst mode from eeprom
	rtc_scalp = eerd(EE_SCALP);						// init scalp from eeprom
	eod = END_OF_DAY_1024;
	minbrt = (U16)(eerd(EE_MINBRT) & 0x0000ffffL);	// init minbrt from eeprom
	maxbrt = (U16)(eerd(EE_MAXBRT) >> 16);			// init maxbrt mode from eeprom
	get_ee_dst();									// recall DST schedule settings from eeprom
	set_lcl_midnight();								// calc lcl midnight offset from GPS time
	isdst_auto();									// override dst if autodst enabled
	iplt2 = 1;										// IPL inits
	iplgps = 1;
	ppsflag = 0;
	auxupd_flag = 0;
	sec_tic_flag = 0;
	minflag = 0;
	tdebug = FALSE;
	rtcgsp = -1L;									// invalidate GPS set-point
	PWM1_1_CMPB_R = 0;								// max brt LED
	rtc_init();										// init RTC clk out
	aux_mode = 0;
	for(i=0;i<NPXL_ARRAY_LEN;i++){					// all-on lamp test
		npxl_array[i] = 0x00202020;
	}
    send_npxl();
	init_du();
	for(i=0; i<4; i++){
		led_string[i] = 0x40;						// init LED segment string (invalid)
	}
	set_sbright('a');								// all DPs off
	set_sbright('i');
	set_sbright('j');
	parse_fmain("GPS AUX003");						// post machine banner (last 3 digs are SW rev)
	parse_fpl("LEDU");
	parse_fsub("  =J0EH=");
    proc_init_NVIC_en(GPIOA_NVIC);					// enable interrupts
	while(iplt2);									// wait for LED control to boot...
    npxl_array[0] = 0x00040000;						// color range test - set ring test colors
    npxl_array[1] = 0x00080000;
    npxl_array[2] = 0x00100000;
    npxl_array[3] = 0x00200000;
    npxl_array[4] = 0x00000400;
    npxl_array[5] = 0x00000800;
    npxl_array[6] = 0x00001000;
    npxl_array[7] = 0x00002000;
    npxl_array[8] = 0x00000004;
    npxl_array[9] = 0x00000008;
    npxl_array[10] = 0x00000010;
    npxl_array[11] = 0x00000020;
    npxl_array[12] = 0x00040404;
    npxl_array[13] = 0x00080808;
    npxl_array[14] = 0x00101010;
    npxl_array[15] = 0x00202020;
    send_npxl();
    wait(SEC2);
	for(i=0;i<NPXL_ARRAY_LEN;i++){					// init ring to start seconds loop
		npxl_array[i] = 0; //0x00080808;
	}
    send_npxl();
    sec_idx = 0;
    sec_prscl = SEC_TIC_RATE;
    calday = 0;
	arch_gps(0xff);									// init GPS archive
	// main loop
    while(swcmd != SW_ESC){
		putchar_b(XON);
		buf[0] = '\0';
		putss("rtc>");										// prompt
    	if(bar_state > MAX_BARS) bar_state = 0;
		cmd_string = gets_tab(buf, rebufN, 80); 			// get cmd line & save to re-do buf
		if(!got_cmd){										// if no valid commands since reset, look for baud rate change
			if(cur_baud != abaud){							// abaud is signal from gets_tab() that indicates a baud rate change
				if(set_baud(abaud)){						// try to set new baud rate
					puts0("");								// move to new line
					dispSWvers();							// display reset banner & prompt to AKN new baud rate
					while(gotchr()) getchr();				// clear out don't care characters
					putss("press <Enter> to accept baud rate change: ");
					while(!gotchr());						// wait for user input
					puts0("");								// move to new line
					if(getchr() == '\r'){					// if input = CR
						cur_baud = abaud;					// update current baud = new baud
						got_cmd = TRUE;						// freeze baud rate
					}else{
						set_baud(0);						// input was not a CR, return to default baud rate
						cur_baud = abaud = 0;
					}
				}else{
					abaud = cur_baud;						// new baud rate not valid, ignore & keep old rate
				}
			}else{
				got_cmd = TRUE;								// freeze baud rate (@115.2kb)
			}
		}
		argn = parse_args(cmd_string,args);					// parse cmd line
		if(x_cmdfn(argn, args, &offset)) got_cmd = TRUE;	// process cmd line, set got_cmd if cmd valid
    }
    return 0;
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// gets_tab puts serial input into buffer, UART0.
//-----------------------------------------------------------------------------
// Main loop for command line input.
// waits for a chr and puts into buf.  If 1st chr = \t, copy re-do buf into
//  cmdbuf and cycle to next re-do buf.  if more than n chrs input, nul term buf,
//	disp "line too long", and return.  if \n or \r, copy buf to save_buf & return
//  returns buf (pointer).
//	if n == 0xff, initialize statics and exit.
//
//	11/08/13: Modified to support 4 (MAX_REBUF) rolling cmd save buffers
//	11/15/13: Modified to support auto-baud detect on CR input
//		For the following, each bit chr shown is one bit time at 115200 baud (8.68056us).
//			s = start bit (0), p = stop bit (1), x = incorrect stop, i = idle (1), bits are ordered  lsb -> msb:
//	 the ascii code for CR = 10110000
//			At 115.2kb, CR = s10110000p = 0x0D
//
//			At 57.6 kb, CR = 00110011110000000011 (1/2 115.2kb)
//			@115.2, this is: s01100111ps00000001i = 0xE6, 0x80
//
//			At 38.4 kb, CR = 000111000111111000000000000111 (1/3 115.2kb)
//			@115.2, this is: s00111000p11111s00000000xxxiii = 0x1c, 0x00
//
//			At 19.2 kb, CR = 000000111111000000111111111111000000000000000000000000111111 (1/6 115.2kb)
//			@115.2, this is: s00000111piis00000111piiiiiiiis00000000xxxxxxxxxxxxxxxiiiiii = 0xE0, 0xE0, 0x00
//
//			At 9600 b,  CR = 000000000000111111111111000000000000111111111111111111111111000000000000000000000000000000000000000000000000111111111111 (1/12 115.2kb)
//			@115.2, this is: s00000000xxxiiiiiiiiiiiis00000000xxxiiiiiiiiiiiiiiiiiiiiiiiis00000000xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxiiiiiiiiiiii = 0x00, 0x00, 0x00
//
//		Thus, @ 57.6 kb, a CR = 0xE6 followed by 0x80
//			  @ 38.4 kb, a CR = 0x1C followed by 0x00
//			  @ 19.2 kb, a CR = 0xE0 followed by 0xE0 (plus a 0x00)
//			  @ 9600 b, a  CR = 0x00 followed by 0x00 (plus a 0x00)
//
//		NOTE: gets_tab is only used for command line input and thus should not
//		see non-ascii data under normal circumstances.

char *gets_tab(char *buf, char *save_buf[], int n)
{
	char	*cp;
	char	*sp;
	char	c;
	int		i = 0;
	static	U8   rebuf_num;
	static	U8	 last_chr;

//	if((rebuf_num >= MAX_REBUF) || (n == GETS_INIT)){ // n == 0xff is static initializer signal
	if(n == GETS_INIT){ // n == 0xff is static initializer signal
		rebuf_num = 0;									// init recall buffer pointer
		last_chr = 0xff;								// init to 0xff (not a valid baud select identifier chr)
		return buf;										// skip rest of Fn
	}
    cp = buf;
    sp = save_buf[rebuf_num];
    do{
        c = getch00();									// look for chrs and run process_IO()
        switch(c){
			case 0xE0:									// look for 19.2kb autoselect
				if(last_chr == 0xE0){
					abaud = 19200L;
					c = '\r';
				}
				break;

			case 0x00:									// look for 38.4kb or 9600b autoselect
				if(last_chr == 0x1C){
					abaud = 38400L;
					c = '\r';
				}else{
					if(last_chr == 0x00){
						abaud = 9600L;
						c = '\r';
					}
				}
				break;

			case 0x80:									// look for 57.6kb autoselect
				if(last_chr == 0xE6){
					abaud = 57600L;
					c = '\r';
				}
				break;

            case '\t':
				if(i != 0){								// if tab, cycle through saved cmd buffers
					do{
						i--;							// update count/point
						cp--;
						if((*cp >= ' ') && (*cp <= '~')){
							putchar0('\b');				// erase last chr if it was printable
							putchar0(' ');
							putchar0('\b');
						}
					}while(i != 0);
					cp = buf;							// just in case we got out of synch
				}
				//copy saved string up to first nul, \n, or \r
				if(--rebuf_num == 0xff){
					rebuf_num = MAX_REBUF - 1;
				}
				sp = save_buf[rebuf_num];
				while((*sp != '\0') && (*sp != '\r') && (*sp != '\n')){
					putdch(*sp);
					*cp++ = *sp++;
					i++;
				}
                break;

            case '\b':
            case 0x7f:
                if(i != 0){								// if bs & not start of line,
                    i--;								// update count/point
                    cp--;
                    if((*cp >= ' ') && (*cp <= '~')){
                        putchar0('\b');					// erase last chr if it was printable
                        putchar0(' ');
                        putchar0('\b');
                    }
                }
                break;

            case '\r':									// if cr, nul term buf & exit
            case '\n':									// if nl, nul term buf & exit
                i++;
                *cp++ = c;
                break;

            case ESC:									// if esc, nul buf & exit
                cp = buf;
                c = '\r';								// set escape condition
				i = 0;
                break;

            default:
                i++;
                *cp++ = c;								// put chr in buf
                putdch(c);								// no cntl chrs here
                break;
        }
		last_chr = c;									// set last chr
    } while((c != '\r') && (c != '\n') && (i < n));		// loop until c/r or l/f or buffer full
	if(i >= n){
		puts0("!! buffer overflow !!");
		*buf = '\0';									// abort line
	}else{
		puts0("");										// echo end of line to screen
		*cp = '\0';										// terminate command line
		if((*buf >= ' ') && (*buf <= '~')){				// if new buf not empty (ie, 1st chr = printable),
			strncpy(save_buf[rebuf_num], buf, n);		// copy new buf to save
			if(++rebuf_num >= MAX_REBUF) rebuf_num = 0;
		}
	}
    return buf;
}

//-----------------------------------------------------------------------------
// process_IO() processes system I/O
//	this fn is called while the system is waiting for user input and does all of the
//	tasks that need to happen while not dealing with a user command.
//	This is where any automatic activities are placed.  A system timer should be used
//	to pace each activity.  Also, state machine loops are generally the easiest way to
//	break tasks into pieces that allow this Fn to execute without tying up CPU time
//	for indefinite periods.
//-----------------------------------------------------------------------------
void process_IO(U8 flag, U8 iplpio){
			U8*	gtptr;			// pointer to gps time registers
			U8*	gsptr;			// pointer to GPS status regs
	static	S32	grtc;			// gps rtc register (ms)
	static	U8	brt_state;		// bright state machine
	static	U8	pbsw_state;		// pbsw state
	static	U8	pbsw_state_last;	 // pbsw last state
	static	U8	pbsw_action;	// pbsw action capture
	static	U8	xpps_edge;		// xrtc pps edge detect
	static	U8	dispmode;		// aux display mode reg
	static	U8	sw3state;		// sw3 state reg
#define	MAX_AVE_BUF	8
	static	U16	brt_buf[MAX_AVE_BUF];	// light level sensor averaging buffer
	static	U8	brt_idx;		// fill index for sensor averaging buffer
			U8	pb;				// pb temp
			S32	delrtc;			// |delta| gps/rtc time
			S32	stemp32;
			S32	xrtc;			// ext rtc temp
			U8	j;				// temps
			U8	i;
			U8	k;
			U8	updt = 0;		// update time flag!
			U16	tt;				// temp16
			U16	bb;
			signed char	si;		// signed temp
			S16	utcoffs;		// utc offset temp
			S16	st;				//
			uint32_t ii;		// u32 temp
			uint32_t jj;		// u32 temp
			char tbuf[5];		// time bufer temp
			char pbuf[45];		// print buffer temp

	// auto-bright IPL init
	if(iplpio){
		brt_timer = 0;
		brt_state = BRT_INIT;
		pbsw_state = PBSW_INIT;
		for(i=0; i<MAX_AVE_BUF; i++){
			brt_buf[i] = 0;
		}
		brt_idx = 0;
	}
	if(sec_tic_flag){
		sec_tic_flag = 0;
		sec_tic();
	}
	// TSIP data process loop
	gtptr = get_gtime_ptr();
	switch(flag){
	case 0xff:
	default:														// ipl
		tsip_main(TRUE);
		xpps_edge = GPIO_PORTA_DATA_R & RTC_INTN;
		dispmode = 0;
		sw3state = SW3;
		break;

	case 0:															// normal exec
		if(tsip_main(0) && (get_gpstat() == DOING_FIXES)){			// process tsip
			// process new GPS seconds data
			if(iplgps){
				for(i=0;i<2;i++){
					get_gotime(TRUE);
					do{
						tsip_main(0);
					}while(!get_gotime(0));
				}
			}
			if(get_gotime(0)){										// check if rtc is valid
//				ppsflag = TRUE;										// set PPS semaphore
//				auxupd_flag = TRUE;
				rtc_rw(xbuf, XREAD);								// get xrtc update for aux display
				grtc = (S32)gtptr[ghour] * MSHOUR_1024;					// calc gps ms since midnight
				grtc += (S32)gtptr[gmin] * MSMIN_1024;
				grtc += (S32)gtptr[gsec] * MS_COUNT_1024;
				utcoffs = get_utc_offs();
				grtc -= (S32)utcoffs * MS_COUNT_1024;				// with UTC correction
				if(grtc < 0){
					grtc += END_OF_DAY_1024;						// re-align to positive space
				}
				si = (S8)gtptr[gsec] - (S8)utcoffs;
				if(si < 0){											// test for begining of minute (UTC) to trigger display update
					si += 60;										// rollover negative
				}
				if(si > 59){
					si -= 60;										// rollover positive
				}
				if(si == 0){
					minflag = 1;
				}
				read_rtc = 1;
				while(read_rtc);									// grab rtc
				// calc error.  if-else tree deals with quadrant mis-alignment to deliver a correct magnitude and polarity
				//	to the delrtc variable (delrtc > 0 if rtc > gps)
				if((rrtc >= 0) && (grtc >= 0)){
					delrtc = rrtc - grtc;
				}else{
					if((rrtc < HOUR_01_1024) && (grtc >= HOUR_23_1024)){
						delrtc = rrtc + END_OF_DAY_1024 - grtc;
					}else{
						if((grtc < HOUR_01_1024) && (rrtc >= HOUR_23_1024)){
							delrtc = eod - rrtc - grtc;
						}else{
							delrtc = 99999999L;						// force error
						}
					}
				}

/*				if((rrtc > HOUR_23_1024) && (gtptr[ghour] == 0)){
					delrtc = grtc + eod - rrtc;						// if rtc is in 23rd hour and gps rolls over, re-adjust gps (un-roll it)
				}else{
					if((rrtc < CYC_HOUR_1024) && (gtptr[ghour] == 23)){
						delrtc = grtc - rrtc - eod; 				// if gps is in 23rd hour and rtc rolls over, re-adjust gps (un-roll it)
					}else{
						delrtc = grtc - rrtc;						// else, take the difference
					}
				}*/

				if(delrtc < 0L){
					stemp32 = delrtc * -1L;
				}else{
					stemp32 = delrtc;
				}
				if(((stemp32 > GERRTCLIM) || iplgps) && !((gtptr[ghour] == 23) && gtptr[gmin] > 57)){	// rtc error is too great OR ipl, reset rtc
					// update & display debug info
					resync += 1;
					sprintf(pbuf,"\nGPS re-sync %02u:%02u:%02u  %d",gtptr[ghour],gtptr[gmin],gtptr[gsec],rrtc);
					puts0(pbuf);
					sprintf(pbuf,"%02u/%02u/%02u",gtptr[gmonth],gtptr[gday],get_gyear());
					puts0(pbuf);
					sprintf(pbuf,"stemp32: %d, Gerrlim: %d, iplgps: %d",stemp32,GERRTCLIM,iplgps);
					puts0(pbuf);
					sprintf(pbuf,"resync count: %d",resync);
					puts0(pbuf);
					iplgps = 0;										// clear ipl flag
					rrtc = grtc;									// rtc follows GPS
					write_rtc = 1;
					while(write_rtc);
					rtcgsp = grtc;
					st = ((S16)gtptr[gsec] - utcoffs);				// calculate seconds position in base clocks
					if(st < 0){
						st += 60;
					}
					tt = (U16)st * MS_COUNT_1024;
					iplt2 = 1;
					set_sec_tic(tt);								// init sec hand
					set_gpsdate(tzone);								// update shadow calendar
					isdst_auto();									// do auto dst update if enabled
					// update xRTC whenever GPS does a correction
					// need to account for utcoffs...
/*					bb = get_gyear();								// update XRTC from GPS
					k = bb / 100;
					j = (U8)(bb%100);
					// do bin->dec conversion of time params
					sprintf(obuf,"%02u%02u%02u%02u%02u%02u%02u",gtptr[ghour],gtptr[gmin],gtptr[gsec],gtptr[gmonth],gtptr[gday],j,k);
					xbuf[3] = (obuf[0] & 0x0f) << 4;
					xbuf[3] |= (obuf[1] & 0x0f);
					xbuf[2] = (obuf[2] & 0x0f) << 4;
					xbuf[2] |= (obuf[3] & 0x0f);
					xbuf[1] = (obuf[4] & 0x0f) << 4;
					xbuf[1] |= (obuf[5] & 0x0f);
					xbuf[6] = (obuf[6] & 0x0f) << 4;
					xbuf[6] |= (obuf[7] & 0x0f);
					xbuf[4] = (obuf[8] & 0x0f) << 4;
					xbuf[4] |= (obuf[9] & 0x0f);
					xbuf[7] = (obuf[10] & 0x0f) << 4;
					xbuf[7] |= (obuf[11] & 0x0f);
					xbuf[8] = (obuf[12] & 0x0f) << 4;
					xbuf[8] |= (obuf[13] & 0x0f);
					rtc_stop(1);
					rtc_rw(xbuf, XWRITE);
					rtc_stop(0);*/
					updt = 1;
					gpscal_flag = 1;
					gpscal_timer = GPSCALTIME;
					arch_gps(0);									// update GPS archive
				}else{
					// process time calibration
					if(gpscal_flag && (gpscal_timer == 0)){			// only do this once per cal period
						gpscal_flag = 0;
						if(stemp32 > GPSCALLIM){					// if rtc error > limit..
							rtc_scalp += (SEC_WEEK * delrtc) / (S32)get_gpset(); // calc correction
							calday = 0;
//							eod = END_OF_DAY_1024 + rtc_scalp;
							sprintf(pbuf,"scalp update = %d",rtc_scalp);
							puts0(pbuf);
						}
					}
					if(tdebug){
						tdebug = FALSE;
						sprintf(pbuf,"\nStatus Gtime = %02u:%02u:%02u",gtptr[ghour],gtptr[gmin],gtptr[gsec]);
						puts0(pbuf);
						sprintf(pbuf,"     (g)urtc = %d",grtc);
						puts0(pbuf);
						sprintf(pbuf,"         rtc = %d",rrtc);
						puts0(pbuf);
						sprintf(pbuf,"       delta = %d",delrtc);
						puts0(pbuf);
						sprintf(pbuf,"         eod = %d",eod);
						puts0(pbuf);
						sprintf(pbuf,"   lcl midnt = %d",local_midnight);
						puts0(pbuf);
						sprintf(pbuf,"        xrtc = %02x:%02x:%02x",*(xbuf+3),*(xbuf+2),*(xbuf+1));
						puts0(pbuf);
						sprintf(pbuf,"       scalp = %d",rtc_scalp);
						puts0(pbuf);
					}
				}
				get_gotime(TRUE);									// clear TSIP messages
				gpsvalid_timer = GPS_FRESH;							// reset GPS data freshness timer
			}
		}
		break;
	}
	// process external RTC
	if((gpsvalid_timer == 0) & (ipl_timer == 0)){					// if no GPS and IPL wait period is expired...
		i = GPIO_PORTA_DATA_R & RTC_INTN;
		if(i != xpps_edge){
			xpps_edge = i;
			if(!xpps_edge){
				rtc_rw(xbuf, XREAD);
				ppsflag = TRUE;
				auxupd_flag = TRUE;
				read_rtc = 1;
				xrtc = (U32)(*(xbuf+1)&0x0f)+(((*(xbuf+1)>>4)&0x0f)*10);			// get TOD in sec
				xrtc += (U32)((*(xbuf+2)&0x0f)+(((*(xbuf+2)>>4)&0x0f)*10)) * 60L;
				xrtc += (U32)((*(xbuf+3)&0x0f)+(((*(xbuf+3)>>4)&0x0f)*10)) * 3600L;
				xrtc *= MS_COUNT_1024;												// now it's in ms
				if(*(xbuf+1) == 0){
					minflag = 1;
				}
				while(read_rtc);											// grab rtc
				if((rrtc > HOUR_23_1024) && (*(xbuf+3) == 0)){
					delrtc = xrtc + eod - rrtc;								// if rtc is in 23rd hour and xrtc rolls over, re-adjust xrtc (un-roll it)
				}else{
					if((rrtc < XERRTCLIM) && (*(xbuf+3) == 23)){
						delrtc = xrtc - rrtc - eod; 						// if xrtc is in 23rd hour and rtc rolls over, re-adjust rtc (un-roll it)
					}else{
						delrtc = xrtc - rrtc;								// else, take the difference
					}
				}
				if(delrtc < 0L){
					stemp32 = delrtc * -1L;
				}else{
					stemp32 = delrtc;
				}
				if(stemp32 > XERRTCLIM){									// rtc error is too great, reset rtc
					// update and display debug data
					resync += 1;
					sprintf(pbuf,"\nxRTC re-sync %02x:%02x:%02x  %d",*(xbuf+3),*(xbuf+2),*(xbuf+1),rrtc);
					puts0(pbuf);
					sprintf(pbuf,"%02x/%02x/%02x",*(xbuf+6),*(xbuf+4),*(xbuf+7));
					puts0(pbuf);
					sprintf(pbuf,"stemp32: %d, Xerrlim: %d, iplgps: %d",stemp32,XERRTCLIM,iplgps);
					puts0(pbuf);
					sprintf(pbuf,"resync count: %d",resync);
					puts0(pbuf);
//					do{														// synchronize to the seconds rollover
//						i = GPIO_PORTA_DATA_R & RTC_INTN;
//					}while(i);

//					j = *(xbuf+1);											// synchronize to the seconds rollover
//					do{
//						rtc_rw(xbuf, XREAD);
//					}while(j == *(xbuf+1));
					xrtc = (U32)(*(xbuf+1)&0x0f)+(((*(xbuf+1)>>4)&0x0f)*10);			// re-calc TOD
					xrtc += (U32)((*(xbuf+2)&0x0f)+(((*(xbuf+2)>>4)&0x0f)*10)) * 60L;
					xrtc += (U32)((*(xbuf+3)&0x0f)+(((*(xbuf+3)>>4)&0x0f)*10)) * 3600L;
					xrtc *= MS_COUNT_1024;
					rrtc = xrtc;
					write_rtc = 1;
					while(write_rtc);
		//			rtcgsp = grtc;
					tt = (U16)((*(xbuf+1)&0x0f) + (((*(xbuf+1)>>4)&0x0f)*10));
					iplt2 = 1;
					set_sec_tic(tt * MS_COUNT_1024);
					// update shadow calendar
					tt = (U16)((*(xbuf+8)&0x0f) + (((*(xbuf+8)>>4)&0x0f)*10)) * 100;
					tt += (U16)((*(xbuf+7)&0x0f) + (((*(xbuf+7)>>4)&0x0f)*10));
					set_xdate(tzone,(*(xbuf+6)&0x0f)+(((*(xbuf+6)>>4)&0x0f)*10),(*(xbuf+4)&0x0f)+(((*(xbuf+4)>>4)&0x0f)*10), tt, xrtc);
					isdst_auto();											// do auto dst update if enabled
					updt = 1;
				}else{
					// process time calibration
		//			if(gpscal_flag && (gpscal_timer == 0)){			// only do this once per cal period
		//				gpscal_flag = 0;
		//				if(stemp32 > GPSCALLIM){					// if rtc error > limit..
		//					rtc_scalp += delrtc / GPSCALTIME_HR;	// calc hourly correction
		//					puts0("scalp update");
		//				}
		//			}
					if(tdebug){
						tdebug = FALSE;
						sprintf(pbuf,"\nStatus Xtime = %02x:%02x:%02x",*(xbuf+3),*(xbuf+2),*(xbuf+1));
						puts0(pbuf);
						sprintf(pbuf,"     (x)urtc = %d",xrtc);
						puts0(pbuf);
						sprintf(pbuf,"         rtc = %d",rrtc);
						puts0(pbuf);
						sprintf(pbuf,"       delta = %d",xrtc-rrtc);
						puts0(pbuf);
						sprintf(pbuf,"         eod = %d",eod);
						puts0(pbuf);
						sprintf(pbuf,"   lcl midnt = %d",local_midnight);
						puts0(pbuf);
						sprintf(pbuf,"        xrtc = %02x:%02x:%02x",*(xbuf+3),*(xbuf+2),*(xbuf+1));
						puts0(pbuf);
					}
				}
			}
		}
	}
	// state machine to detect PBSW press/release actions
#define	MSG_SHORT	2500
#define MSG_LONG	3500
	if((msg_timer == 0) || (msg_state > 18)){
		pb = ~GPIO_PORTF_DATA_R & (SW1 | SW2);						// capture sw state
		switch(pbsw_state){
		default:
		case PBSW_INIT:
			pbsw_state = PBSW_00;
			pbsw_state_last = PBSW_00;
			pbsw_action = 0;
			pbsw_timer = 0;
			break;

		case PBSW_00_DB:											// debounce wait
			if(pbsw_timer == 0){
				pbsw_state = PBSW_00;
				pbsw_state_last = PBSW_00;
			}
			break;

		case PBSW_00:												// both buttons idle
			if(pb){
				switch(pb){
				case (SW1 | SW2):									// both pressed
						pbsw_timer = PBSW_DBOUNCE;
						pbsw_state = PBSW_11_DB;
	//					pbsw_action = PBSW_11_PRS;
						break;

				case (SW1):
						pbsw_timer = PBSW_DBOUNCE;					//sw1 pressed
						pbsw_state = PBSW_01_DB;
	//					pbsw_action = PBSW_01_PRS;
						break;

				case (SW2):
						pbsw_timer = PBSW_DBOUNCE;					//sw2 pressed
						pbsw_state = PBSW_10_DB;
	//					pbsw_action = PBSW_10_PRS;
						break;

				case NOSW:
					break;
				}
			}
			break;

		case PBSW_01_DB:											// debounce wait
			if(pbsw_timer == 0){
				if(pb == SW1){
					pbsw_state = PBSW_01;
					pbsw_state_last = PBSW_01;
					pbsw_action = PBSW_01_PRS;
				}else{
					pbsw_state = pbsw_state_last;
				}
			}
			break;

		case PBSW_01:												// both buttons selected
			switch(pb){
			case (SW1 | SW2):
				pbsw_timer = PBSW_DBOUNCE;
				pbsw_state = PBSW_11_DB;
				pbsw_action = PBSW_11_PRS;
				break;

	/*			case (SW1):
				pbsw_timer = PBSW_DBOUNCE;
				pbsw_state = PBSW_01_DB;
				pbsw_action = PBSW_10_REL;
				break;*/

			case (SW2):
				pbsw_timer = PBSW_DBOUNCE;
				pbsw_state = PBSW_10_DB;
				pbsw_action = PBSW_10_REL;
				break;

			case NOSW:
				pbsw_timer = PBSW_DBOUNCE;
				pbsw_state = PBSW_00_DB;
				pbsw_action = PBSW_01_REL;
				break;
			}
			break;

		case PBSW_10_DB:											// debounce wait
			if(pbsw_timer == 0){
				if(pb == SW2){
					pbsw_state = PBSW_10;
					pbsw_state_last = PBSW_10;
					pbsw_action = PBSW_10_PRS;
				}else{
					pbsw_state = pbsw_state_last;
				}
			}
			break;

		case PBSW_10:												// both buttons selected
			switch(pb){
			case (SW1 | SW2):
				pbsw_timer = PBSW_DBOUNCE;
				pbsw_state = PBSW_11_DB;
				pbsw_action = PBSW_11_PRS;
				break;

				case (SW1):
				pbsw_timer = PBSW_DBOUNCE;
				pbsw_state = PBSW_01_DB;
				pbsw_action = PBSW_10_REL;
				break;

	/*		case (SW2):
				pbsw_timer = PBSW_DBOUNCE;
				pbsw_state = PBSW_10_DB;
				pbsw_action = PBSW_10_REL;
				break;*/

			case NOSW:
				pbsw_timer = PBSW_DBOUNCE;
				pbsw_state = PBSW_00_DB;
				pbsw_action = PBSW_10_REL;
				break;
			}
			break;

		case PBSW_11_DB:											// debounce wait
			if(pbsw_timer == 0){
				if(pb == SW1 | SW2){
					pbsw_state = PBSW_11;
					pbsw_state_last = PBSW_11;
					pbsw_action = PBSW_11_PRS;
				}else{
					pbsw_state = pbsw_state_last;
				}
			}
			break;

		case PBSW_11:												// both buttons selected
			switch(pb){
	/*		case (SW1 | SW2):
				pbsw_timer = PBSW_DBOUNCE;
				pbsw_state = PBSW_11_DB;
				pbsw_action = PBSW_11_DB;
				break;*/

			case (SW1):
				pbsw_timer = PBSW_DBOUNCE;
				pbsw_state = PBSW_01_DB;
				pbsw_action = PBSW_10_REL;
				break;

			case (SW2):
				pbsw_timer = PBSW_DBOUNCE;
				pbsw_state = PBSW_10_DB;
				pbsw_action = PBSW_01_REL;
				break;

			case NOSW:
				pbsw_timer = PBSW_DBOUNCE;
				pbsw_state = PBSW_00_DB;
				pbsw_action = PBSW_11_REL;
				break;
			}
			break;
		}
	}
	// process pbsw_action
	gsptr = get_gstat_ptr();										// get pointer to status
	if((pbsw_action) && (pbsw_timer == 0)){
		switch(pbsw_action){
		default:
			sprintf(pbuf,"pbsw: %u",pbsw_action);
			puts0(pbuf);
			break;

		case PBSW_11_PRS:											// both sw pressed (re-sync GPS
			if(gpsvalid_timer){
				msg_state = 18;
				msg_timer = MSG_SHORT;								// disp hold duration
				set_led_msg(1, "RSYN", 0);
			}
			break;

		case PBSW_10_REL:											// SW2 release: GPS status
			if(msg_state > 20){
				if((msg_state == 26) || (msg_state == 27)){
					tzone += 1;
					if(tzone > 12){
						tzone = -11;
					}
					set_lcl_midnight();								// update local midnight reg
					msg_state = 26;
					msg_timer = 0;
				}
			}else{
				if(gpsvalid_timer){
					msg_state = 11;									// this msg loop runs on thread starting at state "11"
					msg_timer = MSG_LONG;
					switch(gsptr[gpstat]){							// display GPS status message
					default:
						set_led_msg(1, "----", 0);
						break;

					case 0:
						set_led_msg(1, "FIXS", 0);
						break;

					case 1:
						set_led_msg(1, "NOTI", 0);
						break;

					case 3:
						set_led_msg(1, "PDOP", 0);
						break;

					case 8:
						set_led_msg(1, "0SAT", 0);
						break;

					case 9:
						set_led_msg(1, "1SAT", 0);
						break;

					case 10:
						set_led_msg(1, "2SAT", 0);
						break;

					case 11:
						set_led_msg(1, "3SAT", 0);
						break;

					case 12:
						set_led_msg(1, "0USE", 0);
						break;

					case 16:
						set_led_msg(1, "FREJ", 0);
						break;
					}
				}else{
					set_led_msg(1, "GERR", 0);
					msg_state = 1;
					msg_timer = MSG_LONG;
				}
			}
			break;

		case PBSW_01_REL:											// SW1 release: DST modes, auto, or manual.  if manual, allow toggle of DST reg
			if((msg_state > 17) && (msg_state < 21)){
				force_resync();
//				rtc = -100000L;
				msg_state = 19;
				msg_timer = 1100;									// short delay to let system capture resync condition
			}else{
				switch(msg_state){
				default:
					if(dst_auto){									// entry into the dst loop
						set_led_msg(1, "AUTO", 0);
						msg_timer = MSG_LONG;
						msg_state = 21;
					}else{
						set_led_msg(1, "MANU", 0);
						msg_timer = MSG_LONG;
						msg_state = 22;
					}
					break;

				case 21:
					dst_auto = 0;
					set_led_msg(1, "MANU", 0);
					msg_state = 22;
					msg_timer = MSG_SHORT;
					break;

				case 22:
					dst_auto = 1;
					set_led_msg(1, "AUTO", 0);
					msg_state = 21;
					msg_timer = MSG_SHORT;
					break;

				case 23:
					dst = 0;
					msg_state = 24;
					set_led_msg(1, "DST0", 0);
					msg_timer = MSG_SHORT;
					break;

				case 24:
					dst = 1;
					msg_state = 23;
					set_led_msg(1, "DST1", 0);
					msg_timer = MSG_SHORT;
					break;

				case 27:
				case 26:
					tzone -= 1;
					if(tzone < -11){
						tzone = 12;
					}
					set_lcl_midnight();								// set local midnight reg
					msg_timer = 0;
					msg_state = 26;
					break;

				case 29:
					store_eeprom();
					puts0("STOR");
					force_resync();
					msg_timer = 0;
					msg_state = 28;
				}
			}
			break;
		}
		pbsw_action = 0;
	}
	// process msg state
	if(msg_state){
		switch(msg_state){
		default:
		case 1:														// thread #1 kills the message display and returns display to time mode
			if(msg_timer == 0){
				msg_state += 1;
				msg_timer = 500;
				set_led_msg(0, "====", 0);
			}
			break;

		case 2:
			if(msg_timer == 0){
				msg_state = 0;
				pbsw_action = 0;
			}
			break;

		case 11:													// thread #11 is the continuation of the GPS status display
			if(msg_timer == 0){
				msg_state += 1;
				if(gsptr[minoralm1] & 0x20){						// if survey in progress, display prog %
					msg_timer = MSG_LONG;
					sprintf(pbuf,"%03d/",gsptr[survprog]);
					set_led_msg(1, pbuf, 0);
				}
			}
			break;

		case 12:													// display GPS board temp
			if(msg_timer == 0){
				msg_state += 1;										// go to next item
				msg_timer = MSG_LONG;
				get_gtemp(pbuf);
				pbuf[3] = ':';										// degrees symbol
				set_led_msg(1, pbuf, 1);
			}
			break;

		case 13:													// display GPS board temp
			if(msg_timer == 0){
				msg_state = 1;										// go to thread 1 to end
				msg_timer = MSG_LONG;
				//	TJ = 147.5 - (75 * ((rawADC * 3.3 / 4096)))
				stemp32 = 1475L - (75L * ((S32)(adc_buf[7] & 0xfff)) * 33L / 4096L);
				sprintf(pbuf,"%03d ",stemp32);
				pbuf[3] = ';';										// barred-degrees symbol
				set_led_msg(1, pbuf, 1);
			}
			break;

		case 18:
			if(msg_timer == 0){
				msg_state = 19;
				sprintf(pbuf," %03d",resync);
				set_led_msg(1, pbuf, 0);
				msg_timer = MSG_SHORT;
			}
			break;

		case 19:
			if(msg_timer == 0){
				msg_state = 1;
				sprintf(pbuf," %03d",resync);
				set_led_msg(1, pbuf, 0);
				msg_timer = MSG_LONG;
			}
			break;

		case 22:
		case 21:
			if(msg_timer == 0){
				if(dst){
					set_led_msg(1, "DST1", 0);
					msg_state = 23;
					msg_timer = MSG_LONG;
				}else{
					set_led_msg(1, "DST0", 0);
					msg_state = 24;
					msg_timer = MSG_SHORT;
				}
			}
			break;

		case 24:
		case 23:
			if(msg_timer == 0){
				if(dst){
					set_led_msg(1, "DST1", 0);
					msg_state = 25;
					msg_timer = MSG_LONG;
				}else{
					set_led_msg(1, "DST0", 0);
					msg_state = 25;
					msg_timer = MSG_SHORT;
				}
			}
			break;

		case 25:
			if(msg_timer == 0){
				set_led_msg(1, "TZON", 0);
				msg_state = 26;
				msg_timer = 1500;
			}
			break;

		case 26:
			if(msg_timer == 0){
				msg_state = 27;
				if(tzone < 0){
					sprintf(pbuf," %d",tzone);
				}else{
					sprintf(pbuf,"  %d",tzone);
				}
				set_led_msg(1, pbuf, 0);
				msg_timer = MSG_LONG;
			}
			break;

		case 27:
			if(msg_timer == 0){
				msg_state = 29;
				set_led_msg(1, "SAV?", 0);
				msg_timer = MSG_SHORT;
			}
			break;

		case 28:
			if(msg_timer == 0){
				msg_state = 1;
				set_led_msg(1, "SAVD", 0);
				msg_timer = MSG_SHORT;
			}

		case 29:
			if(msg_timer == 0){
				msg_state = 1;
				isdst_auto();										// update dst status
			}
			break;
		}
	}
	// process auto bright control
	if(brt_timer == 0){
		brt_timer = 8;												// ~~125 Hz sample rate to avoid aliasing 60Hz
		adc_in(adc_buf);											// get ADC values

		brt_buf[brt_idx++] = (adc_buf[1] + adc_buf[5]);				// grab both readings and store to averaging buffer
		if(brt_idx >= MAX_AVE_BUF){
			brt_idx = 0;
			ii = 0;
			for(i=0; i<MAX_AVE_BUF; i++){
				ii += brt_buf[i];
			}
			bb = (U16)(ii / (2 * MAX_AVE_BUF));						// calc average
// debug patch
//			sprintf(pbuf,"adc = %u\n",bb);
//			puts0(pbuf);
//////////////
			switch(brt_state){										// process auto-bright state machine
			default:
			case BRT_INIT:
				PWM1_1_CMPB_R = MAX_BRT;							// max brt LED
				brt_state = BRT_HI;
				set_sbright('H');
//				sec_brt(BRT_HI);
				break;

			case BRT_LOW:
				if(bb > DIMU){
					PWM1_1_CMPB_R = MAX_BRT;						// max brt LED
					set_sbright('H');
					sec_brt(BRT_HI);
					brt_state = BRT_HI;
				}
				break;

			case BRT_HI:
				if(bb < BRTL){
					PWM1_1_CMPB_R = MIN_BRT;						// min brt LED
					set_sbright('L');
					sec_brt(BRT_LOW);
					brt_state = BRT_LOW;
				}
				break;
			}
		}
	}
	// process AUX DU updates
	i = GPIO_PORTF_DATA_R & SW3;									// capture SW3 GPIO
	if(i != sw3state){												// look for a change in SW3 status
		sw3state = i;												// update SW3 state
		if(!sw3state){												// if SW3 grounded, change modes
			dispmode += 1;
			if (dispmode >= DISP_MAX){
				dispmode = 0;										// rollover at last mode
			}
			auxdu_title_timer = AUX_TITLE_TIME;
			clear_auxdu();
			auxupd_flag = 1;
//			sprintf(pbuf,"auxmode: %u",dispmode); // debug patch
//			puts0(pbuf);
		}
		wait(10);													// 10ms to debounce
	}
	if(auxupd_flag){												// update AUX DU 1 time per sec
		auxupd_flag = 0;
		switch(dispmode){
		default:
			dispmode = 0;
			break;

		case GPS_DATE:												// display date/time from GPS
			if(auxdu_title_timer){
				parse_fmain("GPS TIME  ");
			}else{
				set_sbright('j');
				set_sbright('S');
				if(get_tsip_valid()){
					sprintf((char*)abuf,"  %02u%02u%02u",gtptr[ghour],gtptr[gmin],gtptr[gsec]);
					set_sbright('I');
					parse_fsub(abuf);
					if((gtptr[gmonth] == 0) || (gtptr[gmonth] > 12)){
						sprintf((char*)abuf,"ERR=--=---");
					}else{
						i = gtptr[gmonth];
						k = gtptr[gday];
						bb = get_gyear();
						j = dow_calc(k, i, bb);
						sprintf((char*)abuf,"%s=%02u=%s",mon_str(gtptr[gmonth] | 0x80),gtptr[gday],dow_str(j));
						}
					set_sbright('a');
					parse_fmain(abuf);
					sprintf((char*)abuf,"%04u",bb);
					parse_fpl(abuf);
				}else{
					parse_fsub("  ------");
					parse_fmain("---=--=---");
					parse_fpl("----");
				}
			}
			break;

		case XUTC_DATE:												// display date/time from xRTC
			if(auxdu_title_timer){
				parse_fmain("RTC TIME  ");
			}else{
				sprintf((char*)abuf,"  %02x%02x%02x",*(xbuf+3),*(xbuf+2),*(xbuf+1));
				set_sbright('I');
				parse_fsub(abuf);

				i = (((*(xbuf+6) & 0x10) >> 4) * 10) + (*(xbuf+6) & 0x0f);
				k = (((*(xbuf+4) & 0x30) >> 4) * 10) + (*(xbuf+4) & 0x0f);
				bb = ((*(xbuf+8) >> 4) * 1000) + ((*(xbuf+8) & 0x0f) * 100 ) + ((*(xbuf+7) >> 4) * 10) + (*(xbuf+7) & 0x0f);
				j = dow_calc(k, i, bb);
				sprintf((char*)abuf,"%s=%02x=%s",mon_str(*(xbuf+6)),*(xbuf+4),dow_str(j));
				set_sbright('a');
				parse_fmain(abuf);

				sprintf((char*)abuf,"%02x%02x",*(xbuf+8),*(xbuf+7));
				set_sbright('j');
				parse_fpl(abuf);
				set_sbright('s');
			}
			break;

		case LCL_DATE:												// display date/time from internal registers
			if(auxdu_title_timer){
				parse_fmain("LOCAL TIME");
			}else{
				set_sbright('I');
				read_rtc = 1;
				while(read_rtc);
				jj = rrtc;												// get minutes since midnight (S.M.)
				ii = jj/SEC_MIN_1024;									// ii = min S.M.
				si = (signed char)(ii/60);								// si = hrs S.M.
				j = (U8)(ii - (si*60));									// j = mins S.H.
				si += (signed char)get_timez();							// apply time zone correction
				si += (signed char)dst;									// apply dst factor
				if(si < 0){												// rollover hours
					si += 24;
				}
				if(si >= 24){
					si -= 24;
				}
				k = (U8)((jj - (ii * SEC_MIN_1024))/MS_COUNT_1024);
	/*			si = (S8)((((*(xbuf+3) & 0x30) >> 4) * 10) + (*(xbuf+3) & 0x0f)) + (S8)tzone;
				if(si >23){
					si -= 24;
				}
				if(si < 0){
					si += 24;
				}*/
				sprintf((char*)abuf,"  %02u%02u%02u",si,j,k);
				parse_fsub(abuf);

				i = get_smon();
				k = get_sday();
				bb = get_syear();
				j = dow_calc(k, i, bb);
				sprintf((char*)abuf,"%s %02u %s",mon_str(get_smon() | 0x80),get_sday(),dow_str(j));
				set_sbright('a');
				parse_fmain(abuf);

				sprintf((char*)abuf,"%04u",bb);
				set_sbright('j');
				parse_fpl(abuf);
				set_sbright('s');
			}
			break;

		case GPS_STAT:												// display GPS status info
			if(auxdu_title_timer){
				parse_fmain("GPS STATUS");
			}else{
				set_sbright('i');
				set_sbright('S');
				if(get_tsip_valid()){
					if(gsptr[minoralm0] & (ANT_OPEN | ANT_SHORT | SURVEY_INPROG)){
						if(gsptr[minoralm0] & SURVEY_INPROG){
							sprintf((char*)abuf,"SURV   %03u",gsptr[survprog]);
							parse_fmain(abuf);
						}else{
							if(gsptr[minoralm0] & ANT_OPEN){
								parse_fmain("   OPENANT");
							}else{
								parse_fmain("  SHORTANT");
							}
						}
					}else{
						switch(gsptr[gpstat]){							// display GPS status message
						default:
							parse_fmain("----------");
							break;

						case 0:
							get_gtemp(tbuf);							// get GPS temperature
							tbuf[3] = ':';								// degrees symbol
							sprintf(pbuf,"  %s   ",tbuf);
							parse_fmain((unsigned char*)pbuf);
							set_sbright('A');
	//						parse_fmain("FIXS      ");
							break;

						case 1:
							parse_fmain("NO GPST   ");
							break;

						case 3:
							parse_fmain("PDOP2HI   ");
							break;

						case 8:
							parse_fmain("0 SAT     ");
							break;

						case 9:
							parse_fmain("1 SAT     ");
							break;

						case 10:
							parse_fmain("2 SAT     ");
							break;

						case 11:
							parse_fmain("3 SAT     ");
							break;

						case 12:
							parse_fmain("0 USE     ");
							break;

						case 16:
							parse_fmain("TRAMREJ   ");
							break;
						}
					}
					sprintf((char*)abuf,"  %06u",get_tow());
					parse_fsub(abuf);
					sprintf((char*)abuf,"%04u",get_wknum());
					parse_fpl(abuf);
				}else{
					parse_fmain("----------");						// placekeeper
					parse_fpl("----");
					parse_fsub("  ------");
				}
			}
			break;

		case CLOCK_VERS:										// display SW version
			if(auxdu_title_timer){
				parse_fmain("SW VERSION");
			}else{
				set_sbright('a');
//				parse_fmain("GPS AUX002");ledSWvers(void)
				parse_fmain(ledSWvers());
				parse_fpl("LEDU");
				parse_fsub("  GPSAUX");
				set_sbright('s');
			}
			break;

		case AUX_BLANK:											// Blank out the aux DU
			clear_auxdu();
			break;
		}
	}
	// process LED segment update
	if(led_msg){
		process_LEDMSG();
	}else{
		// update led segment registers once per min (or if updt force) if not in msg mode
		if(minflag || updt){
			minflag = 0;
			read_rtc = 1;
			while(read_rtc);
			jj = rrtc;												// get minutes since midnight (S.M.)
			ii = jj/SEC_MIN_1024;									// ii = min S.M.
			si = (signed char)(ii/60);								// si = hrs S.M.
			j = (U8)(ii - (si*60));									// j = mins S.H.
			si += (signed char)get_timez();							// apply time zone correction
			si += (signed char)dst;									// apply dst factor
			if(si < 0){												// rollover hours
				si += 24;
			}
			if(si >= 24){
				si -= 24;
			}
			sprintf(tbuf,"%02d%02d",si,j);
			for(j=0; j<4; j++){
				led_string[j] = (led_string[j] & 0x80) | set_7seg(tbuf[j]); // preserve colon status
	//			led_string[j] = set_7seg(tbuf[j]);					// clear colon status
			}
/*			if(!updt){
				set_sec_tic(0);										// restart sec hand
			}*/
		}


/*		read_rtc = 1;
		while(read_rtc);
		jj = rrtc;													// get minutes since midnight
		ii = jj/60000;
		if((jj%60000 == 0) || updt){
			minflag = 0;
			si = (signed char)(ii/60);								// si = hrs
			j = (U8)(ii - (si*60));									// j = mins
			si += (signed char)get_timez();							// apply time zone correction
			si += (signed char)dst;									// apply dst factor
			if(si < 0){												// rollover hours
				si += 24;
			}
			if(si >= 24){
				si -= 24;
			}
			sprintf(tbuf,"%02d%02d",si,j);
			for(j=0; j<4; j++){
				led_string[j] = (led_string[j] & 0x80) | set_7seg(tbuf[j]); // preserve colon status
	//			led_string[j] = set_7seg(tbuf[j]);					// clear colon status
			}
		}*/
	}
	return;
}
//-----------------------------------------------------------------------------
// process_LEDMSG() handles LED message loops
//-----------------------------------------------------------------------------
void process_LEDMSG(void){

	return;
}
//-----------------------------------------------------------------------------
// set_7seg() does an ASCII look up of the segment map and returns the segment code
//
//             a		a = bit 0 ... g = bit 6 (bit 7 is dp or one of the colon segments)
//           ----
//		f  /    / b			           a
//         ---- g		ex.:         ----		ASCII "?" segment code = 01010011 = 0x53
//     e /    / c				        / b
//       ----			           ---- g
//        d					  e  /
//
//	Table must be continuous as the start character is subtracted from input ASCII
//	to determine the offset index from which to read the code.  ASCII outside the
//	table bounds are set to blank (no segments activated).
//-----------------------------------------------------------------------------
U8 set_7seg(char c){
const U8 seg_lut[] = {
//					   -     .     /
					   0x40, 0x80, 0x52,
//                     0     1     2     3     4     5     6     7     8     9
                       0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f,
// ":" is the degree symbol, ";" is the degree symbol with underscore
//					   :     ;     <     =     >     ?     @
				       0x63, 0x6b, 0x18, 0x48, 0x0c, 0x53, 0x5f,
//                     A     B     C     D     E     F     G     H     I
				       0x77, 0x7c, 0x58, 0x5e, 0x79, 0x71, 0x3b, 0x74, 0x04,
//                     J     K     L     M     N     O     P     Q     R
				       0x1e, 0x76, 0x38, 0x55, 0x54, 0x5c, 0x73, 0x67, 0x50,
//                     S     T     U     V     W     X     Y     Z
				       0x6d, 0x78, 0x3e, 0x1c, 0x1d, 0x76, 0x6e, 0x5b
			         };
	U8	oc = 0x00;

	if((c >= '-') && (c <= 'Z')){
		oc = seg_lut[c - '-'];
	}
	return oc;
}

//-----------------------------------------------------------------------------
// get_imin() returns Imin value in float units of mA
//	Converts ADC reading to Rsense current in mA
//	Assumes that the ADC is set with the PGA at max gain (Vfull-scale = +/-0.256V)
//	and Rsense = 0.025 ohms: I = 1000 * V/R gives mA
//	Vadc = (ADC max V) * (ADC raw value (k)) / (ADC maximum value (for this ADC, 32768))
//	If flag == TRUE, reset the lower value to it's starting point
//-----------------------------------------------------------------------------
float get_imin(U8 flag){
	float f;		// float temp

	f = 256.0 * (float)klower/32768.0;		// calculate ADC voltage (mV)
	f = f/0.025;							// calculate current (mA)
	if(flag) klower = ADC_MAX;				// if flag, reset klower
	return f;
}

//-----------------------------------------------------------------------------
// get_imax() returns Imax value in float units of mA
//	Converts ADC reading to Rsense current in mA
//	Assumes that the ADC is set with the PGA at max gain (Vfull-scale = +/-0.256V)
//	and Rsense = 0.025 ohms: I = 1000 * V/R gives mA
//	Vadc = (ADC max V) * (ADC raw value (k)) / (ADC maximum value (for this ADC, 32768))
//	If flag == TRUE, reset the upper value to it's starting point
//-----------------------------------------------------------------------------
float get_imax(U8 flag){
	float f;		// float temp

	f = 256.0 * (float)kupper/32768.0;		// calculate ADC voltage (mV)
	f = f/0.025;							// calculate current (mA)
	if(flag) kupper = 0;					// if flag, reset kupper
	return f;
}

//-----------------------------------------------------------------------------
// waitpio() uses a dedicated ms timer to establish a defined delay (+/- 1LSB latency)
//	loops through process_IO during wait period.
//-----------------------------------------------------------------------------
void waitpio(U16 waitms){
//	U32	i;

//	i = 545 * (U32)waitms;
    waittimer = waitms;
//    for(;i!=0;i--);		// patch
    while(waittimer != 0) process_IO(0,0);		// wait for the waittimer, run process_IO() in the mean time
}

//-----------------------------------------------------------------------------
// wait() uses a dedicated ms timer to establish a defined delay (+/- 1LSB latency)
//	waittimer is a 1ms count-down from "N" and halt timer that is processed by
//	the timer 2 interrupt.
//-----------------------------------------------------------------------------
void wait(U16 waitms)
{
//	U32	i;

//	i = 545 * (U32)waitms;
    waittimer = waitms;
//    for(;i!=0;i--);		// patch
    while(waittimer != 0);
}

//-----------------------------------------------------------------------------
// wait2() does quick delay pace.  This loop is affected by the SYSCLK setting
//	and will thus change as SYSCLK is changed.  !!! PLATFORM DEPENDENT !!!
//-----------------------------------------------------------------------------
void wait2(U16 waitms)
{
	U32	i;

	i = 10 * (U32)waitms;
    waittimer = waitms;
    for(;i!=0;i--);		// patch
//    while(waittimer != 0);
}

//-----------------------------------------------------------------------------
// wait_reg0() waits for (delay timer == 0) or (regptr* & clrmask == 0)
//	if delay expires, return TRUE, else return FALSE
//	allows calling function to poll a register bit(s) with a defined timeout
//	this prevents the system from locking up if the desired bit behavior does
//	not occur.  Returns TRUE if bit(s) fails to transition to 0
//-----------------------------------------------------------------------------
U8 wait_reg0(volatile uint32_t *regptr, uint32_t clrmask, U16 delay){
	U8 timout = FALSE;

    waittimer = delay;
    while((waittimer) && ((*regptr & clrmask) != 0));
    if(waittimer == 0) timout = TRUE;
    return timout;
}

//-----------------------------------------------------------------------------
// wait_reg1() waits for (delay timer == 0) or (regptr* & setmask == setmask)
//	if delay expires, return TRUE, else return FALSE
//	allows calling function to poll a register bit(s) with a defined timeout
//	this prevents the system from locking up if the desired bit behavior does
//	not occur.  Returns TRUE if bit(s) fails to transition to 1
//-----------------------------------------------------------------------------
U8 wait_reg1(volatile uint32_t *regptr, uint32_t setmask, U16 delay){
	U8 timout = FALSE;

    waittimer = delay;
    while((waittimer) && ((*regptr & setmask) != setmask));
    if(waittimer == 0) timout = TRUE;
    return timout;
}

//-----------------------------------------------------------------------------
// getipl() returns current ipl flags value
//-----------------------------------------------------------------------------
U16 getipl(void){

	return ipl;
}

//-----------------------------------------------------------------------------
// led_rgb() sets PWM registers
//-----------------------------------------------------------------------------
void led_rgb(S16 ledred, S16 ledblu, S16 ledgrn){

	PWM1_2_CMPB_R = ledred;						// set PWM compare regsiters (sets duty cycle)
	PWM1_3_CMPA_R = ledblu;
	PWM1_3_CMPB_R = ledgrn;
	return;
}

//-----------------------------------------------------------------------------
// set_timez() sets timezone
//-----------------------------------------------------------------------------
void set_timez(S16 zone){

	tzone = zone;
	set_lcl_midnight();
//	local_midnight = (S32)tzone * -3600000L;		// calc lcl midnight reg
	eewr(EE_TIME_ZONE, (U32)zone);
	force_resync();
}

//-----------------------------------------------------------------------------
// get_timez() gets timezone
//-----------------------------------------------------------------------------
S16 get_timez(void){

	return (S16)tzone;
}

//-----------------------------------------------------------------------------
// get_rtc() gets rtc reg
//-----------------------------------------------------------------------------
S32 get_rtc(void){
	read_rtc = 1;
	while(read_rtc);
	return rrtc;
}

//-----------------------------------------------------------------------------
// set_rtc() sets rtc reg and syncs other time-involved regs.
//-----------------------------------------------------------------------------
void set_rtc(S32 time){

	rrtc = time;
	write_rtc = 1;
	while(write_rtc);
	isdst_auto();									// auto dst update if enabled
	iplt2 = 1;
	return;
}

//-----------------------------------------------------------------------------
// set_cal() sets rtc_scalp reg
//-----------------------------------------------------------------------------
void set_cal(S32 scal){

	rtc_scalp = scal;
//	eod = END_OF_DAY_1024;
	eewr(EE_SCALP, rtc_scalp);
	return;
}

//-----------------------------------------------------------------------------
// read_cal() returns rtc_scalp reg
//-----------------------------------------------------------------------------
S32 read_cal(void){

	return rtc_scalp;
}

//-----------------------------------------------------------------------------
// set_tdebug() sets tdebug reg
//-----------------------------------------------------------------------------
void set_tdebug(void){

	tdebug = TRUE;
	return;
}

//-----------------------------------------------------------------------------
// set_tsip_timer() sets tsip timer
//-----------------------------------------------------------------------------
void set_tsip_timer(U16 tval){

	tsip_timer = tval;
	return;
}

//-----------------------------------------------------------------------------
// get_tsip_timer() sets tsip timer
//-----------------------------------------------------------------------------
U16 get_tsip_timer(void){

	return tsip_timer;
}

//-----------------------------------------------------------------------------
// get_resync() returns resync value
//-----------------------------------------------------------------------------
U16 get_resync(void){

	return resync;
}

//-----------------------------------------------------------------------------
// get_adcbuf() gets adc value at idx
//-----------------------------------------------------------------------------
U16 get_adcbuf(U8 idx){

	return adc_buf[idx];
}

//-----------------------------------------------------------------------------
// store_eeprom() stores EEPROM params
//-----------------------------------------------------------------------------
void store_eeprom(void){

	eewr(EE_TIME_ZONE, ((U32)tzone & 0xffff) | (((U32)dst_auto << 16) & 0x00ff0000) | (((U32)dst << 24) & 0xff000000));
	eewr(EE_SCALP, rtc_scalp);
	eewr(EE_MINBRT, (U32)minbrt | ((U32)maxbrt << 16));
	return;
}

//-----------------------------------------------------------------------------
// init_eeprom() inits EEPROM values to factory defaults
//-----------------------------------------------------------------------------
void init_eeprom(void){

	tzone = 0;
	set_lcl_midnight();
//	local_midnight = 0L;						// init lcl midnight reg
	dst_auto = 0;
	dst = 0;
	rtc_scalp = 0;
//	eod = END_OF_DAY_1024 + rtc_scalp;
	minbrt = 0;
	maxbrt = 4096;
	store_eeprom();
	return;
}

//-----------------------------------------------------------------------------
// set_led_msg() overrides clock and xfrs custom message to 7seg buffer
//-----------------------------------------------------------------------------
void set_led_msg(U8 msgflag, char* ptr, U8 dp){
	U8	j;

	led_msg = msgflag;
	for(j=0; j<4; j++){
		led_string[j] = set_7seg(ptr[j]);
	}
	if(dp){
//		led_string[0] |= 0x80;
		led_string[1] |= 0x80;
	}
	return;
}

//-----------------------------------------------------------------------------
// set_ppsflag() sets the ppsflag to trigger a GPS pps align
//-----------------------------------------------------------------------------
void set_ppsflag(void){

	ppsflag = 1;
	auxupd_flag = 1;
	return;
}

//-----------------------------------------------------------------------------
// isdst_auto() updates dst reg if auto_dst is enabled
//-----------------------------------------------------------------------------
void isdst_auto(void){

	if(dst_auto){
		if(isdst()){
			dst = 1;
		}else{
			dst = 0;
		}
	}
	//debug patch
	set_tdebug(); // trigger debug display
	puts0("isdst_auto");
	//
	return;
}

//-----------------------------------------------------------------------------
// set_lcl_midnight() updates lcl midnight reg
//	lcl midnight is the match register for rtc to determine when to advance the
//	shadow calendar
//-----------------------------------------------------------------------------
void set_lcl_midnight(void){

	if(tzone < 0){
		local_midnight = (S32)tzone * -CYC_HOUR_1024;
	}else{
		if(tzone == 0){
			local_midnight = 0;
		}else{
			local_midnight = END_OF_DAY_1024 - ((S32)tzone * CYC_HOUR_1024);
		}
	}
}

//-----------------------------------------------------------------------------
// force_resync() sets rtc = -100000 to force a GPS resync
//-----------------------------------------------------------------------------
void force_resync(void){

	rrtc = -100000000L;				// non-sequitor time value
	write_rtc = 1;					// enable write
	while(write_rtc);				// hold here until processed
	return;
}

//-----------------------------------------------------------------------------
// sec_brt() controls brightness of sec-ring by adjusting the sec_ring[]
//		array (shifting left or right).
//-----------------------------------------------------------------------------
//

void sec_brt(U8 brite){
		U8	i;				// temp
static	U8 brt_status;		// brt status memory

	if(brite == 0xff){
		sec_ring[0] = 0x0;
		sec_ring[1] = 0x2000 >> 1;
		sec_ring[2] = 0x200000 >> 1;
		sec_ring[3] = 0x202800 >> 1;
		sec_ring[4] = 0x20 >> 1;
		sec_ring[5] = 0x2020 >> 1;
		sec_ring[6] = 0x180028 >> 1;
		sec_ring[7] = 0x202020 >> 1;
		brt_status = BRIGHT;
	}else{
		if(brite != brt_status){
			brt_status = brite;
			if(brite){
				for(i=0;i<8;i++){
					sec_ring[i] <<= 2;
				}
				for(i=0;i<NPXL_ARRAY_LEN;i++){
					npxl_array[i] <<= 2;
				}
			}else{
				for(i=0;i<8;i++){
					sec_ring[i] >>= 2;
				}
				for(i=0;i<NPXL_ARRAY_LEN;i++){
					npxl_array[i] >>= 2;
				}
			}
			send_npxl();
		}
	}
}

//-----------------------------------------------------------------------------
// set_sec_tic() initializes sec ring to match current second of minute
//	secs is a U16 that indicates #ms in current minute
//-----------------------------------------------------------------------------
//
// sets ring LED per the current secs value.  Call this any time the seconds
//	register is updated (other than the normal advance to the next second).

void set_sec_tic(U16 secs){
	U8	i;		// temps
	U8	j;
	U8	k;
	U16	ii;
	U16	jj = secs;

	if(secs > 512){
		jj = secs - 512;
	}
	ii = jj/SEC_TIC_RATE;
	j = ii/16;
	i = ii%16;
	sec_prscl = SEC_TIC_RATE - (jj%SEC_TIC_RATE);
	for(k=0;k<=i;k++){
		npxl_array[k] = sec_ring[j&0x07];
	}
	sec_idx = (j & 0x07);
	j -= 1;
	if(j>7){
		j = 7;
	}
	if(k < NPXL_ARRAY_LEN){
		for(;k<NPXL_ARRAY_LEN;k++){
			npxl_array[k] = sec_ring[j&0x07];
		}
	}
	send_npxl();
	return;
}

//-----------------------------------------------------------------------------
// sec_tic() tics off color wheel once LED per sec
//-----------------------------------------------------------------------------
//
// Advance LED to next tic state

void sec_tic(void){
	U8	i;		// temps
	U8	j;

	i = 0;
	j = 1;
	do{
		if(npxl_array[i+1] != npxl_array[i]){
			npxl_array[i+1] = npxl_array[i];
			j = 0;
		}
	}while(j && (++i < (NPXL_ARRAY_LEN - 1)));
	if(i >= (NPXL_ARRAY_LEN - 1)){
		npxl_array[0] = sec_ring[++sec_idx & 0x07];
	}
	send_npxl();
	return;
}

//-----------------------------------------------------------------------------
// send_npxl()
//-----------------------------------------------------------------------------
//
// Sends 16 word (24 bits/wd) neopixel string to PF2 pin via SSI1 interface
//	Disables all interrupts (save state)
//	send all bits in message
//	restore interrupts
//	delay 50us for RES period
//	exit
//
//-----------------------------------------------------------------------------

void send_npxl(void)
{
	U32	nvic_save0 = NVIC_EN0_R;			// save enabled interrupts
	U32	nvic_save1 = NVIC_EN1_R;
	U32	nvic_save2 = NVIC_EN2_R;
	U32	nvic_save3 = NVIC_EN3_R;
	U32	nvic_save4 = NVIC_EN4_R;
	U8	i;											// loop temp

	npxl_flag = 1;									// synch with timer2isr
	while(npxl_flag);
	NVIC_DIS0_R = 0xffffffff;						// disable all interrupts
	NVIC_DIS1_R = 0xffffffff;
	NVIC_DIS2_R = 0xffffffff;
	NVIC_DIS3_R = 0xffffffff;
	NVIC_DIS4_R = 0xffffffff;
//	NVIC_EN0_R = NVIC_EN0_SSI1;						// enable ssi1 isr
	np_idx = NPXL_ARRAY_LEN - 1;
	// send data array
	for(i=0;i<NPXL_ARRAY_LEN;i++){
		npxl_data = npxl_array[np_idx--];
		for(np_mask = 0x00800000; np_mask; np_mask >>= 1){
			while((SSI1_SR_R & SSI_SR_TNF) == 0);	// wile tx fifo full
			if(np_mask & npxl_data){
				SSI1_DR_R = NPXL_1;					// set "1"
			}else{
				SSI1_DR_R = NPXL_0;					// set "0"
			}
		}
	}
	NVIC_EN0_R = nvic_save0;						// restore interrupt enables
	NVIC_EN1_R = nvic_save1;
	NVIC_EN2_R = nvic_save2;
	NVIC_EN3_R = nvic_save3;
	NVIC_EN4_R = nvic_save4;
	// send end of data RES (at least 50us...this step doesn't care if the isr's lengthen the RES time)
	for(i=0;i<41;i++){
		while((SSI1_SR_R & SSI_SR_TNF) == 0);		// wile tx fifo full
		SSI1_DR_R = 0;								// set "0"
	}
	return;
}

//-----------------------------------------------------------------------------
// clear_auxdu()
//-----------------------------------------------------------------------------
//
// blanks out all aux du indicators
//
//-----------------------------------------------------------------------------

void clear_auxdu(void){

	parse_fmain("          ");
	parse_fpl("    ");
	parse_fsub("        ");
	set_sbright('s');
	set_sbright('i');
	set_sbright('a');
}

//-----------------------------------------------------------------------------
// SSR1_ISR
//-----------------------------------------------------------------------------
//
// SSI isr.
//
//-----------------------------------------------------------------------------

void SSI1_ISR(void)
{
	return;
}

//-----------------------------------------------------------------------------
// Timer2_ISR
//-----------------------------------------------------------------------------
//
// Called when timer2 A overflows (NORM mode):
//	intended to update app timers @ 1ms per lsb.  app timers are "count-down-and-
//		halt".  Fns write a time value to a particular timer variable, then poll
//		the variable until it reaches 0.  The counting then stops until the variable
//		is written with another value.  Note that there is a granularity error
//		of 1 ms.  This is due to the fact that all of the timers are updated on
//		the same 1ms cadence.  This means that the actual time value that expires
//		once the timer variable is written is +0/-1 ms from the value that was
//		written.  This is because there is no provision for re-setting the timer2
//		counters.  Fns may "synchronize" by writing a value of 1 to one of the
//		timer variables, then wait for that to reach 0.  At that point, the Fn
//		is synchronized to the timer2 divider chain, and subsequent timer variable
//		interactions are likely to be very close to the full value desired.
//		This is generally not an issue except for timer values that are very near
//		to 1ms (approx 10ms or less) or timing functions that require a great deal
//		of precision (such as time of day, or pulse width measurements.  These
//		applications should use a dedicated timer to provide the desired precision.
//
//-----------------------------------------------------------------------------

void Timer2_ISR(void)
{
	static	U8	digitaddr;
	static	U16	ledrate;				// prescales the ms rate to the 7seg LED update rate
			U8	digit_rdy;				// disply temps
			U8	digit_data;
#define	LEDSCAN			3				// #ms between 7seg digits

//	GPIO_PORTE_DATA_R ^= PORTE_SPR5;		// toggle debug pin
	TIMER2_ICR_R = TIMER2_MIS_R;
	digit_rdy = 0;											// preclear update flag (update spread between start and end of interrupt to allow
															// I/O changes to settle a bit before re-enabling the LED segments)
    if(++ledrate > LEDSCAN){								// sets 7seg led digit scan rate
//    	GPIO_PORTA_DATA_R |= SEG_BLANK;						// blank output
    	GPIO_PORTB_DATA_R = 0;								// blank output
    	ledrate = 0;
    	digitaddr += SEG_ADR0;								// advance digit address
    	if(digitaddr > (SEG_ADR1 | SEG_ADR0)){
    		digitaddr = 0;									// rollover
    	}
    	GPIO_PORTC_DATA_R = (GPIO_PORTC_DATA_R & DADDR_MASK) | digitaddr;	// select next digit
    	digit_rdy = digitaddr >> 6;							// drive next digit
    	digit_data = led_string[digit_rdy];
    	digit_rdy = 1;
//    	GPIO_PORTA_DATA_R = digitaddr | i;					// enable drive
    }
    npxl_flag = 0;											// clear interlock
#if	(NORTC_DEBUG == 1)
    if(write_rtc){											// if enabled, process update to rtc (this is an interlock to prevent sync issues if an INT
    	rtc = rrtc;											// disrupts a write in progress)
    	write_rtc = 0;
    }
	if(++rtc == END_OF_DAY){								// roll-over rtc regs with calibration at the end of each day
		rtc = rtc_scalp;
		min = 0;
		sec = 0;
	}
	if(rtc == local_midnight){								// check for local midnight
		do_nextday();										// calendar is based on local midnight
		// debug patch
		puts0("nxdy");
		set_tdebug(); // trigger debug display
		//
	}
	if((rtc == (local_midnight + ONEHOUR)) || (rtc == (local_midnight + TWOHOUR))){
		isdst_auto();
	}
	if(minflag == 0){										// process minute timer
		if(++sec == 60000){
	//		lrun = 1;
			sec = 0;
			minflag = 1;
			if(++min > 59){									// perform cal at the top of each hour
				min = 0;
				rtc += rtc_scalp;
			}
		}
	}
    if(--sec_prscl == 0){
    	sec_tic_flag = 1;
    	sec_prscl = SEC_TIC_RATE;
    }
    if(read_rtc){											// semaphore controlled rtc capture (prevents disruption by interrupt)
    	rrtc = rtc;
    	read_rtc = 0;
    }
#endif
    if (auxdu_title_timer != 0){					// update app timer
    	auxdu_title_timer--;
    }
    if (tsip_timer != 0){					// update tsip timer
    	tsip_timer--;
    }
    if (procio_timer1ms != 0){				// update procio timer
    	procio_timer1ms--;
    }
    if (waittimer != 0){					// update wait timer
        waittimer--;
    }
    if (xm_timer != 0){						// update xmodem timer
        xm_timer--;
    }
    if(gpsvalid_timer != 0){				// update gps stale timeout
    	gpsvalid_timer--;
    }
    if(gpscal_timer != 0){					// update gps cal timeout
    	gpscal_timer--;
    }
    if(brt_timer != 0){						// update brt delay timeout
    	brt_timer--;
    }
    if(pbsw_timer != 0){					// update pbsw state delay
    	pbsw_timer--;
    }
    if(msg_timer != 0){						// update message state delay
    	msg_timer--;
    }
    if(ipl_timer != 0){						// update message state delay
    	ipl_timer--;
    }
	if(digit_rdy){
    	GPIO_PORTB_DATA_R = digit_data;		// update segments (after a bunch of cycles to settle the drivers)
	}

//	GPIO_PORTE_DATA_R ^= PORTE_SPR5;		// toggle debug pin
}

//-----------------------------------------------------------------------------
// GPIOA ISR
//-----------------------------------------------------------------------------
//
// Called when GPIO A interrupt is called:
//	intended to update the rtc registers.
//
//-----------------------------------------------------------------------------

void rtc_1024_isr(void){
static	U8	colonflag1;				// colon flash flags
static	U8	colonflag2;

	GPIO_PORTA_ICR_R = RTC_CLK;
	if(iplt2){
		iplt2 = 0;
		colonflag1 = 0x80;
		colonflag2 = 0x80;
	}
#if	(NORTC_DEBUG == 0)
	if(!led_msg){											// process colon flash if message mode disabled
		if(gpsvalid_timer){
	    	if(ppsflag){									// if GPS valid, colon flash is aligned to PPS (give or take serial latencies)
	    		ppsflag = 0;
	    		led_string[1] |= colonflag1;
	    		led_string[2] |= colonflag1;
	    		colonrate = 0;
	    	}
	    	if(++colonrate == SEC500MS_1024){					// turn colon off after 500ms (and wait for next pps from GPS)
	    		led_string[1] &= ~colonflag1;
	    		led_string[2] &= ~colonflag1;
	    	}
	    }else{
	    	if(ppsflag){									// if GPS valid, colon flash is aligned to PPS (give or take serial latencies)
	    		ppsflag = 0;
	    		led_string[1] ^= colonflag1;				// alternate colons in non-GPS mode
	    		led_string[2] = (led_string[2] & ~colonflag2) | (~led_string[1] & colonflag2);
	    		colonrate = 0;
	    	}
	    	if(++colonrate > SEC500MS_1024){						// 7seg led colon flash rate = 1sec
	    		colonrate = 0;
	    		led_string[1] ^= colonflag1;				// alternate colons in non-GPS mode
	    		led_string[2] = (led_string[2] & ~colonflag2) | (~led_string[1] & colonflag2);
	    	}
	    }
	}
	if(write_rtc){									// if enabled, process update to rtc (this is an interlock to prevent sync issues if an INT
    	rtc = rrtc;									// disrupts a write in progress)
    	write_rtc = 0;
    }
	if(++rtc >= eod){								// roll-over rtc regs with calibration at the end of each day
		rtc = 0;
	}
	if(rtc == local_midnight){						// check for local midnight
		do_nextday();								// calendar is based on local midnight
		if(++calday == CAL_PERIOD){
			calday = 0;
			eod = END_OF_DAY_1024 + rtc_scalp;
		}else{
			eod = END_OF_DAY_1024;
		}
		// debug patch
		puts0("nxdy");
		set_tdebug(); // trigger debug display
	}
	if((rtc == (local_midnight + ONEHOUR)) || (rtc == (local_midnight + TWOHOUR))){
		isdst_auto();
	}
	if(--sec_prscl == 0){
		sec_tic_flag = 1;
		sec_prscl = SEC_TIC_RATE;
	}
    if(read_rtc){									// semaphore controlled rtc capture (prevents disruption by interrupt)
    	rrtc = rtc;
    	read_rtc = 0;
    }
#endif
	return;
}

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------

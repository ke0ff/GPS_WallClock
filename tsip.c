/****************************************************************************************
 ****************** COPYRIGHT (c) 2018 by Joseph Haas (DBA FF Systems)  *****************
 *
 *  File name: main.c
 *
 *  Module:    Control
 *
 *  Summary:   This is the main code file for the tsip protocol interpreter
 *
 *  Project scope revision history:
 *    10-27-18 jmh:  Rev 0.0:
 *                   copied from GPSlave project
 *
 ***************************************************************************************/

/****************************************************************************************
 *  File scope revision history:
 *  05-10-13 jmh:	creation date
 *
 ***************************************************************************************/

//--------------------------------------------------------------------------------------
// tsip.c
//		For Resolution-T GPS:
//
//      UART: 9600 baud, GPS or CLI
//
//      SYSTEM NOTES:
//
//      TSIP packet capture looks for packet id = 0x8f, subcode = 0xac.  This
//      packet features 69 bytes, of which only a handful are monitored.
//      From minor alarms, ANT_OPEN, ANT_SHORTED, NOT_TRACKING, and ~PPS_GEN
//      will generate a system alarm (sledstat = FASTBLINK for fast blink).
//      If the minor alarms are OK, the next level (sledstat = QUICKBLINK)
//      is set if GPS decode stat != 0.  Lastly, if previous error levels are
//      OK, if self-survey != 100 sledstat = SLOWBLINK.  If all error levels
//      are clear, sledstat = NOBLINK  to turn off status LED which indicates
//      that the GPS is surveyed and timing is assured.
//
//--------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
// compile defines
//#define F360DK ON

#include <stdio.h>
#include <string.h>
#include <math.h>
#define IS_TSIPC
#include "tsip.h"
#include "init.h"
#include "typedef.h"
#include "serial.h"
#include "version.h"
#include "tsip.h"
#include "rtc.h"

//-----------------------------------------------------------------------------
// Definitions
//-----------------------------------------------------------------------------

//  see init.h for #defines

// union to capture single precision float from serial stream
union tsip_float {
    U8  bytes[4];
    F32 f;
};
// union to capture double precision float from serial stream
union tsip_float2 {
    U8  bytes[8];
    F64 f2;
};

enum gps_idx{gsec,gmin,ghour,gmonth,gday,gtref,gpref,guinfo,guoffsH,guoffsL};

//-----------------------------------------------------------------------------
// External Variables
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Main Variables
//-----------------------------------------------------------------------------

// port assignments


//-----------------------------------------------------------------------------
// Local variables
//-----------------------------------------------------------------------------
U8	gtime[10];						// GPS time/date regs
//	gsec, gmin, ghour, gmonth, gday, gtref, gpref, guinfo};
U16	gyear;
U8	gtime_arch[10];					// archive GPS time/date regs from last re-sync
U16	gyear_arch;
S16	utc_offs;						// utc offset (sec)
U8	gotgpstime;
U8	gps_flags;						// GPS status flags
U8	last_day[13];					// last day of month array (index 0 is not used)
const S8 surv_start[] =  {0x00,  DLE, 0x8e, 0xa6, 0x00,  DLE, ETX};
char tsipbuf[TSIPBUFLEN+2];			// tsip input buffer
U8	gps_inhibit;					// inhibit GPS input (debug)
union tsip_float gps_temp;			// unions to capture float
U8	tsip_valid;						// tsip data valid flag
U32	gtow;							// tow
U16	gwknum;							// wk num

U8	gstat[4];

U8	smon;							// shadow calendar
U8	sday;
U16	syear;

//-----------------------------------------------------------------------------
// Local Prototypes
//-----------------------------------------------------------------------------

//******************************************************************************
// tsip_main()
// GPS tsip comms.
//
//******************************************************************************
U8 tsip_main(U8 iplfl){
	char c;									// serial input holding reg
	S16 ci;									// serial input holding reg (retains empty flag)
	static U8  tsip_state;					// tsip input state
	static U8  tsip_ptr;					// tsip buffer pointer
	static U8  numDLE;						// tsip DLE counter
	static U8  tsip_stale;					// 0 if tsip data fresh, count incremente by 1 every time a packet is expected
	U8  i;									// temps
	U8	j;
//	union tsip_float gps_qerr;				// data from GPS
//	union tsip_float2 pps_delay;			// pps cable delay float (double)
	static U8  error_state;					// error SM state
//	static U8  surv_start_len;

	if(iplfl){
		gps_inhibit = 0;						// debug flag
		gstat[survprog] = 0;					// GPS status: survey progress (%)
		gstat[minoralm0] = 0xff;				// GPS status: alarm flags lsby
		gstat[minoralm1] = 0xff;				// GPS status: alarm flags msby
		gstat[gpstat] = 0xff;					// GPS status: status reg
		tsip_state = TSTART;					// tsip input state
		tsip_stale = 1;							// 0 if tsip data fresh, count incremente by 1 every time a packet is expected
		error_state = MAJORERR;					// error SM state
		tsip_valid = FALSE;
//		surv_start_len = sizeof(surv_start);
		gotgpstime = FALSE;
		gtime[gtref] = 'x';
		gtime[gpref] = 'x';
		gtime[guinfo] = 'x';
		set_tsip_timer(0);						// clear tsip timer
		last_day[0] = 0;						// table of last day of month + 1
		last_day[JAN] = LASTJAN + 1;			// for each month of year (index 0 is not used)
		last_day[FEB] = LASTFEB + 1;
		last_day[MAR] = LASTMAR + 1;
		last_day[APR] = LASTAPR + 1;
		last_day[MAY] = LASTMAY + 1;
		last_day[JUN] = LASTJUN + 1;
		last_day[JUL] = LASTJUL + 1;
		last_day[AUG] = LASTAUG + 1;
		last_day[SEP] = LASTSEP + 1;
		last_day[OCT] = LASTOCT + 1;
		last_day[NOV] = LASTNOV + 1;
		last_day[DEC] = LASTDEC + 1;
		for(i=0; i<TSIPBUFLEN; i++){			// preset tsip buffer to all 0xff
			tsipbuf[i] = 0xff;
		}
		tsip_ptr = 0;
		gps_temp.f = 0.0;						// init unions
//		gps_qerr.f = 0.0;
//		pps_delay.f2 = 0.0;
		gtime[gsec] = 60;								// invalidate date/time
		gtime[gmin] = 60;
		gtime[ghour] = 24;
		gtime[gmonth] = 13;
		gtime[gday] = 32;
		gyear = 0;
		// test patch for UART1
		putchar1('t');
		putchar1('e');
		putchar1('s');
		putchar1('t');
		putchar1('\n');
		putchar1('\r');
//		init_serial();							// init serial module
//		timehack_timer = 50;					// pre-set time hack timer to 50 sec to allow gps to init timer
	}
	//
	// process TSIP packet input if GPS<->PC or auto mode selected
	//
	if(get_tsip_timer() == 0){								// if tsip times out, abort msg & restart
		tsip_stale++;										// set stale data
		set_tsip_timer(MS2000);
		tsip_state = TSTART;								// reset message RX logic
		tsip_valid = FALSE;
	}else{
		ci = getchr1();
		if(ci != 0x100){
			c = (char)(ci & 0xff);							// capture input data
			switch(tsip_state){								// process tsip protocol to fill tsip buffer
			case TSTART:
			default:
				if(c == DLE){								// if DLE, init process vars
					tsip_state = TRUN;
					numDLE = 1;								// set DLE count = 1
					tsip_ptr = 0;							// clear buffer
//					gledtimer = LED_BLINK_TIME;
//					step_232sw(GPSLEDON);					// blink GPS LED
				}
				break;

			case TRUN:
				switch(c){
				case DLE:
					if(tsip_ptr == 0){
						tsip_state = TSTART;				// if 1st buffer chr = DLE, restart TSIP SM
					}else{
						if((numDLE++ & 0x01) == 0x01){		// if DLE count is odd, add DLE to buffer
							tsipbuf[tsip_ptr++] = c;
						}
					}
					break;

				case ETX:
					if((numDLE & 0x01) == 0x01){			// if DLE count is odd, message complete,
						tsip_state = TPROCESS;				// process buffer
					}else{
						tsipbuf[tsip_ptr++] = c;			// if DLE count is even, add ETX to buffer
						numDLE = 0;
					}
					break;

				default:
					tsipbuf[tsip_ptr++] = c;				// Add char to buffer
					numDLE = 0;								// reset consecutive DLE counter
					break;
				}
				if(tsip_ptr > TSIPBUFLEN){					// limit tsip buffer
					tsip_ptr = TSIPBUFLEN;					// caps buffer if input message attempts to overflow
				}											// (following message data lost, but end of message still recognized)
				break;
			}
		}
	}
	if(tsip_state == TPROCESS){
		// (TPROCESS is handled outside the switch{} to allow instant recognition of EOM)
		// test for msg ID & extract data:
		//	Buffer holds all TSIP data from <id> to last data byte
		//	(DLE and ETX control codes are not in the buffer).
		//
		//	Secondary timing packet is <id> = 0x8f msg# = 0xac
		//	The presence of the <id> in the buffer causes an offset
		//	of 1 between the buffer index and the byte numbers in the
		//	Resolution-T manual.  The indicies listed below are in buffer
		//  coordinates:
		//  <index>   <type>		<field description>
		//	0		U8			<id> byte (0x8f)
		//	1		U8			<msg#> byte (0xac)
		//	4		U8			Survey progress (0-100%).  Valid if survey started, else = 0.
		//	11		U8			Minor Alarms 0 (low byte) - bitmapped
		//							ANT_OPEN		0x02
		//							ANT_SHORT		0x04
		//							NOT_TRACKING	0x08
		//							SURVEY_INPROG	0x20
		//							NO_STORED_POS	0x40
		//							LEAPSEC_PEND	0x80
		//	12		U8			Minor Alarms 1 (high byte) - bitmapped
		//							TEST_MODE		0x01
		//							POS_CONF_LOW	0x02
		//							NO_ALMANAC		0x08
		//							PPS_GENERATED	0x10
		//	13		U8			GPS error flags - value
		//							DOING_FIXES		0x00
		//							NO_GPS_TIME		0x01
		//							PDOP_TOOHI		0x03
		//							NO_SATS			0x08
		//							ONE_SAT			0x09
		//							TWO_SAT			0x0a
		//							THREE_SAT		0x0b
		//							SAT_UNUSABL		0x0c
		//							FIX_REJECTED	0x10
		//	33		F32			GPS temperature (C)
		//	61		F32			quantization error (s)

		if(((U8)tsipbuf[0] == 0x8f) && ((U8)tsipbuf[1] == 0xac)){
			set_tsip_timer(MS2000);							// reset timeout
			if(tsip_ptr > 64){
				tsip_valid = TRUE;
				tsip_stale = 0;								// refresh data
				gstat[survprog] = (U8)tsipbuf[4];			// gather relevant data
				gstat[minoralm1] = (U8)tsipbuf[11];
				gstat[minoralm0] = (U8)tsipbuf[12];
				gstat[gpstat] = (U8)tsipbuf[13];
				// gps temperature = single-float at index 33-36
				for(i=0, j=3;i<4;i++, j--){
					gps_temp.bytes[i] = tsipbuf[j+33];
				}
				// ppsquan = single-float at index 61-64
//				for(i=0;i<4;i++){
//					gps_qerr.bytes[i] = tsipbuf[i+61];
//				}
				switch(error_state){
				case MAJORERR:
				default:
//					sledstat = FASTBLINK;
					if(((gstat[minoralm0] & (ANT_OPEN | ANT_SHORT | NOT_TRACKING)) == 0) && ((gstat[minoralm1] & NO_ALMANAC) == 0)){
//						sledstat = QUICKBLINK;
						error_state = NOFIX;
					}
					break;

				case NOFIX:
					if(((gstat[minoralm0] & (ANT_OPEN | ANT_SHORT | NOT_TRACKING)) != 0) || ((gstat[minoralm1] & NO_ALMANAC) != 0)){
//						sledstat = FASTBLINK;
						error_state = MAJORERR;
					}
					if(gstat[gpstat] == DOING_FIXES){
						if((gstat[minoralm0] & NO_STORED_POS) != 0){
//							sledstat = SLOWBLINK;
							error_state = NOSURV;
						}else{
							error_state = NOERR;
//							sledstat = NOBLINK;
							// may need to trigger a survey here...
						}
					}
					break;
	
				case NOSURV:
					if(((gstat[minoralm0] & (ANT_OPEN | ANT_SHORT | NOT_TRACKING)) != 0) || ((gstat[minoralm1] & NO_ALMANAC) != 0)){
//						sledstat = FASTBLINK;
						error_state = MAJORERR;
					}
					if(gstat[gpstat] != DOING_FIXES){
//						sledstat = QUICKBLINK;
						error_state = NOFIX;
					}
					if(gstat[survprog] == 100){
						error_state = NOERR;
//						sledstat = NOBLINK;
					}
					break;
	
				case NOERR:
					if(((gstat[minoralm0] & (ANT_OPEN | ANT_SHORT | NOT_TRACKING)) != 0) || ((gstat[minoralm1] & NO_ALMANAC) != 0)){
//						sledstat = FASTBLINK;
						error_state = MAJORERR;
					}
					if(gstat[gpstat] != 0){
//						sledstat = QUICKBLINK;
						error_state = NOFIX;
					}
					if((gstat[minoralm0] & NO_STORED_POS) != 0){
//						sledstat = SLOWBLINK;
						error_state = NOSURV;
						}
					break;
				}
			}
		}
		// Look for GPS time:
		//	Primary timing packet is <id> = 0x8f msg# = 0xab
		//	The presence of the <id> in the buffer causes an offset
		//	of 1 between the buffer index and the byte numbers in the
		//	Resolution-T manual.  The indicies listed below are in buffer
		//  coordinates:
		//  <index>   <type>		<field description>
		//	0		U8			<id> byte (0x8f)
		//	1		U8			<msg#> byte (0xab)
		//	2		U32			<GPS seconds of week>
		//	6		U16			<GPS week#>
		//	8		S16			<UTC offset> (sec)
		//	10		U8			<timing flag> byte - bitmapped
		//											 (bit set, bit clear)
		//							UTCTIME		0x01 (else, gps time)
		//							UTCPPS		0x02 (else, gps pps)
		//							TIMESET		0x04 (time invalid, else, time valid)
		//							UTCINFO		0x08 (no offset info rcvd, else, UTCoffs valid)
		//							USERTIME	0x10 (else, time from gps)				
		//	11		U8			<seconds> byte
		//	12		U8			<minutes> byte
		//	13		U8			<hours> byte
		//	14		U8			<date> byte
		//	15		U8			<month> byte
		//	16		U16			<year> word - [16] is high byte, [17] is low byte

		if(((U8)tsipbuf[0] == 0x8f) && ((U8)tsipbuf[1] == 0xab)){
			// if buffer length = OK:
			if(tsip_ptr == 18){
				tsip_valid = TRUE;
//				set_ppsflag();								// trigger pps align
				// if GPS time is valid:
				gps_flags = tsipbuf[10];
				if(!(gps_flags & TIMESET)){
					set_ppsflag();								// trigger pps align
				}
				if((tsipbuf[10] & TIMESET) == 0){
//					iplTMR = TMRSECHACK;					// align DR seconds
					gwknum = ((U16)tsipbuf[7] & 0xff) | (((U16)tsipbuf[6] & 0xff) << 8);
					gtow = ((U32)tsipbuf[5] & 0xff) | (((U32)tsipbuf[4] & 0xff) << 8);
					gtow |= (((U32)tsipbuf[3] & 0xff) << 16) | (((U32)tsipbuf[2] & 0xff) << 24);
					gtime[ghour] = tsipbuf[13];				// capture time/date to working regs
					gtime[gmin] = tsipbuf[12];
					gtime[gsec] = tsipbuf[11];
					gtime[gmonth] = tsipbuf[15];
					gtime[gday] = tsipbuf[14];
					gyear = ((U16)tsipbuf[17] & 0xff) | (((U16)tsipbuf[16] & 0xff) << 8);
					gtime[guoffsH] = tsipbuf[8];
					gtime[guoffsL] = tsipbuf[9];
					utc_offs = ((U16)tsipbuf[9] & 0xff) | (((U16)tsipbuf[8] & 0xff) << 8);
					if(((gyear % 4) == 0) && ((gyear % 400) != 0)){
						last_day[FEB] = (LASTFEB + 2);		// set leap year
					}else{
						last_day[FEB] = (LASTFEB + 1);		// set normal year
					}
					// extract the time flags
					if((tsipbuf[10] & UTCTIME) == 0){
						gtime[gtref] = 'G';
					}else{
						gtime[gtref] = 'Z';
					}
					if((tsipbuf[10] & UTCPPS) == 0){
						gtime[gpref] = 'G';
					}else{
						gtime[gpref] = 'U';
					}
					if((tsipbuf[10] & UTCINFO) == 0){
						gtime[guinfo] = 'U';
					}else{
						gtime[guinfo] = 'X';
					}
				// ..otherwise, clear time flag displays
				}else{
					gtime[gtref] = 'x';				// set display flags to invalid
					gtime[gpref] = 'x';
					gtime[guinfo] = 'x';
				}
				gotgpstime = TRUE;
			}
		}
		tsip_state = TSTART;						// enable next tsip message RX
	}
	return gotgpstime;
}  // end tsip()

/*
	sprintf(buf,"%6u   x",456L);
	puts0(buf);
 */

// *********************************************
//  *************** SUBROUTINES ***************
// *********************************************

//=============================================================================
// get_lastday() return last day table entry
//=============================================================================
U8 get_lastday(U8 mon, U16 year){
	U8 temp;

	temp = last_day[mon];
	if(mon == FEB){
		if(isly(year)){
			temp += 1;
		}
	}
	return temp;
}

//=============================================================================
// get_gstat_ptr() return pointer to gps status array
//=============================================================================
U8* get_gstat_ptr(void){

	return gstat;
}

//=============================================================================
// get_gtime_ptr() return pointer to gps time array
//=============================================================================
U8* get_gtime_ptr(void){

	return gtime;
}

//=============================================================================
// get_gtime_arch_ptr() return pointer to gps archive time array
//=============================================================================
U8* get_gtime_arch_ptr(void){

	return gtime_arch;
}

//=============================================================================
// get_gyear() return gyear
//=============================================================================
U16 get_gyear(void){

	return gyear;
}

//=============================================================================
// get_utc_offs() return utc offset (sec)
//=============================================================================
S16 get_utc_offs(void){
	S16	si = 0;		// temp

	if(!(gps_flags & UTCTIME)){
		if(!(gps_flags & UTCINFO)){
			si = utc_offs;
		}
	}
	return si;
}

//=============================================================================
// enable_gps() set/clr gps_inhibit flag
//=============================================================================
void enable_gps(U8 flag){

	if(flag){
		gps_inhibit = 0;
	}else{
		gps_inhibit = 1;
	}
	return;
}

//=============================================================================
// get_gotime() return/clear gotgpstime flag
//=============================================================================
U8 get_gotime(U8 clear){

	if(clear || gps_inhibit){
		gotgpstime = FALSE;
	}
	return gotgpstime;
}

//=============================================================================
// get_gtemp() return gps temp to string at cptr
//=============================================================================
void get_gtemp(char* cptr){
	S16	si;			// temp

	si = (S16)(gps_temp.f * 10.0);
	sprintf(cptr,"%03d",si);
	return;
}

//=============================================================================
// disp_gpstat() display GPS status
//=============================================================================
void disp_gpstat(char* obuf){
	S16	si;

	si = ((U16)gtime[guoffsH] << 8) | ((U16)gtime[guoffsL]);
	sprintf(obuf,"tref: %c; ppsref: %c; info: %c; time: %02u-%02u-%04u  %02u:%02u:%02u UTCoffs: %d",gtime[gtref],gtime[gpref],gtime[guinfo],gtime[gmonth],gtime[gday],gyear,gtime[ghour],gtime[gmin],gtime[gsec],si);
	return;
}

//=============================================================================
// do_nextday() increments shadow calendar to next day
//=============================================================================
void do_nextday(void){

	if(++sday > get_lastday(smon, syear) - 1){				// process shadow calendar
		sday = 1;
		smon += 1;
		if(smon > DEC){
			syear += 1;
			smon = 1;
		}
	}
	return;
}

//=============================================================================
// set_gpsdate() copies current GPS date to shadow calendar and corrects shadow
//	calendr to local time.
//=============================================================================
void set_gpsdate(S16 tzone){
	S32	sii;

	// calc local time
	sii = ((S32)gtime[ghour] * 3600000L) + ((S32)gtime[gmin] * 60000L) + ((S32)gtime[gsec] * 1000L) - ((S32)utc_offs * 1000L);
	sii += (S32)tzone * 3600000L;
	sday = gtime[gday];
	smon = gtime[gmonth];
	syear = gyear;
	if(sii < 0){						// if lcltime < 0, local day is GPS day - 1
		if(--sday == 0){
			if(--smon == 0){
				syear -= 1;
				smon = 12;
			}
			sday = get_lastday(smon, syear) - 1;
		}
	}
	if(sii >= END_OF_DAY_1024){				// if lcltime > EOD, local day is GPS + 1
		do_nextday();
	}
	return;
}

//=============================================================================
// set_xdate() copies xRTC date to shadow calendar and corrects shadow
//	calendr to local time.
//=============================================================================
void set_xdate(S16 tzone, U8 mon, U8 day, U16 yrs, S32 xrtc_time){
	S32	sii;

	// calc local time
	sii = ((S32)tzone * 3600000L) + xrtc_time;
	sday = day;
	smon = mon;
	syear = yrs;
	if(sii < 0){						// if lcltime < 0, local day is GPS day - 1
		if(--sday == 0){
			if(--smon == 0){
				syear -= 1;
				smon = 12;
			}
			sday = get_lastday(smon, syear) - 1;
		}
	}
	if(sii >= END_OF_DAY_1024){				// if lcltime > EOD, local day is GPS + 1
		do_nextday();
	}
	return;
}

//=============================================================================
// set_sdate() stores values into local shadow calendar.
//=============================================================================
void set_sdate(U8 mon, U8 day, U16 yr){

	smon = mon;							// set shadow calendar
	sday = day;
	syear = yr;

	return;
}

//=============================================================================
// get_smon() returns shadow month
//=============================================================================
U8 get_smon(void){

	return smon;
}

//=============================================================================
// get_smon() returns shadow month
//=============================================================================
U8 get_sday(void){

	return sday;
}

//=============================================================================
// get_smon() returns shadow month
//=============================================================================
U16 get_syear(void){

	return syear;
}

//=============================================================================
// get_ghour() returns current gps hour
//=============================================================================
U8 get_ghour(void){

	return gtime[ghour];
}

//=============================================================================
// get_ghour() returns current gps hour
//=============================================================================
U8 get_gpstat(void){

 	return gstat[gpstat];
}

//=============================================================================
// get_tsip_valid() returns current tsip data valid status
//=============================================================================
U8 get_tsip_valid(void){

 	return tsip_valid;
}

//=============================================================================
// get_wknum() returns current week#
//=============================================================================
U16 get_wknum(void){

 	return gwknum;
}

//=============================================================================
// get_tow() returns current tow
//=============================================================================
U32 get_tow(void){

 	return gtow;
}

//=============================================================================
// arch_gps() archives the current GPS time data
//=============================================================================
void arch_gps(U8 cmd){
	U8	i;		//temp
	//	gsec, gmin, ghour, gmonth, gday, gtref, gpref, guinfo};

	if(cmd == 0xff){						// set invalid time/date
		gtime_arch[gsec] = 60;
		gtime_arch[gmin] = 60;
		gtime_arch[ghour] = 24;
		gtime_arch[gmonth] = 13;
		gtime_arch[gday] = 32;
		gtime_arch[gtref] = 'x';
		gtime_arch[gpref] = 'x';
		gtime_arch[guinfo] = 'x';
		gyear_arch = 0;
	}else{
		gyear_arch = gyear;					// archive GPS time/date regs from last re-sync
		for(i=0;i<8;i++){
			gtime_arch[i] = gtime[i];
		}
	}
 	return;
}

//=============================================================================
// get_secofday() returns current time in sec as elapsed from midnight
//=============================================================================
U32 get_secofday(U8* ptr){
	U32	ii;
	//	gsec, gmin, ghour, gmonth, gday, gtref, gpref, guinfo};

	ii = (U32)*(ptr+gsec) + ((U32)*(ptr+gmin) * 60L) + ((U32)*(ptr+ghour) * 3600L);
	return ii;
}

//=============================================================================
// get_dayofyr() returns #days from start of year
//=============================================================================
U16 get_dayofyr(U8* ptr, U16 yrval){
	U8	i;		// temps
	U16	ii = 0;
	//	gsec, gmin, ghour, gmonth, gday, gtref, gpref, guinfo};

	for(i=1; i<*(ptr+gmonth); i++){
		ii += get_lastday(i,yrval) - 1;
	}
	ii += *(ptr+gday);
	return ii;
}

//=============================================================================
// get_gpset() return sec since GPS archive
//=============================================================================
U32 get_gpset(void){
	U16	diay;		//temps
	U16	dicy;
	U16 i;
	U32	siad;
	U32	sicd;

	diay = get_dayofyr(gtime_arch, gyear_arch);		// get #days elapsed since newyear in archive
	dicy = get_dayofyr(gtime, gyear);				// get #days elapsed since newyear in current
	if(gyear == gyear_arch){						// if same year, just take the difference
		dicy = dicy - diay;
	}else{
		if(isly(gyear_arch)){						// calc # days remaining in arch year
			diay = 366 - diay;
		}else{
			diay = 365 - diay;
		}
		dicy += diay;								// intermediate total
		if(gyear > gyear_arch + 1){					// calc # days in intervening years
			for(i=gyear_arch+1; i<gyear; i++)
				if(isly(i)){
					dicy += 366;
				}else{
					dicy += 365;
				}
		}
	}
	// dicy = # elapsed days since archive
	siad = get_secofday(gtime_arch);
	sicd = get_secofday(gtime);
	if(dicy){
		if(sicd < siad){
			dicy -= 1;								// correct for tod non-overlap
		}
		siad = 86400L - siad;						// #sec left in arch day
		sicd = sicd + siad + ((U32)dicy * 86400L);		// # sec since GPS archive
	}else{
		sicd -= siad;								// # sec since GPS archive
	}
	// sicd = #sec since gps archive
	return sicd;
}

#undef IS_TSIPC
//**************
// End Of File
//**************

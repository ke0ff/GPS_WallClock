 /********************************************************************
 ************ COPYRIGHT (c) 2021by KE0FF, Taylor, TX   *************
 *
 *  File name: rtc.c
 *
 *  Module:    Control
 *
 *  Summary:
 *  RTC definitions and support Fns
 *
 *******************************************************************/

/********************************************************************
 *  File scope declarations revision history:
 *    11-17-18 jmh:  Adapted from ACU for the GPS Wall clock.
 *
 *    10-04-15 jmh:  Have majority of time functions coded and preliminary testing shows no serious bugs.
 *    					* Set time/date and several querry Fns
 *    					* System does a periodic check for DST change at IPL.  Due to nature of this system,
 *    						it will likely not be powered during a time change transition, so capturing this
 *    						at IPL is a 99.9% solution.
 *    						--> [not done yet] --> To cover most of the remaining possibilities,
 *    						if the system is powered on the day of, or day prior to a time change, an MCU
 *    						timed event could be scheduled to run isdst() just after the transition.
 *    					* need to look at creating an MCU driven RTC with public resolution of 1 sec that mirrors
 *    						the i2c RTC.  This would allow alarm-driven event scheduling.  DS1307 has 1HZ output
 *    						option that could drive an MCU edge interrupt (apply PU=1 for given pin) which would
 *    						drive the working clock registers.  This would be exactly in-sych with the DS1307 thus
 *    						allowing i2c transfers to happen only on IPL or when any user changes are implemented
 *    					* RTC raw registers are maintained at GMT with a time-zone correction that is applied
 *    						to produce local time.  DST is also used to produce local time.  To accomodate the
 *    						possibility that the DST transition dates may change in the future, the SW maintains
 *    						4 RTC RAM registers that hold the month and week of the start and stop days for DST.
 *    						The start event ocurrs at 01:00L (time-zone only correction) on the Sunday identified
 *    						by the registers.  The stop event ocurrs at 02:00L on the corresponding Sunday identifed
 *    						by the registers.
 *    					* added checksum system to validate RTC ram for YYxx to CHKS_H-1.  Currently only displays
 *    						error banner if checksum on read does not match.  Places onus on operator to validate RAM
 *    						contents (for now).
 *    08-26-15 jmh:  creation date
 *
 *******************************************************************/

#include <stdint.h>
#include "typedef.h"
#include "inc/tm4c123gh6pm.h"
#include "rtc.h"
#include "tsip.h"
#include "eeprom.h"
#include "init.h"
#include "du.h"

//=============================================================================
// local registers

enum gps_idx{gsec,gmin,ghour,gmonth,gday,gtref,gpref,guinfo,guoffsH,guoffsL};

char	dow_str_list[] = { "ERR\0SUN\0MON\0TUE\0WED\0THU\0FRI\0SAT\0" };
char	mon_str_list[] = { "ERR\0JAN\0FEB\0MAR\0APR\0MAY\0JUN\0JUL\0AUG\0SEP\0OCT\0NOV\0DEC\0" };
U8		dst_array[4];
U8		pcf2129_array[XR_END+1];	// ext RTC register mirror -- 1st byte is command reg
U8		rtc_array[XR_END+1];		// ext RTC register working

//=============================================================================
// local Fn declarations

//=============================================================================

//***************************************************************************
// get_dst_info stores the dst/std month/wk to RAM
U8* get_dst_info(void){

	return dst_array;
}

//***************************************************************************
// set_dst_info stores the dst/std month/wk to RAM
void set_dst_info(U8 dhmon, U8 dhwk, U8 shmon, U8 shwk){

	dst_array[std_mon] = shmon;
	dst_array[std_wk] = shwk;
	dst_array[dst_mon] = dhmon;
	dst_array[dst_wk] = dhwk;
	return;
}

//***************************************************************************
// get_ee_dst fetches the dst/std month/wk from EEPROM

void get_ee_dst(void){
	U32 ii;

	ii = eerd(EE_DST_STOR);
	dst_array[std_mon] = (U8)(ii >> 24); 	// fetch dst info from eeprom
	dst_array[std_wk] = (U8)(ii >> 16);
	dst_array[dst_mon] = (U8)(ii >> 8);
	dst_array[dst_wk] = (U8)(ii);
	return;
}

//***************************************************************************
// wr_ee_dst stores the dst/std month/wk to EEPROM

void wr_ee_dst(void){
	U32 ii;

	ii = (U32)dst_array[std_mon] << 24;
	ii |= (U32)dst_array[std_wk] << 16;
	ii |= (U32)dst_array[dst_mon] << 8;
	ii |= (U32)dst_array[dst_wk];
	eewr(EE_DST_STOR, ii);
	return;
}
//***************************************************************************
// rtc_ram_io maintains the working array for the RTC contents
// returns pointer to start of array.  If cmd == 0xff, clear the array

volatile U8* rtc_ram_io(U8 cmd){
	static	volatile U8	rtc_ram[64];		// rtc working array
	U8	i;						// temp

	if(cmd == 0xff){
		for(i = 0;i<64;i++){				// pre-set the array
			rtc_ram[i] = 0xff;				// mimics the erasure of an EEPROM based memory
		}
	}
	return rtc_ram;							// return pointer to array
}

//=============================================================================
// adjust_rtc_local() applies DELTA_GMT and DST factors to time array to get local time
//	caller must already have loaded RAM from chip before running this fn
//=============================================================================
void adjust_rtc_local(U8 dst_i){
	volatile U8*	ramptr;		// pointer to RAM copy array
	S8	hr_i;					// temps
//	U8	date_i;
	U8	month_i;
	U16	year_i;
	U16	i;

	ramptr = rtc_ram_io(0);									// get pointer to ram copy
	hr_i = bcd_int(ramptr[HR]);
	hr_i = hr_i + (S8)dst_i + (S8)ramptr[DELTA_GMT];	// do basic hours correction
	// now, check to see if a midnight crossing is involved:
	if(hr_i < 0){										// if underflow, jump back to previous day
		ramptr[HR] = int_bcd(hr_i + 24);					// correct hours
		ramptr[DOW] -= 1;									// correct DOW
		if(ramptr[DOW] == 0) ramptr[DOW] = 7;
		ramptr[DATE] = int_bcd(bcd_int(ramptr[DATE]) - 1);	// correct date
		if(ramptr[DATE] == 0){								// if date underflows, it is a month crossing
			month_i = bcd_int(ramptr[MONTH]) - 1;			// correct month
			year_i = ((U16)ramptr[YYxx] << 8) | (U16)ramptr[xxYY]; // get int year, we need it later no matter what
			year_i = bcd_int16(year_i);						// get int year, we need it later no matter what
			if(month_i == 0){								// if month underflows, it is a new-year crossing
				ramptr[MONTH] = 0x12;						// reset month
				year_i -= 1;								// correct year
				i = int_bcd16(year_i);						// and convert back to BCD
				ramptr[xxYY] = (U8)(i & 0xff);
				ramptr[YYxx] = (U8)(i >> 8);
				month_i = 12;								// need int updated to correct date below
			}else{
				ramptr[MONTH] = int_bcd(month_i);			// no month underflow, convert back to BCD
			}
			ramptr[DATE] = int_bcd((U8)dsny(0, month_i, 0xffff)); // new date is last day of new month
			if(isly(year_i) && (month_i == 2)) ramptr[DATE] = 0x29; // if LY and (new) month = feb, update
		}
	}else{
		if(hr_i > 23){										// if overflow, jump to next day
			ramptr[HR] = int_bcd(hr_i - 24);				// correct hours
			ramptr[DOW] += 1;								// correct DOW
			if(ramptr[DOW] == 8) ramptr[DOW] = 1;
			ramptr[DATE] = int_bcd(bcd_int(ramptr[DATE]) + 1); // correct date
			year_i = ((U16)ramptr[YYxx] << 8) | (U16)ramptr[xxYY]; // get int year, we need it later no matter what
			year_i = bcd_int16(year_i);						// get int year, we need it later no matter what
			i = dsny(0, bcd_int(ramptr[MONTH]), 0xffff);	// get # days in current month
			if(isly(year_i) && (month_i == 2)) i = 29;		// if LY and (new) month = feb, update
			i = (U16)int_bcd(i);
			if(ramptr[DATE] > (U8)i){						// if date overflows, it is a month crossing
				month_i = bcd_int(ramptr[MONTH]) + 1;		// correct month
				if(month_i > 12){							// if month underflows, it is a new-year crossing
					ramptr[MONTH] = 0x01;					// reset month
					year_i += 1;							// correct year
					i = int_bcd16(year_i);					// and convert back to BCD
					ramptr[xxYY] = (U8)(i & 0xff);
					ramptr[YYxx] = (U8)(i >> 8);
				}else{
					ramptr[MONTH] = int_bcd(month_i);		// no month underflow, convert back to BCD
				}
			}
		}else{
			ramptr[HR] = int_bcd(hr_i);						// convert hr back to BCD
		}
	}
	return;
}

//=============================================================================
// adjust_rtc_gmt() applies DELTA_GMT and DST factors to time array to convert
//	local time to GMT.  Must have local time entered into RAM array prior to exec.
//=============================================================================
void adjust_rtc_gmt(void){
	volatile U8*	ramptr;		// pointer to RAM copy array
	S8	hr_i;					// temps
//	U8	date_i;
	U8	month_i;
	U16	year_i;
	U16	i;

	ramptr = rtc_ram_io(0);									// get pointer to ram copy
	hr_i = bcd_int(ramptr[HR]);
	hr_i = hr_i - (S8)ramptr[DST] - (S8)ramptr[DELTA_GMT];	// do basic hours correction
	// now, check to see if a midnight crossing is involved:
	if(hr_i < 0){										// if underflow, jump back to previous day
		ramptr[HR] = int_bcd(hr_i + 24);				// correct hours
		ramptr[DOW] -= 1;									// correct DOW
		if(ramptr[DOW] == 0) ramptr[DOW] = 7;
		ramptr[DATE] = int_bcd(bcd_int(ramptr[DATE]) - 1);	// correct date
		if(ramptr[DATE] == 0){								// if date underflows, it is a month crossing
			month_i = bcd_int(ramptr[MONTH]) - 1;			// correct month
			year_i = ((U16)ramptr[YYxx] << 8) | (U16)ramptr[xxYY]; // get int year, we need it later no matter what
			year_i = bcd_int16(year_i);						// get int year, we need it later no matter what
			if(month_i == 0){								// if month underflows, it is a new-year crossing
				ramptr[MONTH] = 0x12;						// reset month
				year_i -= 1;								// correct year
				i = int_bcd16(year_i);						// and convert back to BCD
				ramptr[xxYY] = (U8)(i & 0xff);
				ramptr[YYxx] = (U8)(i >> 8);
				month_i = 12;								// need int updated to correct date below
			}else{
				ramptr[MONTH] = int_bcd(month_i);			// no month underflow, convert back to BCD
			}
			ramptr[DATE] = int_bcd((U8)dsny(0, month_i, 0xffff)); // new date is last day of new month
			if(isly(year_i) && (month_i == 2)) ramptr[DATE] = 0x29; // if LY and (new) month = feb, update
		}
	}else{
		if(hr_i > 23){										// if overflow, jump to next day
			ramptr[HR] = int_bcd(hr_i - 24);				// correct hours
			ramptr[DOW] += 1;								// correct DOW
			if(ramptr[DOW] == 8) ramptr[DOW] = 1;
			ramptr[DATE] = int_bcd(bcd_int(ramptr[DATE]) + 1); // correct date
			year_i = ((U16)ramptr[YYxx] << 8) | (U16)ramptr[xxYY]; // get int year, we need it later no matter what
			year_i = bcd_int16(year_i);						// get int year, we need it later no matter what
			i = dsny(0, bcd_int(ramptr[MONTH]), 0xffff);	// get # days in current month
			if(isly(year_i) && (month_i == 2)) i = 29;		// if LY and (new) month = feb, update
			i = (U16)int_bcd(i);
			if(ramptr[DATE] > (U8)i){						// if date overflows, it is a month crossing
				month_i = bcd_int(ramptr[MONTH]) + 1;		// correct month
				if(month_i > 12){							// if month underflows, it is a new-year crossing
					ramptr[MONTH] = 0x01;					// reset month
					year_i += 1;							// correct year
					i = int_bcd16(year_i);					// and convert back to BCD
					ramptr[xxYY] = (U8)(i & 0xff);
					ramptr[YYxx] = (U8)(i >> 8);
				}else{
					ramptr[MONTH] = int_bcd(month_i);		// no month underflow, convert back to BCD
				}
			}
		}else{
			ramptr[HR] = int_bcd(hr_i);						// convert hr back to BCD
		}
	}
	return;
}

//=============================================================================
// dow_str() returns pointer to 3-chr dow string
//	index 0 is "ERR" text
//=============================================================================
char* dow_str(U8 dow){

	return &dow_str_list[((dow) & 0x07) * 4];	// calc index to corresponding text and return pointer
}

//=============================================================================
// mon_str() returns pointer to 3-chr month string
//	index 0 is "ERR" text
//=============================================================================
char* mon_str(U8 month){
	U8	i;		// temp

	if(month & 0x80){
		i = month & 0x7f;
	}else{
		i = (((month & 0x10) >> 4) * 10) + (month & 0x0f);
	}
	if(i > 12){
		i = 0;
	}
	return &mon_str_list[i * 4];	// calc index to corresponding text and return pointer
}

//=============================================================================
// turn_off_rtc() sets bit 0x80 of RTC SEC reg to turn it off.
//=============================================================================
void turn_off_rtc(void){
	volatile U8*	ramptr;		// pointer to RAM copy array

	ramptr = rtc_ram_io(0);					// get pointer to ram copy
	ramptr[SEC] |= RTC_OSC_OFF;
	return;
}

//=============================================================================
// set_time() validates time params and writes them to the rtc RAM
//=============================================================================
U8 set_time(U8 hh, U8 mm, U8 ss){
	U8	rtn = TRUE;				// time set complete response
	volatile U8*	ramptr;		// pointer to RAM copy array
	U8	bcd_hh;					// temps
	U8	bcd_mm;
	U8	bcd_ss;

	if(hh > 23) rtn = FALSE;							// validate hh, mm, ss
	if(mm > 59) rtn = FALSE;
	if(ss > 59) rtn = FALSE;
	if(rtn){											// if valid,
		bcd_hh = int_bcd(hh);							// convert int values to BCD
		bcd_mm = int_bcd(mm);
		bcd_ss = int_bcd(ss);
		ramptr = rtc_ram_io(0);							// get pointer to RAM copy
		ramptr[SEC] = bcd_ss;							// stroe BCD values to RAM array
		ramptr[MIN] = bcd_mm;
		ramptr[HR] = bcd_hh;
	}
	return rtn;											// return TRUE if valid time, else FALSE
}

//=============================================================================
// set_date() validates date params and writes them to the rtc RAM
//=============================================================================
U8 set_date(U8 mon, U8 day, U16 year){
	U8	rtn = TRUE;			// time set complete response
	volatile U8*	ramptr;	// pointer to RAM copy array
	U8	bcd_mon;			// temps
	U8	bcd_day;
	U16	bcd_year;
	U8	dow;

	if((mon > 12) || (mon == 0)) rtn = FALSE;			// validate month
	if(year < 2001) rtn = FALSE;						// validate year
	if(rtn){
		if((day > dsny(0, mon, 0xffff)) || (day == 0)){	// validate day of month
			rtn = FALSE;
		}else{
			dow = dow_calc(day, mon, year);				// calc DOW
			bcd_mon = int_bcd(mon);						// convert int values to BCD
			bcd_day = int_bcd(day);
			bcd_year = int_bcd16(year);
			ramptr = rtc_ram_io(0);						// get pointer to ram copy array
			ramptr[DOW] = dow;							// store BCD values to RAM array
			ramptr[MONTH] = bcd_mon;
			ramptr[DATE] = bcd_day;
			ramptr[xxYY] = (U8)(bcd_year & 0xff);
			ramptr[YYxx] = (U8)((bcd_year >> 8) & 0xff); // this is REF location of year 1000's/100's BCD
		}
	}
	return rtn;											// return TRUE if valid date, else FALSE
}

//=============================================================================
// isdst() determines if DST is in effect based on state of RTC array.
//	caller must refresh RTC array and call adjust_rtc_local(0) before calling
//	this routine.  Thus, this fn is based on the adjustment without DST correction
//=============================================================================
U8	isdst(void){
			U8		rtn = FALSE;	// temp rtn, default to NOT DST
			U8		i;				// temp
			U8		gmonth_i;		// temp shadow calendar
			U8		gday_i;
			U16		gyear_i;
			S16		tzon;
			S8		ghour_i;

	tzon = get_timez();										// get current local hour
	ghour_i = (S8)((S16)get_ghour() + tzon);
	if(ghour_i < 0){
		ghour_i += 24;
	}
	if(ghour_i > 23){
		ghour_i -= 24;
	}
	gmonth_i = get_smon();									// get current shadow calendara
	gday_i = get_sday();
	gyear_i = get_syear();
	if((gmonth_i > dst_array[dst_mon]) && (gmonth_i < dst_array[std_mon])){
		rtn = TRUE;											// if month is between start and end month, then DST = TRUE
	}
	if(gmonth_i == dst_array[dst_mon]){						// if month == start month, see if DST or not
		i = 7 - dow_calc(1, gmonth_i, gyear_i);				// calc date of DST_SUN'th
		i = (dst_array[dst_wk] * 7) - i;
		if(gday_i > i){										// if after this date, DST is TRUE
			rtn = TRUE;
		}
		if((gday_i == i) && (ghour_i >= 0x01)){				// if on this date, DST is true on or after 01:00 (un-corrected lcl time)
			rtn = TRUE;
		}
	}
	if(gmonth_i == dst_array[std_mon]){						// if month == end month, see if it is DST or not
		i = 7 - dow_calc(1, gmonth_i, gyear_i) + 1; 		// calc date of STD_SUN'th
		i = (dst_array[std_wk] * 7) - i;
		if(gday_i < i){										// if before this date, DST is TRUE
			rtn = TRUE;
		}
		if((gday_i == i) && (ghour_i < 0x02)){				// if on this date, DST is true before 02:00 (un-corrected lcl time)
			rtn = TRUE;
		}
	}
	return rtn;
}

//=============================================================================
// isly() determines if given year is a leap year
//=============================================================================
U8	isly(U16 year){
//	U16	i;			// temp
	U8		r = FALSE;	// temp rtrn, default to NOT a leap year

	if((year%400 == 0) || ((year%100 != 0) && (year%4 == 0))){
		r = TRUE;
	}
/*	i = year/4;
	if((year - (i*4)) == 0){						// if year is evenly divisible by 4, it is a candidate for a leap year
		i = year/400;
		if((year - (i*400)) == 0){					// if year is evenly divisible by 400, it is a leap year
			r = FALSE;
		}else{
			i = year/100;
			if((year - (i*100)) != 0){				// if year is NOT evenly divisible by 100, it is NOT a leap year
				r = TRUE;
			}
		}
	}*/
	return r;
}

//=============================================================================
// dsny() calculates # days since new year (1/1/year) and includes leap-day
//	if isly() && month
//	if year = 0xffff, this fn returns last day of mon only
//=============================================================================
U16 dsny(U8 day, U8 mon, U16 year){
	// list of last day of month for JAN(1)-DEC(12)
	U8	last_day[] = { 0,31,28,31,30,31,30,31,31,30,31,30,31 };
	U8		i;				// temp
	U8		ld = 0;			// leap day holding reg
	U16		days = day;		// accum, start with #days in current month

	if(year == 0xffff){								// code trap to allow last_day[] array to be reused to provide last day of indicated month
		days = last_day[mon];						// return last day of input month
		if((isly(year) && (mon == 2))){
			days += 1;								// it's FEB in a LY, add leap day to end of month
		}
	}else{
		i = last_day[mon];							// get end of current month
		if((isly(year) && (mon == 2))){
			i += 1;									// it's FEB in a LY, add leap day to end of month
		}
		if((day <= i) && (mon <= 12)){				// if mon/day are valid, calc days up to current month
			if((isly(year) && (mon > 2))){
				ld = 1;								// it's after FEB in a LY, add leap day to end of month
			}
			i = 1;
			while(i < mon){
				days += last_day[i++];				// sum up prior months
			}
			days += ld;								// add leap day reg
			if(year == 2000) days--;				// first year correction (first year starts at 0)
		}else{
			days = 999;								// set error exit
		}
	}
	return days;
}

//=============================================================================
// dse() calculates # days since Y2K epoch (Saturday, 1/1/2000)
//	not valid for years < 2000 (returns 0).
//=============================================================================
U32 dse(U8 day, U8 mon, U16 year){
	U32	i;					// temps
	U32	j;
	U32	k;

	if(year >= 2000){
		// days in prior years (back to 2000)
		i = ((U32)year-2000L) * 365L;				// number of days from epoch to beginning of this year
		// number of leap-days since 2000
		j = (U32)((year - 2000)/4);					// have to sum this way to avoid rounding errors
		j -= (U32)((year - 2000)/100);
		j += (U32)((year - 2000)/400);
		// number of days since 1/1/year
		k = (U32)dsny(day, mon, year);				// number of days since 1/1/ of this year (includes leap-day for current year)
		return (i + j + k);							// dse = sum of the above 3 calculations
	}
	return 0;										// invalid year, return 0
}

//=============================================================================
// dow_calc() calculates day of week by calculating the remainder of
//	"days_since_epoch/7" ... "7" (adjusted from 0) = Saturday, "1" = Sunday,
//	"6" = Friday.
//	Returns 0xff if year < 2000.
//=============================================================================
U8	dow_calc(U8 day, U8 mon, U16 year){
	U8	rtn;				// temp

	if(year >= 2000){
		rtn = (dse(day, mon, year) % 7);			// calc DOW index value
		if(rtn == 0) rtn = 7;						// convert "0" to "7" to align index with the dow LUT
	}else{
		rtn = 0xff;									// error return
	}
	return rtn;
}

//=============================================================================
// bcd_int() converts packed bcd byte to int.  If invalid BCD, returns NAN_BCD8
//=============================================================================
U8	bcd_int(U8 bcd){
	U8	i;					// temp

	i = (bcd & 0x0f);								// mask lsnyb
	if(i>9){
		i = NAN_BCD8;								// if invalid, set error return
	}else{
		i += ((bcd >> 4) & 0x0f) * 10;				// shift lsnyb and calc final bcd value
		if(i>99) i = NAN_BCD8;						// if invalid, set error return
	}
	return i;										// return value of bcd
}

//=============================================================================
// bcd_int16() converts packed bcd word to int.  If invalid BCD, returns NAN_BCD8
//=============================================================================
U16	bcd_int16(U16 bcd){
	U16	i;					// temp
	U16	j = 0;				// temp, clear error rtrn

	i = (bcd & 0x000f);								// mask lsnyb
	if(i>9) j = NAN_BCD16;							// if invalid, set err
	i += ((bcd & 0x00f0) >> 4) * 10;				// mask & shift nyb1 and add value to accum
	if(i>99) j = NAN_BCD16;							// if invalid, set err
	i += ((bcd & 0x0f00) >> 8) * 100;				// mask & shift nyb2 and add value to accum
	if(i>999) j = NAN_BCD16;						// if invalid, set err
	i += ((bcd & 0xf000) >> 12) * 1000;				// mask & shift nyb3 and add value to accum
	if(i>9999) j = NAN_BCD16;						// if invalid, set err
	if(j == NAN_BCD16) i = j;						// latch error
	return i;										// return value of bcd
}

//=============================================================================
// int_bcd() converts int byte to packed bcd byte.  If int > 99, returns NAN_BCD8
//=============================================================================
U8	int_bcd(U8 i){
	U8	bcd = NAN_BCD8;		// temp, set default return = error
	U8	t;					// temp

	if(i<=99){										// if valid, process
		t = i/10;									// calc 10's digit
		bcd = i - (t * 10);							// calc 1's digit
		bcd |= t << 4;								// combine with 10's
	}
	return bcd;										// return bcd representation of value
}

//=============================================================================
// int_bcd16() converts int word to packed bcd word.  If int > 9999, returns NAN_BCD16
//=============================================================================
U16	int_bcd16(U16 i){
	U16	bcd = NAN_BCD16;	// temp, set default return = error
	U16	t;					// temp
	U16	it = i;				// temp

	if(i<=9999){									// if valid, process
		t = it/1000;								// calc 1000's digit
		bcd = t << 12;								// place digit into result
		it -= t * 1000;								// calc remainder
		t = it/100;									// calc 100's digit
		bcd |= (t << 8) & 0x0f00;					// place digit into result
		it -= t * 100;								// calc remainder
		t = it / 10;								// calc 10's digit
		bcd |= (t << 4) & 0x00f0;					// place digit into result
		it -= t * 10;								// calc remainder (1's digit)
		bcd |= it & 0x000f;							// place digit into result
	}
	return bcd;										// return bcd representation of value
}

//=============================================================================
// chks_ram() calculates 16b checksum of RAM array addr 8-14
//=============================================================================
U16	chks_ram(volatile U8* rptr){
	U16	chks = 0;			// temp
	U8	i;					// temp

	for(i=YYxx;i<CHKS_H;i++){
		chks += rptr[i];
	}
	return chks;										// return bcd representation of value
}

//=============================================================================
// rtc_spi() sends/rcvs string to PCF2129
//=============================================================================
void rtc_spi(U8* txptr, U8 txlen, U8* rxptr){
//U8		pcf2129_array[XR_END];	// ext RTC register mirror
	U8* tptr = txptr;
	U8* rptr = rxptr;
	U8	i;

	GPIO_PORTA_DATA_R &= ~RTC_CSN;	 // bring RTC CS low
	for(i=0;i<txlen;i++){
		*rptr++ = send_spi(*tptr++);
	}
	wait2(5);
	GPIO_PORTA_DATA_R |= RTC_CSN;	 // bring RTC CS hi
	return;
}

//=============================================================================
// rtc_rw() sets/reads time from PCF2129
//	uses the seconds alarm to hold the thousands and hundreds of years.
//	This will hold time up to the year 5999, but SW will have to figure
//	out how to manage the 99-00 year crossings
//=============================================================================
void rtc_rw(U8* mptr, U8 rw){

	// 1st buffer byte is r/w/address pre-amble
	*mptr = (XR_SA | 0x03);		// select device starting at address 0x03 (seconds reg)
	if(rw){
		*mptr |= XR_RW;			// if read, set read bit
	}
//	*mptr |= (XR_SA | 0x03);
	rtc_spi(mptr, 9, mptr);		// send/rcv data (use tx buffer for rx)
	return;
}

//=============================================================================
// get_rtcxptr() returns pointer to ext rtc data array
//	if scratch_ptr_select == 0, return main ptr
//	else return scratch ptr
//=============================================================================
U8* get_rtcxptr(U8 scratch_ptr_select){
	U8*	iptr;

	if(scratch_ptr_select){
		iptr = rtc_array;
	}else{
		iptr = pcf2129_array;
	}
	return iptr;
}

//=============================================================================
// rtc_stop() stops or starts the RTC
//=============================================================================
void rtc_stop(U8 stop){
	U8	tbuf[4];

	if(stop){
		tbuf[1] = STOP;
	}else{
		tbuf[1] = SI;
	}
	tbuf[2] = 0;
	tbuf[3] = 0;
	tbuf[0] = (XR_SA | XR_CNTL1);
	rtc_spi(tbuf, 4, tbuf);
	return;
}

//=============================================================================
// rtc_init() inits clk out & otp refresh
//=============================================================================
void rtc_init(void){
	U8	tbuf[2];

	tbuf[1] = 0;						// toggle OTPR
	tbuf[0] = (XR_SA | XR_CNTLCK);
	rtc_spi(tbuf, 2, tbuf);
	wait(100);
	tbuf[1] = OTPR | COF2 | COF0;		// and set 1024 Hz clk out
	tbuf[0] = (XR_SA | XR_CNTLCK);
	rtc_spi(tbuf, 2, tbuf);
	wait(100);
	tbuf[1] = TI_TP;					// pulse /INT out
	tbuf[0] = (XR_SA | XR_CNTLWD);
	rtc_spi(tbuf, 2, tbuf);
	wait(100);
	rtc_stop(0);						// set 24H mode
	return;
}

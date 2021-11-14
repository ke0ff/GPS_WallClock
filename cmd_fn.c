/********************************************************************
 ************ COPYRIGHT (c) 2021 by ke0ff, Taylor, TX   *************
 *
 *  File name: cmd_fn.c
 *
 *  Module:    Control
 *
 *  Summary:
 *  CLI Command Interpreter
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
#include "version.h"
//#include "I2C0.h"
#include "io.h"
#include "tiva_init.h"
#include "tsip.h"
#include "du.h"
#include "rtc.h"

//=============================================================================
// local registers

U16	string_addr;						// string-parse next empty address
U16 sernum;								// serial number utility register
#define MAX_PRESP_BUF 80
char bcmd_resp_buf[MAX_PRESP_BUF + 10];
char* bcmd_resp_ptr;

// enum error message ID
enum err_enum{ no_response, no_device, target_timeout };

// enum list of command numerics
//	each enum corresponds to a command from the above list (lastcmd corresponds
//	with the last entry, 0xff)
enum cmd_enum{ gpsen,bright,cal,log_dat,config_dst,adc_buff,gtest,msg,timer_tst,qtest,sec,setime,timez,help1,help2,vers,lastcmd,helpcmd };
//           {"A\0   B\0    CAL C\0     D\0        DST\0    G\0   M\0 TI\0      Q\0   SEC S\0    Z\0   ?\0   H\0   V\0\xff"};
//const char cmd_list[] = {"A\0B\0CAL\0C\0DST\0DC\0G\0M\0TI\0Q\0SEC\0S\0Z\0?\0H\0VERS\0\xff"};

#define	cmd_type	char	// define as char for list < 255, else define as int

//=============================================================================
// local Fn declarations

U8 get_Dargs(U8 argsrt, U8 nargs, char* args[ARG_MAX], U16 params[ARG_MAX]);
cmd_type cmd_srch(char* string);
char do_cmd_help(U8 cmd_id);
char parm_srch(U8 nargs, char* args[ARG_MAX], char* parm_str, U8 force_len);
void parse_ehex(char * sptr);
void disp_error(U8 errnum);
void disp_fail(char* buf, char* s, U16 c, U16 d);
void disp_wait_addr(char* buf);
U16 boot_se(U8* bptr, U16 start, U16 end, U16 ppaddr);
U8* log_error_byte(U8* lbuf, U8 d, U8 h, U16 a);
void disp_error_log(U8* lbuf, U16 len);
void do_help(void);
void disp_esc(char flag);

//=============================================================================
// CLI cmd processor entry point
//	Uses number of args (nargs) and arg pointer array (args[]) to lookup
//	command and execute.
//	offset is srecord offset value which is maintianed in main() and passed
//	forward to srecord functions (upload)
//=============================================================================

int x_cmdfn(U8 nargs, char* args[ARG_MAX], U16* offset){

#define	OBUF_LEN 110				// buffer length
#define	MAX_LOG_ERRS (OBUF_LEN / 4)	// max number of errors logged by verify cmd
	char	obuf[OBUF_LEN];			// gp output buffer
	U16		params[ARG_MAX];		// holding array for numeric params
	U8		u8buf[30];
	char	c;						// char temp
	char	pw = FALSE;				// W flag (set if "W" found in args)
//	char	pc = FALSE;				// C flag (set if "C" found in args)
//	char	pq = FALSE;				// ? flag (set if "?" found in args)
	char	pm = FALSE;				// minus flag (set if <wsp>-<wsp> found in args)
	char	pv = FALSE;				// V flag (set if <wsp>V<wsp> found in args)
	U8		dst_select = 0;
	int		cmd_found = TRUE;		// default to true, set to false if no valid cmd identified
	char*	s;						// temp pointers
	char*	t;
	U8*		sp;
	char	cmd_id;					// command enumerated id
	char	cmd_id1;				// command enumerated id for 1st param (see if it is "HELP")
	U8		i;						// temp
	signed char	si;						// signed temp
	U8		j;						// temp
	U16		k;						// U16 temp
	S16		sii;					// S16 temp
	uint32_t ii;					// u32 temp
	int32_t jj;						// s32 temp
	int32_t ss;						// s32 temp
//	float	fa;						// float temps
//	float	fb;
//	float	fc;
//	float	fd;


	bchar = '\0';																// clear global escape
    if (nargs > 0){
/*		for(i = 0; i <= nargs; i++){											// upcase all args
			s = args[i];
			str_toupper(s);
		}*/
		s = args[0];
		str_toupper(s);
		t = args[0];															// point to first arg (cmd)
		cmd_id = cmd_srch(t);													// search for command
		t = args[1];															// point to 2nd arg
		cmd_id1 = cmd_srch(t);													// search for command
		s = args[1];															// point to 2nd arg (1st param)
		if((*s == '?') || (cmd_id1 == help2)){									// see if this is a "HELP HELP" entry
			if((cmd_id == help1) || (cmd_id == help2)){							// if base command is help, then:
				do_help();														// list standard help banner
				puts0("");														// blank line
				for(i = 0; i < lastcmd; i++){									// do command-specific helps for all commands
					if(do_cmd_help(i)) puts0("");
				}
			}else{																// otherwise:
				do_cmd_help(cmd_id);											// do help for cmd only
			}
		}else{
			pm = 0;
			pw = 0;
			c = parm_srch(nargs, args, "-",1);									// capture minus floater
			if(c){
				pm = TRUE;
				nargs--;
			}
			c = parm_srch(nargs, args, "V",0);									// capture v-flag floater
			if(c){
				pv = TRUE;
				nargs--;
			}
/*			c = parm_srch(nargs, args, "C",0);									// capture c-flag floater
			if(c){
				pc = TRUE;
				nargs--;
			}
			c = parm_srch(nargs, args, "?",0);									// capture ?-flag select floater
			if(c){
				pq = TRUE;
				nargs--;
			}*/
			c = parm_srch(nargs, args, "W",0);									// capture w-flag floater
			if(c){
				pw = TRUE;
				nargs--;
			}
			c = parm_srch(nargs, args, "DST", 0);									// capture DST select floater
			if(c){
				dst_select = 1;
				nargs--;
			}
			c = parm_srch(nargs, args, "STD", 0);									// capture DST select floater
			if(c){
				dst_select = 2;
				nargs--;
			}
			gas_gage(2);														// init gas gauge to disabled state
			switch(cmd_id){														// dispatch command
				case help1:
				case help2:														// MAIN HELP
					do_help();														// list standard help banner
					break;

				case vers:														// SW VERSION CMD
					dispSWvers();
					sprintf(obuf,"\ntsip buf = %d%% full",tsip_buf_stat());
					puts0(obuf);
					break;

				case sec:														// trigger sec LED
//					led_runstat(TRUE);
					puts0("Sec trigger");
					break;

				case cal:														// set/read cal value
					if(pw){
						params[0] = 0;
						get_Dargs(1, nargs, args, &params[0]);
						ss = (S32)params[0];
						if(pm){
							ss *= -1;
						}
						set_cal(ss);
						putss("Set ");
					}
					if(pv){
						init_eeprom();
						puts0("eeprom init");
					}
					ss = read_cal();
					sprintf(obuf,"cal (ms/hr): %+d",ss);
					puts0(obuf);
					break;

				case adc_buff:
					putss("adc buf stat: ");
					for(i=0; i<8; i+=2){
						sprintf(obuf," %04x",get_adcbuf(i));
						putss(obuf);
					}
					putss("\nadc buf data: ");
					for(i=1; i<8; i+=2){
						sprintf(obuf," %04u",get_adcbuf(i));
						putss(obuf);
					}
					puts0(" ");
					break;

				case gtest:														// GPS status
					puts0("GPS status:");
					disp_gpstat(obuf);
					puts0(obuf);
					sprintf(obuf,"resync count: %d",get_resync());
					puts0(obuf);
					sprintf(obuf,"Shadow Calendar: %02u/%02u/%04u",get_smon(),get_sday(),get_syear());
					puts0(obuf);
					set_tdebug();												// trigger debug display
					if(isdst()){
						puts0("DST");
					}else{
						puts0("STD");
					}
					sprintf(obuf,"tsip buf = %d%% full",tsip_buf_stat());
					puts0(obuf);
/*					putchar_b('{');												// debug patch to display raw TSIP data
//					initbuf1();
					do{
						sii = getchr1();
						if(sii != 0x100){
							puthex((char)sii);
							putchar_b(' ');
//							putchar1(0xaa);
						}
					}while(bchar != ESC);
					putchar_b('\n');
					putchar_b('\r');*/
					break;

				case bright:
					puts0("dp debug");
					set_sbright(*args[1]);
					puts0(args[1]);
					break;
					// set display brightness
/*					params[0] = 0;
					get_Dargs(1, nargs, args, &params[0]);
					if(pm){
						set_sbright(*args[1]);
						puts0(args[1]);
					}else{
						ii = ((U32)params[0] * (U32)(PWM_ZERO - PWM_MAX)) / 100L;
						ii = (U32)PWM_ZERO - ii;
						PWM1_1_CMPB_R = (U16)ii;
						sprintf(obuf,"Brightness debug: %d",ii);
						puts0(obuf);
					}
					break;*/

				case gpsen:														// set gps enable
					params[0] = 0;
					get_Dargs(1, nargs, args, &params[0]);
					enable_gps((U8)params[0]);
					puts0("GPS cntl");
					break;

				case setime:													// time zone
					params[0] = 0;	//hh
					params[1] = 0;	//mm
					params[2] = 0;	//ss
					params[3] = 0;	//MM
					params[4] = 0;	//DD
					params[5] = 0;	//YYYY
					get_Dargs(1, nargs, args, &params[0]);
					if((params[0] < 24) && (params[1] < 60) && (params[2] < 60)){
						ii = ((S32)params[0] * 3600) + ((S32)params[1] * 60) + (S32)params[2];
						ii *= 1000;
						if((params[3] < 13) && (params[4] < 32) && (params[5] != 0)){
							set_rtc(ii);
							set_sdate((U8)params[3], (U8)params[4], (U16)params[5]);
							k = params[5] / 100;
							j = (U8)(params[5]%100);
							sprintf(obuf,"%02u%02u%02u%02u%02u%02u%02u",params[0],params[1],params[2],params[3],params[4],j,k);
							u8buf[3] = (obuf[0] & 0x0f) << 4;
							u8buf[3] |= (obuf[1] & 0x0f);
							u8buf[2] = (obuf[2] & 0x0f) << 4;
							u8buf[2] |= (obuf[3] & 0x0f);
							u8buf[1] = (obuf[4] & 0x0f) << 4;
							u8buf[1] |= (obuf[5] & 0x0f);
							u8buf[6] = (obuf[6] & 0x0f) << 4;
							u8buf[6] |= (obuf[7] & 0x0f);
							u8buf[4] = (obuf[8] & 0x0f) << 4;
							u8buf[4] |= (obuf[9] & 0x0f);
							u8buf[7] = (obuf[10] & 0x0f) << 4;
							u8buf[7] |= (obuf[11] & 0x0f);
							u8buf[8] = (obuf[12] & 0x0f) << 4;
							u8buf[8] |= (obuf[13] & 0x0f);
							rtc_stop(1);
							rtc_rw(u8buf, XWRITE);
							rtc_stop(0);
							puts0(obuf);
						}
					}
					sprintf(obuf,"RTC (UTC, ms since 00:00): %+d",(S32)get_rtc());
					puts0(obuf);
					rtc_rw(u8buf, XREAD);
					sprintf(obuf,"xRTC: %02u:%02u:%02u  %02u/%02u/%02u%02u",(*(u8buf+3)&0x0f)+(((*(u8buf+3)>>4)&0x0f)*10),(*(u8buf+2)&0x0f)+(((*(u8buf+2)>>4)&0x0f)*10),(*(u8buf+1)&0x0f)+(((*(u8buf+1)>>4)&0x0f)*10),(*(u8buf+6)&0x0f)+(((*(u8buf+6)>>4)&0x0f)*10),(*(u8buf+4)&0x0f)+(((*(u8buf+4)>>4)&0x0f)*10),(*(u8buf+8)&0x0f)+(((*(u8buf+8)>>4)&0x0f)*10),(*(u8buf+7)&0x0f)+(((*(u8buf+7)>>4)&0x0f)*10));
					puts0(obuf);
					break;

				case msg:
					params[0] = 99;
					get_Dargs(1, nargs, args, &params[0]);
					switch(params[0]){
					default:
					case 0:
						puts0("MSG test: press esc");
						set_led_msg(1, args[2], 0);
						while(bchar != ESC);
						set_led_msg(0, "====", 0);
						break;

					case 1:
						puts0("AB: Reset...<esc>");
						set_duon('0');
						while(bchar != ESC);
						bchar = 0;
						set_duon('1');
						wait(255);
						puts0("Set AB... <esc>");
						u8buf[0] = 0xc1;
						u8buf[1] = 0;
						u8buf[2] = 0;
						u8buf[3] = 0xc1;
						send_ab(u8buf, 4);
						u8buf[0] = 9;
						u8buf[1] = 1;
						u8buf[2] = 2;
						u8buf[3] = 3;
						u8buf[4] = 4;
						u8buf[5] = 5;
						send_ab(u8buf, 6);
						while(bchar != ESC);
//						send_ab(&dbuf_ab[0], BUF_AB_MAX);
						break;

					case 2:
						puts0("DE: Reset...<esc>");
						set_duon('0');
						while(bchar != ESC);
						bchar = 0;
						set_duon('1');
						wait(255);
						puts0("Set DE... <esc>");
						u8buf[0] = 0xc1;
						u8buf[1] = 0;
						u8buf[2] = 0;
						u8buf[3] = 0xc1;
						send_de(u8buf, 4);
						u8buf[0] = 0x05;
						u8buf[1] = 0x43;
						u8buf[2] = 0x21;
						u8buf[3] = 0x00;
						u8buf[4] = 0x98;
						u8buf[5] = 0x76;
						send_de(u8buf, 6);
						while(bchar != ESC){
							u8buf[0] = 0xc1;
							u8buf[1] = 0;
							u8buf[2] = 0;
							u8buf[3] = 0xc1;
							send_de(u8buf, 1);
							wait(100);
						}
						break;

					case 3:
						puts0("Reset MC14489s...");
						set_duon('0');
						wait(255);
						set_duon('1');
						wait(255);
						puts0("done.");
						break;

					case 4:
						puts0("1-clk to DE.");
						send_de_1clk();
					}
					break;

				case timez:														// time zone
					sprintf(obuf,"max: %d; min: %d; mid: %d",(S32)PWM_MAX,(S32)PWM_ZERO,(S32)PWM_MID);
					puts0(obuf);

					params[0] = get_timez();
					get_Dargs(1, nargs, args, &params[0]);
					sii = (S16)params[0];
					if(pm){
						sii *= -1;
					}
					set_timez(sii);
					sprintf(obuf,"Time Zone, UTC%+d",(S32)get_timez());
					puts0(obuf);
					break;

				case timer_tst:
					puts0("Timer test (esc to exit)...");
					do{
//						GPIO_PORTE_DATA_R ^= PORTE_SPR4;
						wait(100);
					}while(bchar != ESC);
					break;

				case log_dat:													// log data to com port
					// display log data:  C ?
					disp_esc(TRUE);												// display "press esc to exit"
					k = 1;
					do{
						jj = get_rtc();											// get minutes since midnight
						ii = jj/60000;
						if((jj - (ii*60000) == 0) || k){
							k = 0;
							si = (signed char)(ii/60);							// si = hrs
							j = (U8)(ii - (si*60));								// j = mins
							si += (signed char)get_timez();						// apply time zone correction
							if(si < 0){											// rollover hours
								si += 24;
							}
							if(si >= 24){
								si -= 24;
							}
							sprintf(obuf,"%02d:%02d",si,j);
							puts0(obuf);
							wait(100);
						}
					}while(bchar != ESC);
					break;

				case qtest:
					// test command
					puts0("Test date functions:");
					params[0] = 1;		// mm
					params[1] = 1;		// dd
					params[2] = 2000;	// yyyy
					get_Dargs(1, nargs, args, &params[0]);
					if(pv){
						puts0("leapdays since epoch:");
						for(k=2000; k<2404; k++){
							ii = dse(params[1], params[0], k);
							sprintf(obuf,"%04d, %04d",k, ii);
							puts0(obuf);
						}
					}else{
						j = isly(params[2]);
						i = dsny(params[1], params[0], 0xffff);						// deck DOM
						if(i > 366){
							puts0("!! DAY OF MONTH INVALID !!");
						}
						sprintf(obuf,"mm/dd/yyyy = %02d/%02d/%04d",params[0], params[1], params[2]);
						puts0(obuf);
						sprintf(obuf,"dsny = %d",dsny(params[1], params[0], params[2]));
						puts0(obuf);
						ii = dse(params[1], params[0], params[2]);
						sprintf(obuf,"dse = %d",ii);
						puts0(obuf);
						sprintf(obuf,"isly = %d",j);
						puts0(obuf);
						sprintf(obuf,"dow = %d",dow_calc(params[1], params[0], params[2]));
						puts0(obuf);
					}
					break;

				case config_dst:
					// config DST
					params[0] = 0;												// start month
					params[1] = 0;												// nth start sunday
					params[2] = 0;												// end month
					params[3] = 0;												// nth end sunday
					get_Dargs(1, nargs, args, params);							// parse param numerics into params[] array
					for(i=0,j=TRUE;i<4;i++){
						if(params[i] == 0) j = FALSE;							// if any time param == 0, j = FALSE
					}
					if((params[1] > 5) || (params[3] > 5)) j = FALSE;			// invalid week#s
					if((params[0] > 12) || (params[2] > 12)) j = FALSE;			// invalid months
					if(j || (dst_select == 1)){
//						rptr = rtc_ram_io(0);
//						if(!get_rtc(rptr, 0, RAM_start+1)){						// if non-0 params, copy clock data to MCU
//							puts0("RTC RAM ERROR!");
//						}
						if(j){
							set_dst_info((U8)params[0],(U8)params[1],(U8)params[2],(U8)params[3]);
//							rptr[DST_MON] = int_bcd((U8)params[0]);				// init DST start month
//							rptr[DST_SUN] = (U8)params[1];						// init DST start week
//							rptr[STD_MON] = int_bcd((U8)params[2]);				// init DST end month end
//							rptr[STD_SUN] = (U8)params[3];						// init DST end week
							puts0("Config DST accepted.");
						}
						if(dst_select == 1){
							adjust_rtc_local(0);								// adjust for time zone and DST
/*							if(isdst()){										// update DST status
								rptr[DST] = 1;
							}else{
								rptr[DST] = 0;
							}*/
							puts0("DST status applied.");
						}
						wr_ee_dst();
//						ii = chks_ram(rptr);
//						rptr[CHKS_H] = (U8)(ii >> 8);
//						rptr[CHKS_L] = (U8)(ii &0xff);
//						send_rtc_data(&rptr[DST], DST, RAM_start - DST + 1);	// write data to RTC chip
//						wait(RTC_DLY);
					}
					sp = get_dst_info();
//					if(!get_rtc(rptr, 0, RAM_start+1)){							// if non-0 params, copy clock data to MCU
//						puts0("RTC RAM ERROR!");
//					}
					sprintf(obuf,"MON start: %u; Week start: %u; MON end: %u; Week end: %u;",sp[dst_mon],sp[dst_wk],sp[std_mon],sp[std_wk]);
					puts0(obuf);
					break;

				default:
				case lastcmd:													// not valid cmd
					cmd_found = FALSE;
					break;
			}
		}
    }
	if(bchar == ESC) while(gotchr()) getchr();									// if ESC, clear CLI input chrs
	return cmd_found;
}

//=============================================================================
// do_help() displays main help screen
//=============================================================================
void do_help(void){

	puts0("\nGPS Slaved Real Time Clock CMD List:");
	puts0("Syntax: <cmd> <arg1> <arg2> ... args are optional depending on cmd.");
	puts0("\t<arg> order is critical except for floaters.");
	puts0("\"?\" as first <arg> gives cmd help, \"? ?\" lists all cmd help lines. When");
	puts0("selectively entering <args>, use \"-\" for <args> that keep default value.");
	puts0("\"=\" must precede decimal values w/o spaces. Floating <args>: these non-number");
	puts0("<args> can appear anywhere in <arg> list: \"W\" = wait or loop.\n");

//	enum cmd_enum{ gpsen,bright,cal,log_dat,config_dst,adc_buff,gtest,msg,timer_tst,qtest,sec,setime,timez,help1,help2,vers,lastcmd,helpcmd };
	//           {"A\0   B\0    CAL C\0     D\0        DST\0    G\0   M\0 TI\0      Q\0   SEC S\0    Z\0   ?\0   H\0   V\0\xff"};
//	const char cmd_list[] = {"A\0B\0CAL\0LOG\0DST\0DC\0G\0M\0TI\0Q\0SEC\0S\0Z\0?\0H\0V\0\xff"};
	puts0("\tA:gpsen\t\t\tBright");
	puts0("\tCAL\t\t\tLOG data");
	puts0("\tDST: config dst\t\tDC: adc buf");
	puts0("\tGtest\t\t\tMsg");
	puts0("\tTImer tst\t\tQ test");
	puts0("\tSEC\t\t\tS: set time");

	puts0("\ttime Zone\t\tVERSion");
	puts0("\tTImer test\t\tLOG data to term");
	puts0("\nSupports baud rates of 115.2 (default), 57.6, 38.4, 19.2, and 9.6 kb.");
	puts0("Press <Enter> immediately after reset at the desired baud rate to select.\n");
}

//=============================================================================
// do_cmd_help() displays individual cmd help using cmd_id enum
//=============================================================================
char do_cmd_help(U8 cmd_id){

	char c = TRUE;
	
	switch(cmd_id){														// dispatch help line

		case timez:														// Time zone CMD
			// Time Zone
			puts0("Time Zone: Z /(-)n/?");
			puts0("\tEnter time zone (delta UTC)");
			break;

		case timer_tst:													// HOST MEM WR CMD
			// test Timer:  TI ?
			puts0("Timer test: TI ?");
			puts0("\ttoggles PE4 @200ms period, ESC to exit");
			break;

		case log_dat:													// HOST MEM WR CMD
			// log data:  TI ?
			puts0("Clock disp: C ?");
			puts0("\tOutputs clock one line per min.");\
			break;

		default:
			c = FALSE;
			break;
	}
	return c;
}

//=============================================================================
// disp_wait_addr() dead code
//=============================================================================
void disp_wait_addr(char* buf){

	sprintf(buf,"disp_wait_addr = dead code\n");
	puts0(buf);
}

//=============================================================================
// exec_bcmd() dead code
//=============================================================================
void exec_bcmd(char* bcmdbuf_ptr, char* obuf, U16* offset){

}

//***********************//
// bcmd functions follow //
//***********************//

//******************************//
// housekeeping functions follow //
//******************************//


//=============================================================================
// get numeric params from command line args
//	argsrt specifies the first item to be searched and aligns argsrt with params[0]
//	for multi-arg cmds, fields must be entered in order.
//=============================================================================
U8 get_Dargs(U8 argsrt, U8 nargs, char* args[ARG_MAX], U16 params[8]){

	char*	s;
	U16*	ptr1;
	U8		i;
	U8		count = 0;
	U32		temp;

	if(argsrt <= nargs){											// test for start in limit (abort if not)
		for(i = argsrt; i < nargs+1; i++){						// convert strings to values
			s = args[i];										// set pointers to array items
			ptr1 = &params[i - argsrt];
			switch(*s){
				case '-':										// skip if user specified default
				case '\0':										// or if arg is empty
					break;

				default:
					count += sscanf(s,"%d",&temp);				// get decimal value
					*ptr1 = temp;
					break;

				case '$':
					s++;
					count += sscanf(s,"%x",&temp);				// get hex if leading "$
					*ptr1 = temp;
					break;
			}
		}
	}
	return count;
}

//=============================================================================
// search for command keywords, return cmd ID if found
//	uses the cmd_list[] array which is constructed as an array of null terminated
//	strings compressed into a single string definition.  Commands are added by
//	placing the minimum required text from the command name with a '\0' terminating
//	null.  cmd_list[] is terminated by an $ff after all of the command names.
//	cmd_enum{} holds an enumerated, named list of all valid commands.  The enum
//	definition must be at the top of this file, so the commented-out version shown
//	below must be copied and pasted to the top of the file whan any changes are
//	made (commands added or deleted).
//
//	Some thought must be put into the order of command names.  Shorter matching
//	names (e.g., single chr entries) must follow longer names to allow the algortihm
//	to properly trap the shorter cmd name.
//	
//=============================================================================
cmd_type cmd_srch(char* string){
// dummy end of list, used to break out of search loop
const char end_list[] = {0xff};
// list of minimum length command words. cmd words are separated by '\0' and list terminated with 0xff
const char cmd_list[] = {"A\0B\0CAL\0C\0DST\0DC\0G\0M\0TI\0Q\0SEC\0S\0Z\0?\0H\0V\0\xff"};
//enum cmd_enum{ cal,log_dat,gtest,timer_tst,qtest,sec,setime,timez,help1,help2,vers,lastcmd,helpcmd };

//!!! make changes to cmd_enum here, move them to top of file, then un-comment !!!

	char*	ptr;							// temp ptr
	char	cmdid = 0;						// start at beginning of cmd_enum
	char	i;								// temp
	char	found = FALSE;					// cmd found flag (default to not found)

	ptr = (char*)cmd_list;												// start at beginning of serach list
	while((*ptr & 0x80) != 0x80){								// process until 0xff found in search list
		i = strncmp(string, ptr, strlen(ptr));					// inbound string match search list?
		if(i){
			cmdid++;											// no, advance to next cmdid 
			while(*ptr++);										// skip to next item in search list
		}else{
			ptr = (char*)end_list;										// found match,
			found = TRUE;										// set break-out criteria
		}
	}
	if(!found) cmdid = lastcmd;									// not found, set error cmd id
	return cmdid;
}

//=============================================================================
// parm_srch() looks for a match of parm_str in any non-empty args[] strings
//	if found, remove the args entry from param list and return 1st chr of parm_str,
//	else return '\0'
//=============================================================================
char parm_srch(U8 nargs, char* args[ARG_MAX], char* parm_str, U8 force_len){

	U8		i;								// counter temp
	char	c = '\0';						// first chr of matched parm_str (first time thru loop, there is no match)
	static char null_str[] = "";			// null string that persists

//	if(nargs > 1){
	    for(i = 1; i <= nargs; i++){						// search starting with first args[] item
			if(c){											// if(c!=null)...
				args[i] = args[i+1];						// if there was a match, move the subsequent pointers down one
			}else{
				if((strlen(parm_str) == strlen(args[i])) || (force_len)){	// in order to match, the lengths have to be equal...
					if(strncmp(args[i], parm_str, strlen(parm_str)) == 0){	// look for match
						c = *parm_str;						// if match, capture 1st chr in matched string
						if(force_len){
							*args[i] = ' ';					// x-out
						}else{
							i--;							// back-up one to edit this item out of the list
						}
					}
				}
			}
	    }
//	}
 	if((c != '\0') && (force_len == 0)){
		args[ARG_MAX - 1] = null_str;							// if there was a match, the last pointer goes to null
		
	}
	return c;													// return first chr in matched string, or null if no match
}

//=============================================================================
// disp_esc() if param true, display "Press esc to exit" msg
//=============================================================================
void disp_esc(char flag){

	if(flag){
		putss("  Press <ESC> to exit.");
	}
	puts0("");
}

//=============================================================================
// convert all chrs in string to upper case
//=============================================================================
void str_toupper(char *string){

    while(*string != '\0'){
        *string++ = toupper(*string);
    }
}

//=============================================================================
// parse string for delimited arguments
//  on exit, the args[] array holds each delimited argument from the command string input:
//  args[0] holds first arg (command)
//  args[1] holds next arg
//  args[2] etc...
//  up to args[ARG_MAX]
//
//  nargs holds number of arguments collected.  i.e., nargs = 3 specifies that args[0] .. args[3]
//      all hold arguments (three total, including the command).
//=============================================================================
int parse_args(char* cmd_string, char* args[ARG_MAX]){
	int i;
	char quote_c = 0;
	static char null_string[2] = "";

    // clear args pointers
    for (i=0; i<ARG_MAX; i++){
        args[i] = null_string;
    }
    i = 0;
    do{
        if(quotespace(*cmd_string, 0)){         // process quotes until end quote or end of string
            quote_c = *cmd_string;              // close quote must = open quote
            args[i++] = ++cmd_string;               // start args with 1st char after quote
            while(!quotespace(*cmd_string,quote_c)){
                if(*cmd_string == '\0'){
                    return i;                   // end of cmd string, exit
                }
                cmd_string++;
            }
            *cmd_string++ = '\0';               // replace end quote with null
        }
        if(*cmd_string == '\0'){
            return i;                           // end of cmd string, exit
        }
        if(!whitespace(*cmd_string)){
            args[i++] = cmd_string++;			// when non-whitespace encountered, assign arg[] pointer
            if(i > ARG_MAX){
                return i;						// until all args used up
            }
            do{
                if(*cmd_string == '\0'){
                    return i;                   // end of cmd string, exit
                }
                if(whitespace(*cmd_string)){
                    *cmd_string = '\0';			// then look for next whitespace and delimit (terminate) the arg[] string
                    break;
                }
                cmd_string++;					// loop until end of cmd_string or next whitespace
            } while (1);
        }
        cmd_string++;							// loop...
    } while (1);
}

//=============================================================================
// parse_ehex() for embeded hex ($$) arguments
//  on exit, the string holds the original text with %xx replaced by a single
//	hex byte.
//=============================================================================
void parse_ehex(char * sptr){
	char* tptr;
	U8	i;

	while(*sptr){
		if((*sptr == '$') && (*(sptr+1) == '$')){
			i = asc_hex(*(sptr+2)) << 4;
			i |= asc_hex(*(sptr+3));
			*sptr++ = i;
			tptr = sptr;
			do{
				*tptr = *(tptr+3);
				tptr++;
			}while(*(tptr+2));
		}else{
			sptr++;
		}
	}
}
//=============================================================================
// test characer for whitespace
//=============================================================================
int whitespace(char c){

    switch (c){					// These are all valid whitespace:
        case '\n':          	// newline
        case '\r':          	// cr
        case '\t':          	// tab
        case 0x20:{         	// space
		case '/':				// slash is also wsp
            return TRUE;
        }
    }
    return FALSE;
}

//=============================================================================
// test characer for quote
//=============================================================================
int quotespace(char c, char qu_c){

    if(qu_c == '\0'){
        switch (c){				// if qu_c is null, these are valid quotes:
            case '\'':          // newline
            case '\"':          // cr
            case '\t':          // tab
                return TRUE;
            }
    } else {
        if(c == qu_c){			// else, only qu_c results in a TRUE match
            return TRUE;
        }
    }
    return FALSE;
}

//=============================================================================
// gas_gage() display up to 16 "*" chrs based on count rate.
//	Gauge appearance:
//	[****************]	all OK
//	[***.............]	errors detected
//
//	"len" cmds:
//	0: process gauge counter/display
//	1: set gauge error character = "."
//	2: disable gage counter/display (set clen = 0)
//	all others: set creset = count = len/16, display initial gauge characters
//	This calculation identifies how many bytes are in 1/16th of the total
//	byte count (len).  For count events (len == 0), this Fn decrements count, &
//	displays a gauge chr when count == 0.  count is then reloaded with creset.
//	process continues until 16 gauge chrs have been displayed.  After this,
//	any further count eventss result in no further change to the display.
//=============================================================================
U8 gas_gage(U16 len){

#define LENCMD_MAX 2		// max # of gas-gage() cmds

	static U16	creset;		// holding reg for data counter reset value
	static U16	count;		// data counter
	static U8	clen;		// gage chr counter
	static U8	gchr;		// gage chr storage
		   U8	c = 0;		// gage printed flag

	if(len <= LENCMD_MAX){
		if(!len && clen){
			if(--count == 0){ 
				putchar(gchr);					// disp gage chr
				count = creset;					// reset loop counters
				clen--;
				if(clen == 0) putchar(']');		// if end of gage, print end bracket
				c = 1;
			}
		}else{
			if(len == 1) gchr = '.';			// if error flag, change gauge chr to err mode
			if(len == 2) clen = 0;				// disable gauge
		}
	}else{
		creset = count = len >> 4;				// init count & count reset (creset) = len/16
		if(creset == 0) creset = 1;				// if overall length too short, set 1:1
		clen = 16;								// 16 gage chrs max
		gchr = '*';								// set * as gage chr
		putchar('[');							// print start bracket
		for(c = 0; c < 16; c++) putchar(' ');
		putchar(']');							// place end bracket for scale
		for(c = 0; c < 17; c++) putchar('\b');	// backspace to start of scale
		c = 1;
	}
	return c;
}

//=============================================================================
// log_error_byte() places error data into log buffer.  Log format is:
//	(device) (host) (addrH) (addrL).  Called by target verify fns to allow
//	a limited number of errors to be trapped (limit is the buffer used to
//	hold the error log).
//	returns updated pointer to next available log entry
//=============================================================================
U8* log_error_byte(U8* lbuf, U8 d, U8 h, U16 a){

	*lbuf++ = d;								// store device data
	*lbuf++ = h;								// store host data
	*lbuf++ = (U8)(a >> 8);						// store addr
	*lbuf++ = (U8)(a & 0xff);
	return lbuf;								// return updated pointer
}

//=============================================================================
// disp_error_log() displays errors logged into error string.  Log format is:
//	(device) (host) (addrH) (addrL)
//	Display format is:
//	nn: Dev ($xx) != $xx @$xxxx\n = 28 printed chrs
//	nn = err number (ordinal)
//	xx = data bytes
//	xxxx = error address
//=============================================================================
void disp_error_log(U8* lbuf, U16 len){

	char obuf[32];				// local buffer
	// use U16 type to simplify sprintf variable list
	U16  i;						// loop counter
	U16  d;						// device data
	U16  h;						// host data
	U16  a;						// addr

	len++;										// add 1 to end so that we can start loop at "1"
	for(i = 1; i < len; i++){					// loop from 1 to len+1 entries
		d = (U16)*lbuf++ & 0xff;				// format device data
		h = (U16)*lbuf++ & 0xff;				// format host data
		a = ((U16)*lbuf++ & 0xff) << 8;			// format addr
		a |= (U16)*lbuf++ & 0xff;
		sprintf(obuf,"%02u: Dev ($%02x) != $%02x @$%04x", i, d, h, a); // display err line
		puts0(obuf);
	}
}

//=============================================================================
// bcmd_resp_init() inits bcmd_resp_ptr
//=============================================================================
void bcmd_resp_init(void){

	bcmd_resp_ptr = bcmd_resp_buf;
	*bcmd_resp_ptr = '\0';
}

//=============================================================================
// asc_hex() converts ascii chr to 4-bit hex.  Returns 0xff if error
//=============================================================================
U8 asc_hex(S8 c){

	U8 i;

	if((c >= '0') && (c <= '9')){			// if decimal digit,
		i = (U8)(c - '0');					// subtract ASCII '0' to get hex nybble
	}else{
		if((c >= 'A') && (c <= 'F')){		// if hex digit,
			i = (U8)(c - 'A' + 0x0A);		// subtract ASCII 'A', then add 0x0A to get hex nybble
		}else{
			i = 0xff;						// if not valid hex digit, set error return
		}
	}
	return i;	
}

//=============================================================================
// temp_float() converts MCP9800 binary temp to a float (degrees C)
//=============================================================================
float temp_float(U16 k){
	U8		i;			// temp
	U8		j = 0;		// temp sign
	float	fa;			// temp float

	if(k & 0x8000){												// if negative,
		j = 1;													// preserve sign and
		k = ~k + 1;												// convert value to positive
	}
	i = k >> 8;													// get integer portion
	fa = (float)i;												// convert to float
	if(k & 0x0080) fa += 0.5;									// add fractional portion
	if(k & 0x0040) fa += 0.25;
	if(k & 0x0020) fa += 0.125;
	if(k & 0x0010) fa += 0.0625;
	if(j){														// if negative, convert
		fa *= -1;
	}
	return fa;
}

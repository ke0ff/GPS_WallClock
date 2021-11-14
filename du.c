/*************************************************************************
 *********** COPYRIGHT (c) 2018 by Joseph Haas (DBA FF Systems)  *********
 *
 *  File name: du.c
 *
 *  Module:    Control
 *
 *  Summary:   This is the DU driver code file for the FF89UX ICOM UX DU application
 *  			adapted for use as a GPS aux status display.  Only 7-seg digits are used
 *  			on CSA and CSB (CSC is not used).  Onlt the SFn discrete LED is populated.
 *  			All bar-tapes and other discrete LEDs are not pop'd.
 *
 *  			This version of the UX-LEDU also does not populate the SiLabs MCU.  Instead,
 *  			the Tiva MCU for the GPS Wall Clock directly drives the SPI inputs of the
 *  			LED drivers.
 *
 *  Project scope declarations revision history:
 *    11-24-18 jmh:  Rev 0.0:
 *                   Copied from LEDU project
 *    09-23-12 jmh:  Rev 0.0:
 *                   Initial project code
 *
 *******************************************************************/

/********************************************************************
 *  File scope declarations revision history:
 *    07-25-21 jmh:  Debugging the DE driver issue where the displays for the D & E devices are "garbled".  This does not seem to be a SW issue.
 *    					Reset does not cure the issue, and no SW techniques could be devised (tried command line debug and also a "1=clk" technique
 *    					to try to reset the internal logic of the MC14489, nothing worked).  The internal logic of the MC14489 DE chain seems to have
 *    					been corrupted by either a power or a clock glitch.  Nothing save a power-cycle (more than a second was needed) would correct
 *    					the issue.
 *    12-27-18 jmh:  Modified set_sbright(U8 c) to also control DP bits
 *	  11-24-18 jmh:  Removed bar, and discrete message parsers.  Edited for CCS6 syntax.
 *	  01-27-17 jmh:  Got initialization code working.  Each display chip now has
 *						mirror registers that the system updates.  Then, the
 *						mirror-regs are sent to the respective devices.
 *					 Added parsing Fns to update mirror-regs and send updates
 *						to devices for display
 *					 Need to establish serial command interface and hook into parsing Fns
 *					 Need update Fns for PL status LEDs
 *					 added brightness control: High (BRIGHT = 1 and device bright bits set)
 *											    Med (BRIGHT = 1 and device bright bits clear)
 *											    Low (BRIGHT = 0 and device bright bits clear)
 *    09-23-12 jmh:  creation date
 *
 *******************************************************************/

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
// compile defines
#define DEBUG ON							// enable debug code
//#define F360DK ON

#include "inc/tm4c123gh6pm.h"
#include "du.h"
#include "init.h"
#include "typedef.h"
//#include "C8051F360.h"
#include "stdio.h"
//#include "serial.h"

//-----------------------------------------------------------------------------
// Definitions
//-----------------------------------------------------------------------------

//	see du.h for #defines

//-----------------------------------------------------------------------------
// Local variables
//-----------------------------------------------------------------------------
// Processor I/O assignments
/*
//P0
sbit SCK          = P0^0;			// in: SPI mosi
sbit TXD0         = P0^1;			// out: RS-232 tx data
sbit RXD0         = P0^2;			// in: RS-232 rx data
sbit MISO         = P0^7;			// out: SPI miso

//P1
sbit MOSI         = P1^0;			// in: SPI mosi
sbit SDA          = P1^1;			// i/o: I2C
sbit SCL          = P1^2;			// i/o: I2C
sbit CSAB          = P1^3;			// out: MC14489 chip selects	!!! NOTE: schem shows inverters, but AND gates !!!
sbit CSDE          = P1^4;			// out: MC14489 chip selects	!!! are installed so the SW must treat these   !!!
sbit CSC          = P1^5;			// out: MC14489 chip selects	!!! I/Os as active low.                        !!!
sbit CSD          = P1^6;			// out: expansion chip select
sbit BRIGHT       = P1^7;			// out: MC14489 LED bright/dim select
*/
#define SPILEN 10					// queued spi buff length
U8	spibuf[SPILEN];					// spi tx buffer
U8*	spiptr;							// spi pointer
U8*	spiend;							// end of puffer pointer
U8	spi_flags;						// qued-spi flags
U8*	spi_pointer;					// pointer for SPI intr
#define	BUF_AB_MAX	6
U8	dbuf_ab[BUF_AB_MAX];			// A/B data registers
#define	CBUF_AB_MAX	4
U8	cbuf_ab[CBUF_AB_MAX];			// A/B config registers
//#define	BUF_C_MAX	3
//static volatile U8	dbuf_c[BUF_C_MAX];				// C data registers
//#define	CBUF_C_MAX	1
//static volatile U8	cbuf_c[CBUF_C_MAX];				// C config registers
#define	BUF_DE_MAX	6
U8	dbuf_de[BUF_DE_MAX];			// D/E data registers
#define	CBUF_DE_MAX	4
U8	cbuf_de[CBUF_DE_MAX];			// D/E config registers
U8	dpb_a;							// A chip bright & dp bit reg
U8	dpb_b;							// B chip bright & dp bit reg
//U8	dpb_c;							// C chip bright & dp bit reg
U8	dpb_d;							// D chip bright & dp bit reg
U8	dpb_e;							// E chip bright & dp bit reg
U8	spi_temp;

//-----------------------------------------------------------------------------
// Local constants
//-----------------------------------------------------------------------------
// MC14489 init strings...control reg inits
//	DEV A: b5: BCD/nodec		 (G)
//		   b4: BCD/nodec		 (M)
//		   b3-1: BCD/SPdecode	 (MMK)
//		   h1: sp fn led
//		   h2: dp (m)
//	DEV B: b5-1: BCD/SPdecode   (KKHHH)
//		   h1: MTX
//		   h2: MRX
//							  devB            devA
U8 cinit_ab[] = {SPECIAL_123|SPECIAL_45|DISP_ENABLE,0,0,SPECIAL_123|DISP_ENABLE};		// IPL config AB
U8 init_ab[] = {0x78,0x88,0x88,0x78,0x88,0x88};		// IPL AB (all on)
U8 cinit0_ab[] = {ALT_ALL,0,0,ALT_ALL};				// config all off
U8 init0_ab[] = {0,0,0,0,0,0}; 						// all off
U8 cinit1_ab[] = {0xc1,0,0,0xc1};					// stored IPL data
U8 init1_ab[] = {0,0,0,0x20,0,0}; 					// stored IPL config (main dp on)

// DEV C: b5-4: nodec, main bar
//		  b3-2: nodec, sub  bar
//		  b1: bit[1:0] = 1 (subG)
//			  bit2 = sub-
//		      bit3 = decon
//		  h1: PLdet
//		  h2: encon
/*code const U8 cinit_c[] = {ALT_1|ALT_2|ALT_3|ALT_4|ALT_5|DISP_ENABLE};		// IPL config
code const U8 init_c[] = {0x7f,0xff,0xff};						// IPL all on
code const U8 cinit0_c[] = {(ALT_5|ALT_4|ALT_3|ALT_2|ALT_1|DISP_ENABLE)}; // all off config
code const U8 init0_c[] = {0,0,0};								// all off
code const U8 cinit1_c[] = {(ALT_5|ALT_4|ALT_3|ALT_2|ALT_1|DISP_ENABLE)}; // stored IPL data
code const U8 init1_c[] = {0,0,0};								// stored IPL config*/

//	DEV D: b5-1: BCD/SPdecod3 (sub MMMKK)
//		   h1: SRX
//		   h3: dp (s)
//	DEV E: b5: BCD/SPdecode   (sub K)
//		   b4-1: BCD/SPdecode (PL freq hhh.h)
//		   h1: STX
//		   h2: dp (PLfrq)
//							devE         devD
U8 cinit_de[] = {SPECIAL_123|SPECIAL_45|DISP_ENABLE,0,0,SPECIAL_123|SPECIAL_45|DISP_ENABLE};				// IPL config
U8 init_de[] = {0x78,0x88,0x88,0x78,0x88,0x88};		// IPL data
U8 cinit0_de[] = {ALT_ALL,0,0,ALT_ALL};				// all off config
U8 init0_de[] = {0,0,0,0,0,0};						// all off
U8 cinit1_de[] = {ALT_ALL,0,0,ALT_ALL};				// stored IPL data
U8 init1_de[] = {0,0,0,0x30,0,0};					// stored IPL config (sub dp on)

U8 cinit_duoff[] = {0,0,0,0};						// turn off display
U8 cinit_duon[] = {1,0,0,1};						// turn o1 display

//-----------------------------------------------------------------------------
// Local Prototypes
//-----------------------------------------------------------------------------

U8 du_asc_hex(U8 chr);
void spi_1clk(void);

// **************************************************
//  ****************** DU Drivers ******************
// **************************************************

//-----------------------------------------------------------------------------
// init_du()
//	initializes the MC14489 displays and puts up lamp test for 1000ms
//
//-----------------------------------------------------------------------------
void init_du(void){
	U8	i;			// temp
	
	spiptr = 0;								// spi pointer
	spiend = 0;								// end of puffer pointer
	spi_flags = SPI_DONE;					// qued-spi flags
//	CSC = 1;
//	CSD = 1;
//	BRIGHT = 0;								// start in DIM
	init_spi();								// init SPI interface
	wait(50);
	set_duon('0');									// reset MC14489 chains
	wait(50);
	set_duon('1');
	wait(50);


	send_ab(&cinit_ab[0], CBUF_AB_MAX);		// init MC14489s
	send_ab(&init_ab[0], BUF_AB_MAX);		// all lamps on
//	send_c(&cinit_c[0], CBUF_C_MAX);
//	send_c(&init_c[0], BUF_C_MAX);
	send_de(&cinit_de[0], CBUF_DE_MAX);
	send_de(&init_de[0], BUF_DE_MAX);
	wait(SEC3);								// wait for the lamp test to be recognized...
	send_ab(&cinit0_ab[0], CBUF_AB_MAX);	// init MC14489s
	send_ab(&init0_ab[0], BUF_AB_MAX);		// all lamps off
//	send_c(&cinit0_c[0], CBUF_C_MAX);
//	send_c(&init0_c[0], BUF_C_MAX);
	send_de(&cinit0_de[0], CBUF_DE_MAX);
	send_de(&init0_de[0], BUF_DE_MAX);
	wait(MS50);
	for(i=0; i<BUF_AB_MAX; i++){			// init mirror registers
		dbuf_ab[i] = init1_ab[i];
	}
//	for(i=0; i<BUF_C_MAX; i++){
//		dbuf_c[i] = init1_c[i];
//	}
	for(i=0; i<BUF_DE_MAX; i++){
		dbuf_de[i] = init1_de[i];
	}
	for(i=0; i<CBUF_AB_MAX; i++){
		cbuf_ab[i] = cinit1_ab[i];
	}
//	for(i=0; i<CBUF_C_MAX; i++){
//		cbuf_c[i] = cinit1_c[i];
//	}
	for(i=0; i<CBUF_DE_MAX; i++){
		cbuf_de[i] = cinit1_de[i];
	}
}  // end init_du()

//-----------------------------------------------------------------------------
// send_ab()
//	sends SPI buffer to devices a & b
//
//-----------------------------------------------------------------------------
void send_ab(U8* qstr_ptr, U8 offs){

// 	while((get_spi_flags() & SPI_SEND) == SPI_SEND);	// wait for SPI to free up
	spiend = qstr_ptr + offs;
	GPIO_PORTD_DATA_R |= DU_CSAB;
//	CSAB = 0;											// set AB chip select
	spi_start(qstr_ptr);									// start transfer
}  // end send_ab()

//-----------------------------------------------------------------------------
// send_c()
//	sends SPI buffer to device c
//
//-----------------------------------------------------------------------------
/*void send_c(U8* str_ptr, U8 offs){

 	while((get_spi_flags() & SPI_SEND) == SPI_SEND);	// wait for SPI to free up
	spiend = str_ptr + offs;
	CSC = 0;											// set C chip select
	spi_start(str_ptr);									// start transfer
}  // end send_c()*/

//-----------------------------------------------------------------------------
// send_de()
//	sends SPI buffer to devices d & e
//
//-----------------------------------------------------------------------------
void send_de(U8* str_ptr, U8 offs){

// 	while((get_spi_flags() & SPI_SEND) == SPI_SEND);	// wait for SPI to free up
	spiend = str_ptr + offs;
	GPIO_PORTD_DATA_R |= DU_CSDE;
//	CSDE = 0;											// set DE chip select
	spi_start(str_ptr);									// start transfer
}  // end send_de()

//-----------------------------------------------------------------------------
// send_de_1clk()
//	sends one clock to the DE devices
//
//-----------------------------------------------------------------------------
void send_de_1clk(void){

	GPIO_PORTD_DATA_R |= DU_CSDE;
	spi_1clk();											// one ping only, mister vi
	GPIO_PORTD_DATA_R &= ~(DU_CSAB | DU_CSDE);			// turn off chip selects
}  // end send_de()

//-----------------------------------------------------------------------------
// du_asc_hex()
//	converts ascii to hex nybble
//	'0' - '9', and 'A' - 'F' process directly, 'a', 'b', 'd', 'e', 'f' map to upper case
//	':' maps to raised "o" (degrees symbol)
//	<space> = 0x80 (un-mapped chrs also)
//	    '-' = 0x8d
//		'=' = 0x8e
//
//-----------------------------------------------------------------------------
U8 du_asc_hex(U8 chr){
	U8 rtn;				// temp return value

 	switch(chr){
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
			rtn = chr & 0x0f;
			break;

		case 'A':
		case 'B':
		case 'C':
		case 'D':
		case 'E':
		case 'F':
			rtn = chr - 'A' + 10;
			break;

		case 'a':
		case 'b':
		case 'd':
		case 'e':
		case 'f':
			rtn = chr - 'a' + 10;
			break;

		case 'c':
			rtn = 0x81;
			break;

		case 'G':
		case 'g':
			rtn = 0x09;
			break;

		case 'H':
		case 'h':
			rtn = 0x83;
			break;

		case 'I':
		case 'i':
			rtn = 0x01;
			break;

		case 'J':
		case 'j':
			rtn = 0x84;
			break;

		case 'K':
		case 'k':
		case 'X':
		case 'x':
			rtn = 0x82;
			break;

		case 'L':
		case 'l':
			rtn = 0x85;
			break;

		case 'M':
		case 'm':
		case 'N':
		case 'n':
			rtn = 0x86;
			break;

		case 'O':
		case 'o':
			rtn = 0x87;
			break;

		case 'P':
		case 'p':
			rtn = 0x88;
			break;

		case 'Q':
		case 'q':
			rtn = 0x8c;		// y for q... no good sol'n
			break;

		case 'R':
		case 'r':
			rtn = 0x89;
			break;

		case 'S':
		case 's':
			rtn = 0x05;
			break;

		case 'T':
		case 't':
			rtn = 0x07;		// 7 for t... best so far
			break;

		case 'U':
		case 'u':
			rtn = 0x8b;
			break;

		case 'W':
		case 'w':			// v for w... no good sol'n
		case 'V':
		case 'v':
			rtn = 0x8a;
			break;

		case 'y':
		case 'Y':
			rtn = 0x8c;
			break;

		case 'Z':
		case 'z':
			rtn = 0x02;
			break;

		case '-':
			rtn = 0x8d;
			break;
		
		case '=':
			rtn = 0x8e;
			break;

		case ':':
		case ';':
			rtn = 0x8f;
			break;

		default:
		case ' ':
			rtn = 0x80;
			break;
	}
	return rtn;
}  // end du_asc_hex()

//-----------------------------------------------------------------------------
// parse_fmain()
//	parses string into display data stores to BCD reg mirrors and sends to
//		LEDs via SPI
//	string format is {GM MM KK KH HH} (spaces included only for clarity)
//					G..H is any ASCII
//
//-----------------------------------------------------------------------------
void parse_fmain(U8* sptr){
	U8	c;				// local string temps
	U8	d;
	U8	i;				// local temps
	U8	j = 0xc1;

	// process sign and G chrs
	c = *sptr++;						// G chr
	d = du_asc_hex(c);
	if(d & 0x80) j |= ALT_5;
	i = (d & 0x0f);
/*	i = dbuf_ab[3] & 0xf0;				// get lsB of A, mask G & sign
	j = (cbuf_ab[3] & 0xc1);			// configA & mask
	j = 0xc1;
	if((c != ' ') & (d == '1')){
		if(c == '-') i |= 0x2;			// set "-1"
		else i |= 0x0;					// set "+1"
	}else{
		j |= ALT_5;
		if(d == '1'){
			i |= 0x8;					// set "1"
		}else{
			switch(c){
				default:				// set blank
					break;

				case '-':
					i |= 0x1;			// set "+ "
					break;

				case '+':
					i |= 0x7;			// set "- "
					break;
			}
		}
	}*/
	dbuf_ab[3] = (dbuf_ab[3] & 0xf0) | i;						// update mirrors
	c = *sptr++;		// M chr
	d = du_asc_hex(c);
	if(d & 0x80) j |= ALT_4;
	i = (d & 0x0f) << 4;
	c = *sptr++;		// M chr
	d = du_asc_hex(c);
	if(d & 0x80) j |= ALT_3;
	i |= (d & 0x0f);
	dbuf_ab[4] = i;
	c = *sptr++;		// M chr
	d = du_asc_hex(c);
	if(d & 0x80) j |= ALT_2;
	i = (d & 0x0f) << 4;
	c = *sptr++;		// K chr
	d = du_asc_hex(c);
	if(d & 0x80) j |= ALT_1;
	i |= (d & 0x0f);
	dbuf_ab[5] = i;
	cbuf_ab[3] = j;
	
	// B device - get bright/dp reg
	i = dbuf_ab[0] & 0xf0;
	j = (cbuf_ab[0] & 0xc1);
//	j = 0xc1;
	// process K chr
	c = *sptr++;		// K chr
	d = du_asc_hex(c);
	if(d & 0x80) j |= ALT_5;
	i |= d & 0x0f;
	dbuf_ab[0] = i;
	c = *sptr++;		// K chr
	d = du_asc_hex(c);
	if(d & 0x80) j |= ALT_4;
	i = (d & 0x0f) << 4;
	dbuf_ab[1] = i;

	c = *sptr++;		// H chr
	d = du_asc_hex(c);
	if(d & 0x80) j |= ALT_1;
	i = (d & 0x0f);
	
	c = *sptr++;		// H chr
	d = du_asc_hex(c);
	if(d & 0x80) j |= ALT_2;
	i |= (d & 0x0f) << 4;
	dbuf_ab[2] = i;

	i = dbuf_ab[1];
	c = *sptr++;		// H chr
	d = du_asc_hex(c);
	if(d & 0x80) j |= ALT_3;
	i |= (d & 0x0f);
	dbuf_ab[1] = i;
	cbuf_ab[0] = j;
	send_ab(&cbuf_ab[0], CBUF_AB_MAX);
	send_ab(&dbuf_ab[0], BUF_AB_MAX);
}  // end parse_fmain()

//-----------------------------------------------------------------------------
// parse_fsub()
//	parses string into display data stores to BCD reg mirrors and sends to
//		LEDs via SPI
//	string format is {sG MM MK KK} (spaces included only for clarity)
//					s = "+", '-', ' '
//					G is ASCII <space>, 0 or 1 (0 displays as blank)
//					M..H is ASCII 0-9, A-F, <space>, '-', or '='
//
//-----------------------------------------------------------------------------
void parse_fsub(U8* sptr){
	U8	c;				// local string temps
	U8	d;
	U8	i;				// local temps
	U8	j;

	// process sign and G chrs (on C device)
	c = *sptr++;						// sign chr
	d = *sptr++;						// G chr
	i = 0; //dbuf_c[3] & 0xf8;				// get lsB of A, mask G & sign
	if((c != ' ') & (d == '1')){
		if(c == '-') i |= 0x2;			// set "-1"
	}else{
		j = ALT_5;
		if(d == '1'){
			i |= 0x3;					// set "1"
		}else{
			switch(c){
				default:				// set blank
					break;

				case '-':
					i |= 0x4;			// set "+ "
					break;
			}
		}
	}
	//dbuf_c[3] = i;						// update mirrors
	// D device
	i = dbuf_de[3] & 0xf0;
	j = (cbuf_de[3] & 0xc1);			// configD & mask
	c = *sptr++;		// M chr
	d = du_asc_hex(c);
	if(d & 0x80) j |= ALT_5;
	i |= (d & 0x0f);
	dbuf_de[3] = i;

	c = *sptr++;		// M chr
	d = du_asc_hex(c);
	if(d & 0x80) j |= ALT_4;
	i = (d & 0x0f) << 4;
	c = *sptr++;		// M chr
	d = du_asc_hex(c);
	if(d & 0x80) j |= ALT_3;
	i |= (d & 0x0f);
	dbuf_de[4] = i;

	c = *sptr++;		// K chr
	d = du_asc_hex(c);
	if(d & 0x80) j |= ALT_2;
	i = (d & 0x0f) << 4;
	c = *sptr++;		// K chr
	d = du_asc_hex(c);
	if(d & 0x80) j |= ALT_1;
	i |= (d & 0x0f);
	dbuf_de[5] = i;
	cbuf_de[3] = j;
	
	// E device
	i = dbuf_de[0] & 0xf0;
	j = (cbuf_de[0] & (0xc1 | ALT_4 | ALT_3 | ALT_2 | ALT_1));		// configA & mask
	c = *sptr++;		// K chr
	d = du_asc_hex(c);
	if(d & 0x80) j |= ALT_5;
	i |= (d & 0x0f);
	dbuf_de[0] = i;
	cbuf_de[0] = j;
//	send_c(&dbuf_c[0], BUF_C_MAX);
	send_de(&cbuf_de[0], CBUF_DE_MAX);
	send_de(&dbuf_de[0], BUF_DE_MAX);
}  // end parse_fsub()

//-----------------------------------------------------------------------------
// parse_fpl()
//	parses string into display data stores to BCD reg mirrors and sends to
//		LEDs via SPI
//	string format is {sCCCdC} (spaces included only for clarity)
//					C..C = PL freq (4 digits)
//					d is optional decimal point (location only as shown)
//
//-----------------------------------------------------------------------------
void parse_fpl(U8* sptr){
	U8	c;				// local string temps
	U8	d;
	U8	i;				// local temps
	U8	j;

	// E device
	j = (cbuf_de[0] & (0xc1 | ALT_5));		// configA & mask
	c = *sptr++;		// 100's chr
	d = du_asc_hex(c);
	if(d & 0x80) j |= ALT_4;
	i = (d & 0x0f) << 4;
	c = *sptr++;		// 10's chr
	d = du_asc_hex(c);
	if(d & 0x80) j |= ALT_3;
	i |= (d & 0x0f);
	dbuf_de[1] = i;

	c = *sptr++;		// 1's chr
	d = du_asc_hex(c);
	if(d & 0x80) j |= ALT_2;
	i = (d & 0x0f) << 4;

	c = *sptr++;		// 0.1's chr or dp
	dbuf_de[0] &= 0x9f;	// pre-clear dp
	if(c == '.'){		// process dp
		dbuf_de[0] |= 0x20;
		c = *sptr++;	// get 0.1's chr
		if((dbuf_de[0] & 0x30) == 0x30) dbuf_de[0] |= 0x40;
	}
	d = du_asc_hex(c);
	if(d & 0x80) j |= ALT_1;
	i |= (d & 0x0f);
	dbuf_de[2] = i;
	cbuf_de[0] = j;
	send_de(&cbuf_de[0], CBUF_DE_MAX);
	send_de(&dbuf_de[0], BUF_DE_MAX);
}  // end parse_fpl()

//-----------------------------------------------------------------------------
// set_sbright()
//	sets/clears the bright bit in all of the SPI mirrors and performs an update
//		of all display devices (data only)
//	Also modifies "h" control bits to drive DP elements
//	param format is ascii
//	c =
//	"H", "2" or 2 = high bright
//	"M", "1" or 1 = med bright (deprecated)
//	"L", "0" or 0: low bright
//
//	A/a: controls DP in upper string (one position only)
//	B/C/I/i: controls DPs in lower string (B is left, C is right, I is both, i is none)
//	For CTCSS string:
//	D = left, E = mid-left, F = mid-right, G = right, J = all, j = none
//	S/s: controls special Fn LED
//-----------------------------------------------------------------------------
void set_sbright(U8 c){
	U8	i;				// temp

	switch(c){
		case 'H':
		case '2':
		case 2:
			dbuf_ab[0] |= BRIGHTEN;
			dbuf_ab[3] |= BRIGHTEN;
			dbuf_de[0] |= BRIGHTEN;
			dbuf_de[3] |= BRIGHTEN;
			break;

		case 'M':
		case '1':
		case 1:
			dbuf_ab[0] |= BRIGHTEN;
			dbuf_ab[3] |= BRIGHTEN;
			dbuf_de[0] |= BRIGHTEN;
			dbuf_de[3] |= BRIGHTEN;
			break;

		case 'L':
		case '0':
		case 0:
			dbuf_ab[0] &= ~BRIGHTEN;
			dbuf_ab[3] &= ~BRIGHTEN;
			dbuf_de[0] &= ~BRIGHTEN;
			dbuf_de[3] &= ~BRIGHTEN;
			break;
		// DP modifiers:
		case 'A':
			i = dbuf_ab[3] | DP_2;
			if((i & 0x30) == 0x30){
				i |= 0x70;
			}
			dbuf_ab[3] = i;
			break;

		case 'a':
			dbuf_ab[3] &= ~(DP_2 | DP_4);
			break;

		case 'B':
			dbuf_de[3] = (dbuf_de[3] & ~DP_ALL) | DP_4;
			break;

		case 'C':
			dbuf_de[3] = (dbuf_de[3] & ~DP_ALL) | DP_2;
			break;

		case 'I':
			dbuf_de[3] |= DP_ALL;
			break;

		case 'i':
			dbuf_de[3] &= ~DP_ALL;
			break;

		case 'D':
			dbuf_de[0] = (dbuf_de[0] & ~DP_ALL) | DP_4;
			break;

		case 'E':
			dbuf_de[0] = (dbuf_de[0] & ~DP_ALL) | DP_3;
			break;

		case 'F':
			dbuf_de[0] = (dbuf_de[0] & ~DP_ALL) | DP_2;
			break;

		case 'G':
			dbuf_de[0] = (dbuf_de[0] & ~DP_ALL) | DP_1;
			break;

		case 'J':
			dbuf_de[0] |= DP_ALL;
			break;

		case 'j':
			dbuf_de[0] &= ~DP_ALL;
			break;
		// SP Fn LED modifiers:
		case 'S':
			i = dbuf_ab[3] | DP_1;
			if((i & 0x30) == 0x30){
				i |= 0x70;
			}
			dbuf_ab[3] = i;
			break;

		case 's':
			dbuf_ab[3] &= ~(DP_1 | DP_4);
			break;

		default:
			break;
	}
	send_ab(&dbuf_ab[0], BUF_AB_MAX);
	send_de(&dbuf_de[0], BUF_DE_MAX);
}  // end set_sbright()

//-----------------------------------------------------------------------------
// set_duon()
//	sets/clears the device enable bit in all of the SPI mirrors and performs an update
//		of all display devices (data only)
//	param format is ascii
//					c = "1", or "0"
//
//-----------------------------------------------------------------------------
void set_duon(U8 c){
	
	switch(c){
		case '1':
		case 1:
			send_ab(cinit_duon, CBUF_AB_MAX);
			send_de(cinit_duon, CBUF_DE_MAX);
			break;

		default:
		case '0':
		case 0:
			send_ab(cinit_duoff, CBUF_AB_MAX);
			send_de(cinit_duoff, CBUF_DE_MAX);
			break;
	}
}  // end set_duon()

// **************************************************
//  ***************** SPI Drivers ******************
// **************************************************

//-----------------------------------------------------------------------------
// init_spi()
//	resets SPI and inits queued flags & buffer
//
//-----------------------------------------------------------------------------
void init_spi(void){

	spiptr = 0;								// spi pointer
	spiend = 0;								// end of puffer pointer
	spi_flags = SPI_DONE;					// qued-spi flags
//	ESPI0 = 0;								// disable spi intr
//	SPIEN = 0;								// disable SPI
//	SPIF = 0;
//	SPI0CFG = 0x40;
//	SPIEN = 1;								// re-enable SPI
	GPIO_PORTD_DATA_R &= ~(DU_CSAB | DU_CSDE); // init chip selects
	GPIO_PORTA_DATA_R &= ~(DU_MOSI | DU_SPCK); // init clk & data
}  // end init_spi()

//-----------------------------------------------------------------------------
// get_spi_flags()
//	returns value of spi_flags
//
//-----------------------------------------------------------------------------
U8 get_spi_flags(void){

	return spi_flags;
}  // end get_spi_flags()

//-----------------------------------------------------------------------------
// put_spi()
//	places data into spi buffer	at passed pointer addr
//	returns new pointer value													  
//-----------------------------------------------------------------------------
/*
U8* put_spi(U8* str_ptr, U8 spidata){

	*str_ptr++ = 0x80 | (spidata >> 4);			// nybble un-pack data into string
	*str_ptr++ = 0x80 | (spidata & 0x0f);
	*str_ptr = '\0';							// keep a running EOS
	return str_ptr;
}  // end put_spi()
*/

/****************
 * send_spi does bit-bang SPI
 */
uint8_t send_spi(uint8_t data)
{
	uint8_t	i;
	uint8_t di = 0;

#define	BIT_DELAY 8

	for(i=0x80;i;i>>= 1){
		if(i & data) GPIO_PORTA_DATA_R |= DU_MOSI;		// set MOSI
		else GPIO_PORTA_DATA_R &= ~(DU_MOSI);
		wait2(BIT_DELAY);								// delay bit_time/4
		GPIO_PORTA_DATA_R |= DU_SPCK;					// set SCK
		wait2(BIT_DELAY);								// delay bit_time/4
		if(GPIO_PORTA_DATA_R & DU_MISO) di |= i;		// read MISO
		wait2(BIT_DELAY);								// delay bit_time/4
		GPIO_PORTA_DATA_R &= ~(DU_SPCK);				// clear SCK
		wait2(BIT_DELAY);								// delay bit_time/4
	}
	return di;
}
//-----------------------------------------------------------------------------
// spi_start()
//	sends buffered SPI message
//
//-----------------------------------------------------------------------------
void spi_start(U8* str_ptr){

	spi_pointer = str_ptr;							// init pointer
	for(;spi_pointer != spiend;){
		send_spi(*spi_pointer++);
	}
	spi_flags |= SPI_DONE;							//	set done, not sending
	spi_flags &= ~SPI_SEND;
	GPIO_PORTD_DATA_R &= ~(DU_CSAB | DU_CSDE);		// turn off chip selects
	wait2(10);										// delay bit_time/2
	return;
}  // end spi_start()

//-----------------------------------------------------------------------------
// spi_start()
//	sends a single clock with 0 data
//
//-----------------------------------------------------------------------------
void spi_1clk(void){

	GPIO_PORTA_DATA_R &= ~(DU_MOSI);
	wait2(BIT_DELAY);								// delay bit_time/4
	GPIO_PORTA_DATA_R |= DU_SPCK;					// set SCK
	wait2(BIT_DELAY);								// delay bit_time/4
	wait2(BIT_DELAY);								// delay bit_time/4
	GPIO_PORTA_DATA_R &= ~(DU_SPCK);				// clear SCK
	wait2(BIT_DELAY);								// delay bit_time/4
}  // end spi_1clk()

//-----------------------------------------------------------------------------
// spi_start()
//	starts interrupt driven queued spi send session
//
//-----------------------------------------------------------------------------
/*void spi_start(U8* str_ptr){

	SPIF = 0;										// clear spif flag
	spi_flags |= SPI_SEND;							// set flags for send
	spi_flags &= ~(SPI_DONE | SPI_ERR);
	spi_pointer = str_ptr;							// init pointer
	ESPI0 = 1;										// enable interrupts
	spi_temp = *spi_pointer++;
	SPI0DAT = spi_temp;				// send next byte from buffer
//	SPI0DAT = *spi_pointer++;						// send next byte from buffer
}  // end spi_start()

//-----------------------------------------------------------------------------
// spi_isr()
//	Clocks out DU buffer to MC14489 parts
//
//-----------------------------------------------------------------------------
void spi_isr(void) interrupt 6
{
	volatile U8	i = SPI0DAT; // clear rxdata buff
	
	if(spi_pointer == spiend){			// if end of buffer:
		ESPI0 = 0;						//	disable interrupt
		spi_flags |= SPI_DONE;			//	set done, not sending
		spi_flags &= ~SPI_SEND;
		CSAB = 1;						// turn off chip selects
		CSDE = 1;
		CSC = 1;
	}else{
		spi_temp = *spi_pointer++;
		SPI0DAT = spi_temp;				// send next byte from buffer
	}
	SPIF = 0;
}  // end spi_isr()
*/

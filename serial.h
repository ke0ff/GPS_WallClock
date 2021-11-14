/********************************************************************
 ************ COPYRIGHT (c) 2018 by ke0ff, Taylor, TX   *************
 *
 *  File name: serial.h
 *
 *  Module:    Control
 *
 *  Summary:   This is the header file for serial I/O.
 *
 *******************************************************************/


/********************************************************************
 *  File scope declarations revision history:
 *    07-30-14 jmh:  creation date
 *
 *******************************************************************/

//------------------------------------------------------------------------------
// extern defines
//------------------------------------------------------------------------------

#define	CHR_DLY0	87					// max char length, UART0, (us) (= 10/baud)
#define	CHR_DLY1	87					// max char length, UART1, (us) (= 10/baud)
#define	CHR_DLY2	521					// max char length, UART2, (us) (= 10/baud)
#define	CHR_WAT0	6					// max delay to wait for UART0 TX to clear (ms)
#define	CHR_WAT1	2					// max delay to wait for UART1 TX to clear (ms)
#define	CHR_WAT2	3					// max delay to wait for UART2 TX to clear (ms)

#define RXD_ERR			0x01			// rx data error
#define RXD_CHAR		0x08			// rx char ready
#define TXD_CHRS		0x01			// tx chr sent
#define TXD_ERR			0x10			// tx data error (buffer overflow)
#define TXD_CERR		0x20			// tx data error (collision)
#define TXD_JAMMER		0x40			// jammer code detect
#define TXD_SND			0x80			// tx data sending
#define SOH		0x01
#define ETX		0x03
#define	EOT		0x04
#define AKN		0x06
#define DLE		0x10
#define XON		0x11
#define XOFF	0x13
#define NAK		0x15
#define CAN		0x18
#define ESC		27
#define SW_ESC	0xa5					// sw restart escape code
#define	myEOF	26

#define TI1		0x02					// SCON1 bitmaps
#define	RI1		0x01

// xmodem defines
#define	XPOLY	0x1021					// xmodem polynomial (the CRC default)

// RX state commands
#define	RX_STATE_PROC	0x00			// state command to process state
#define	RX_STATE_INIT	0x20			// state command to set initialization state & init vars
#define	RX_STATE_PACK	0x21			// state command to terminate reception
#define	RX_STATE_CLEAR	0x22			// state command to clear xmode
#define	RX_STATE_QERY	0x23			// state command to querry data ready
// RX status responses
#define	XMOD_NULL		0x00			// nothing to report
#define	XMOD_DR			0x01			// xmodem data buffer ready
#define	XMOD_ABORT		0x7f			// xmodem error (aborted)
#define	XMOD_PKAK		0x02			// packet ack processed, seek next packet
#define	XMOD_ERR		0x03			// transmitter done sending (CAN)
#define	XMOD_DONE		0x04			// transmitter done sending
// TX state commands
#define	TX_STATE_PROC	0x00			// cmd to process TX data
#define	TX_STATE_INIT	0x20			// cmd to start a TX session
#define	TX_STATE_CLEAR	0x22			// cmd to do IPL
#define	TX_STATE_LAST	0x21			// cmd to signal end of tx

// Timer and retry limits
#define	XM_TO			(3*ONESEC)		// 3 seconds for xmodem timeout
#define ACKTRIES		10				// # tries to do start of packet before abort
										// this gives 30 sec to start an xmodem transfer in CRC before
										// SW reverts to checksum

//------------------------------------------------------------------------------
// public Function Prototypes
//------------------------------------------------------------------------------

char process_xrx(U8 state_cmd);
char process_xtx(char dc, U8 state_cmd);
void initserial(void);
char putchar0(char c);
char putdch(char c);
U8 puthex(U8 dhex);
char getchr(void);
char gotchr(void);
int putss(const char *string);
int puts0(const char *string);
int putsN(const char *string);
char getch00(void);
//char xmode_off(void);
U16 calcrc(char c, U16 oldcrc, U16 poly);
char is_xmode(void);
char putchar_b(char c);
U32 init_uart0(U32 baud);
char set_baud(U32 baud);

void rxd_intr(void);

U32 init_uart1(U32 baud);
U32 config_uart1(U32 baud);
S16 getchr1(void);
char putchar1(char c);
void initbuf1(void);
U8 tsip_buf_stat(void);

void rxd1_intr(void);

/*************************************************************************
 *********** COPYRIGHT (c) 2018 by Joseph Haas (DBA FF Systems)  *********
 *
 *  File name: init.h
 *
 *  Module:    Control
 *
 *  Summary:   This is the header file for du.c
 *
 *******************************************************************/


/********************************************************************
 *  File scope declarations revision history:
 *    10-27-18 jmh:  creation date
 *
 *******************************************************************/

#include "typedef.h"
#include <stdint.h>

#ifndef DU_H
#define DU_H
#endif

typedef struct {
	U8* ptr;						// du string pointer
	U8	len;						// du string length
} duptr_struct;

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------

// qued-spi flags (spi_flags)
#define SPI_SEND 0x80						// spi buffer send in progress
#define SPI_DONE 0x40						// spi buffer send done
#define SPI_ERR  0x20						// queued-spi error

//MC14489 LED driver control register defines
//	segment data	hex decode		special decode
//       0				0			  <blank>
//       1              1                c
//       2              2                H
//       3              3                h
//       4              4                J
//       5              5                L
//       6              6                n
//       7              7                o
//       8              8                P
//       9              9                r
//       a              A                U
//       b              b                u
//       c              C                y
//       d              d                -
//       e              E                =
//       f              F                o (raised)

#define SPECIAL_123		0x40			// enables special decode for banks 1-3
#define SPECIAL_45		0x80			// enables special decode for banks 4 & 5
#define ALT_1			0x02			// alt selects special decode chrs or no decode (lamps on segments a-d)
#define ALT_2			0x04			// if no alt, hex decode is selected
#define ALT_3			0x08
#define ALT_4			0x10
#define ALT_5			0x20
#define DISP_ENABLE		0x01			// cycle low_high to reset chip
#define	ALT_ALL			(SPECIAL_123|SPECIAL_45|ALT_1|ALT_2|ALT_3|ALT_4|ALT_5|DISP_ENABLE)
// dp controls (1st data byte shifted out)
#define BRIGHTEN		0x80			// 1= set bright mode
#define DP_MASK			0x8F			// mask off dp control bits
										// mask off dp control  bits to disable dp
#define DP_1			0x10			// DP on in bank 1
#define DP_2			0x20			// DP on in bank 2
#define DP_3			0x30			// DP on in bank 3
#define DP_4			0x40			// DP on in bank 4
#define DP_5			0x50			// DP on in bank 5
#define DP_12			0x60			// DP on in banks 1 & 2
#define	DP_ALL			0x70			// DP on in all banks
//ALT mode characters
#define BLANK_CHR		0x00			// enabled if ALT and SPECIAL bits are active
#define c_CHR			0x01
#define DASH_CHR		0x0D
#define DDASH_CHR		0x0E

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Prototypes
//-----------------------------------------------------------------------------

void init_du(void);
void send_ab(U8* qstr_ptr, U8 offs);
//void send_c(U8* str_ptr, U8 offs);
void send_de(U8* str_ptr, U8 offs);
void parse_fmain(U8* sptr);
//void parse_trmain(U8* sptr);
//void parse_sfn(U8* sptr);
void parse_fsub(U8* sptr);
//void parse_trsub(U8* sptr);
void parse_fpl(U8* sptr);
//void parse_barmain(U8* sptr);
//void parse_barsub(U8* sptr);
void set_sbright(U8 c);
void set_duon(U8 c);
void parse_statpl(U8* sptr);
void send_de_1clk(void);

void init_spi(void);
U8 get_spi_flags(void);
//U8* put_spi(U8* str_ptr, U8 spidata);
void spi_start(U8* str_ptr);
uint8_t send_spi(uint8_t data);

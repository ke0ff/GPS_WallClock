/********************************************************************
 ************ COPYRIGHT (c) 2018 by ke0ff, Taylor, TX   *************
 *
 *  File name: tiva_init.h
 *
 *  Module:    Control
 *
 *  Summary:   defines and global declarations for tiva_init.c
 *
 *******************************************************************/

#include "typedef.h"

#ifndef TIVA_INIT_H_
#define TIVA_INIT_H_
#endif /* TIVA_INIT_H_ */

//-----------------------------------------------------------------------------
// defines
//-----------------------------------------------------------------------------

#define PORTF SYSCTL_RCGCGPIO_R5
#define PORTE SYSCTL_RCGCGPIO_R4
#define PORTD SYSCTL_RCGCGPIO_R3
#define PORTC SYSCTL_RCGCGPIO_R2
#define PORTB SYSCTL_RCGCGPIO_R1
#define PORTA SYSCTL_RCGCGPIO_R0

// tiva_init() ipl status flag bitmap
#define	IPL_HIBINIT		0x0001		// HIB is activated
#define	IPL_UART0INIT	0x0002		// UART0 initialized
#define	IPL_UART1INIT	0x0004		// UART1 initialized
#define	IPL_UART2INIT	0x0008		// UART2 initialized
#define	IPL_PWM1INIT	0x0010		// PWM module 1 initialized
#define	IPL_ADCINIT		0x0020		// ADC initialized
#define	IPL_TIMER0INIT	0x0040		// Timer0 initialized
#define	IPL_TIMER1INIT	0x0080		// Timer1 initialized
#define	IPL_TIMER2INIT	0x0100		// Timer2 initialized
#define	IPL_PLLINIT		0x0200		// PLL initialized
// tiva_init() ipl error flag bitmap
#define	IPL_REGWERR		0x2000		// register bit wait error
#define	IPL_EEPERR		0x4000		// EEPROM configuration error
#define	IPL_HIBERR		0x8000		// HIB configuration error

#define	GPIOA_NVIC		0x01
#define	UART_NVIC		0x02
#define	TIMER2_NVIC		0x04
#define	ALL_NVIC		0xff

// PWM defines
#define	PWM_FREQ		10000		// this is the PWM frequency in Hz
#define	PWM_DIV			2			// bit pattern for the PWM_DIV field in SYSCTL_RCC
									// 0 = /2,  1 = /4,  2 = /8,  3 = /16,
									// 4 = /32, 5 = /64, 6 = /64, 7 = /64
#define	PWM_DIVSR		8			// = 2^(PWM_DIV + 1)
#define	PWM_ZERO		((SYSCLK/(PWM_DIVSR * PWM_FREQ)) - 1)		// the PWM "zero" point based on a 2000/8E6 PWM frequency
#define	PWM_MAX			(5*PWM_ZERO/8)	//1499		// this is the max PWM counter value for the LED output
#define	PWM_MID			(PWM_MAX + ((PWM_ZERO - PWM_MAX)/2))	//1499		// this is the max PWM counter value for the LED output

//-----------------------------------------------------------------------------
// Fn prototypes
//-----------------------------------------------------------------------------

U16 proc_init(void);
void proc_init_NVIC_en(U8 sel);

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------

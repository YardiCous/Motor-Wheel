/*
 *----------------------------------------------------------------------
 *    micro T-Kernel 3.0 BSP
 *
 *    Copyright (C) 2021-2022 by Ken Sakamura.
 *    This software is distributed under the T-License 2.2.
 *----------------------------------------------------------------------
 *
 *    Released by TRON Forum(http://www.tron.org) at 2022/11.
 *
 *----------------------------------------------------------------------
 */

/*
 *    tm_com.c
 *    T-Monitor Communication low-level device driver (RaspberryPi Pico / Pico W)
 *
 *    Serial monitor output is disabled for both USB CDC and UART0.
 *    Keep the input and initialization interfaces available to the kernel.
 */

#include <tk/typedef.h>
#include <sys/sysdef.h>

#if USE_TMONITOR
#include "../../libtm.h"

#ifdef PICO_RP2040

/* UART register definition (Use UART0) */
#define UART_BASE              0x40034000

#define UART_DR		(*(_UW*)(UART_BASE+0x000))
#define UART_FR		(*(_UW*)(UART_BASE+0x018))
#define UART_IBRD	(*(_UW*)(UART_BASE+0x024))
#define UART_FBRD	(*(_UW*)(UART_BASE+0x028))
#define UART_LCR_H	(*(_UW*)(UART_BASE+0x02C))
#define UART_CR		(*(_UW*)(UART_BASE+0x030))

#define	FR_RXFE		(1<<4)	// Receive FIFO empty

/* Discard every console write, including boot messages and input echoes. */
EXPORT	void	tm_snd_dat( const UB* buf, INT size )
{
	(void)buf;
	(void)size;
}

EXPORT	void	tm_rcv_dat( UB* buf, INT size )
{
#if TM_CONSOLE_UART
	for( ; size > 0; size--, buf++ ){
		while ( (UART_FR & FR_RXFE) != 0 );
		*buf = UART_DR & 0xff;
	}
#else
	/* Console input is not implemented on the USB-only configuration. */
	for( ; size > 0; size--, buf++ ){
		*buf = 0;
	}
#endif
}

EXPORT	void	tm_com_init(void)
{
#if TM_CONSOLE_UART
	UART_IBRD	= 67;			/* Baud rate setting */
	UART_FBRD	= 52;
	UART_LCR_H	= 0x70;			/* Communication data format setting */
	UART_CR 	= (1<<9)|(1<<8)|(1<<0);	/* Communication enabled */
#endif
	/* The USB side needs the scheduler, so it is started later from
	   knl_start_device(); nothing to do here. */
}

#endif /* PICO_RP2040 */
#endif /* USE_TMONITOR */

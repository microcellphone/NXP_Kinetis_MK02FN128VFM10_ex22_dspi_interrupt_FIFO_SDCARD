/*
 * Copyright 2016-2023 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    MK02FN128VFM10_ex00_basic_MK02FN128VFM10.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK02F12810.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include "fsl_uart.h"
#include "MK02FN128VFM10_uart.h"
#include "xprintf.h"
#include "my_delay.h"
#include "diskio.h"
#include "pffconf.h"
#include "pff.h"

/* TODO: insert other definitions and declarations here. */
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*******************************************************************************
 * Variables
 ******************************************************************************/
char Line[128];		/* Console input buffer */
/*******************************************************************************
 * Code
 ******************************************************************************/

static void put_drc (uint8_t res)
{
    xprintf("rc=%d\n\r", res);
}

static void put_rc (FRESULT rc)
{
	const char *p;
	FRESULT i;
	static const char str[] =
		"OK\0DISK_ERR\0NOT_READY\0NO_FILE\0NOT_OPENED\0NOT_ENABLED\0NO_FILE_SYSTEM\0";

	for (p = str, i = 0; i != rc && *p; i++) {
		while(*p++) ;
	}
	xprintf("rc=%u FR_%S\n", rc, p);
}

static void put_dump (
	const void* buff,		/* Pointer to the array to be dumped */
	unsigned long addr,		/* Heading address value */
	int len,				/* Number of items to be dumped */
	int width				/* Size of the items (DW_CHAR, DW_SHORT, DW_LONG) */
)
{
	int i;
	const unsigned char *bp;
	const unsigned short *sp;
	const unsigned long *lp;


	xprintf("%08lX:", addr);		/* address */

	switch (width) {
	case DW_CHAR:
		bp = buff;
		for (i = 0; i < len; i++)		/* Hexdecimal dump */
			xprintf(" %02X", bp[i]);
		xputc(' ');
		for (i = 0; i < len; i++)		/* ASCII dump */
			xputc((bp[i] >= ' ' && bp[i] <= '~') ? bp[i] : '.');
		break;
	case DW_SHORT:
		sp = buff;
		do								/* Hexdecimal dump */
			xprintf(" %04X", *sp++);
		while (--len);
		break;
	case DW_LONG:
		lp = buff;
		do								/* Hexdecimal dump */
			xprintf(" %08LX", *lp++);
		while (--len);
		break;
	}

#if !_LF_CRLF
	xputc('\r');
#endif
	xputc('\n');
}


/*
 * @brief   Application entry point.
 */
int main(void) {
	char *ptr;
	long p1, p2;
	BYTE res;
	UINT s1, s2, s3, ofs, cnt, w;
	FATFS fs;			/* File system object */
	DIR dir;			/* Directory object */
	FILINFO fno;		/* File information */

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    xdev_out(UART0_PutByte);
    xdev_in(UART0_GetByte);

    xprintf ("MK02FN128VFM10_ex22_dspi_interrupt_FIFO_SDCARD\n") ;


    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
		xputs("SDCARD>");
		xgets(Line, sizeof Line);
		ptr = Line;

		switch (*ptr++) {

		case 'd' :
			switch (*ptr++) {
			case 'i' :	/* di - Initialize physical drive */
				res = disk_initialize();
				put_drc(res);
				break;

			case 'd' :	/* dd <sector> <ofs> - Dump partial secrtor 128 bytes */
				if (!xatoi(&ptr, &p1) || !xatoi(&ptr, &p2)) break;
				s2 = p2;
				res = disk_readp((BYTE*)Line, p1, s2, 128);
				if (res) { put_drc(res); break; }
				s3 = s2 + 128;
				for (ptr = Line; s2 < s3; s2 += 16, ptr += 16, ofs += 16) {
					s1 = (s3 - s2 >= 16) ? 16 : s3 - s2;
					put_dump((BYTE*)ptr, s2, s1, 16);
				}
				break;
			}
			break;

		case 'f' :
			switch (*ptr++) {

			case 'i' :	/* fi - Mount the volume */
				put_rc(pf_mount(&fs));
				break;

			case 'o' :	/* fo <file> - Open a file */
				while (*ptr == ' ') ptr++;
				put_rc(pf_open(ptr));
				break;
#if PF_USE_READ
			case 'd' :	/* fd - Read the file 128 bytes and dump it */
				ofs = fs.fptr;
				res = pf_read(Line, sizeof Line, &s1);
				if (res != FR_OK) { put_rc(res); break; }
				ptr = Line;
				while (s1) {
					s2 = (s1 >= 16) ? 16 : s1;
					s1 -= s2;
					put_dump((BYTE*)ptr, ofs, s2, DW_CHAR);
					ptr += 16; ofs += 16;
				}
				break;

			case 't' :	/* ft - Type the file data via dreadp function */
				do {
					res = pf_read(0, 32768, &s1);
					if (res != FR_OK) { put_rc(res); break; }
				} while (s1 == 32768);
				break;
#endif
#if PF_USE_WRITE == 1
			case 'w' :	/* fw <len> <val> - Write data to the file */
				if (!xatoi(&ptr, &p1) || !xatoi(&ptr, &p2)) break;
				for (s1 = 0; s1 < sizeof Line; Line[s1++] = (BYTE)p2) ;
				p2 = 0;
				while (p1) {
					if ((UINT)p1 >= sizeof Line) {
						cnt = sizeof Line; p1 -= sizeof Line;
					} else {
						cnt = (UINT)p1; p1 = 0;
					}
					res = pf_write(Line, cnt, &w);	/* Write data to the file */
					p2 += w;
					if (res != FR_OK) { put_rc(res); break; }
					if (cnt != w) break;
				}
				res = pf_write(0, 0, &w);		/* Finalize the write process */
				put_rc(res);
				if (res == FR_OK)
					xprintf("%lu bytes written.\n", p2);
				break;

			case 'p' :	/* fp - Write console input to the file */
				xputs("Enter lines to write. A blank line finalize the write operation.\n");
				for (;;) {
					xgets(Line, sizeof Line);
					if (!Line[0]) break;
					strcat(Line, "\r\n");
					res = pf_write(Line, strlen(Line), &w);	/* Write a line to the file */
					if (res) break;
				}
				res = pf_write(0, 0, &w);		/* Finalize the write process */
				put_rc(res);
				break;
#endif
#if PF_USE_LSEEK == 1
			case 'e' :	/* fe - Move file pointer of the file */
				if (!xatoi(&ptr, &p1)) break;
				res = pf_lseek(p1);
				put_rc(res);
				if (res == FR_OK)
					xprintf("fptr = %lu(0x%lX)\n", fs.fptr, fs.fptr);
				break;
#endif
#if PF_USE_DIR == 1
			case 'l' :	/* fl [<path>] - Directory listing */
				while (*ptr == ' ') ptr++;
				res = pf_opendir(&dir, ptr);
				if (res) { put_rc(res); break; }
				s1 = 0;
				for(;;) {
					res = pf_readdir(&dir, &fno);
					if (res != FR_OK) { put_rc(res); break; }
					if (!fno.fname[0]) break;
					if (fno.fattrib & AM_DIR)
						xprintf("   <DIR>   %s\n", fno.fname);
					else
						xprintf("%9lu  %s\n", fno.fsize, fno.fname);
					s1++;
				}
				xprintf("%u item(s)\n", s1);
				break;
#endif
			}
			break;
		}
        i++;
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        __asm volatile ("nop");
    }
    return 0 ;
}

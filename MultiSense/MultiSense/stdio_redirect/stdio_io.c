/* *****************************************************************************************************************************************************************
   (C) Copyright Detection Technologies Ltd / PeriNet GmbH 2018. All Rights Reserved
   - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
   Nine Ways Research & Development Ltd authorized on behalf of Detection / PeriNet
   Project: MultiSense / VibraTek / MaxiIO / MiniIO / Daughter Card IO
   Author:  Mr Paul P. Bates
   Company: Nine Ways Research & Development Ltd
   Date:    May/June 2018
   - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
   License: STRICTLY FOR DETECTION TECHNOLOGY LTD, PERINET GmbH, NINE WAYS R&D LTD. ANY UNAUTHORIZED COPYING OR DISTRIBUTION IS PROHIBITED
   File:    Part of the MultiSense Source Build in Atmel Studio for Microchip SAM Series CPU/SoC
   ***************************************************************************************************************************************************************** */

#include <stdio.h>
#include <stdio_io.h>

/** IO descriptor for STDIO access. */
static struct io_descriptor *stdio_io = NULL;

void stdio_io_init(struct io_descriptor *io)
{
#if defined(__GNUC__)
	/* Specify that stdout and stdin should not be buffered. */
	setbuf(stdout, NULL);
	setbuf(stdin, NULL);
/* Note: Already the case in IAR's Normal DLIB default configuration
 * and AVR GCC library:
 * - printf() emits one character at a time.
 * - getchar() requests only 1 byte to exit.
 */
#endif
	stdio_io = io;
}

void stdio_io_set_io(struct io_descriptor *io)
{
	stdio_io = io;
}

int32_t stdio_io_read(uint8_t *buf, const int32_t len)
{
	if (stdio_io == NULL) {
		return 0;
	}
	return io_read(stdio_io, buf, len);
}

int32_t stdio_io_write(const uint8_t *buf, const int32_t len)
{
	if (stdio_io == NULL) {
		return 0;
	}
	return io_write(stdio_io, buf, len);
}

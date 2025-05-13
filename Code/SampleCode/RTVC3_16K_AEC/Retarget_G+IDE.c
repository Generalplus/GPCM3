/**************************************************************************************************
 * Copyright(c) 2019 GENERALPLUS Technology Corporation. All rights reserved.
 *-------------------------------------------------------------------------------------------------
 * @File:
 *   RETARGET.C
 * @Version:
 *   V0.9.0
 * @Date:
 *   30th, April 2019
 * @Abstract:
 *   This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 **************************************************************************************************/

/*---------------------------------------------------------------------------------------
 * Header File Included Area
 *---------------------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <_ansi.h>
#include <time.h>
#include <sys/errno.h>
#include <sys/stat.h>
#if defined(__CC_ARM)
#include <rt_misc.h>
#endif
#include "GPCM2_CM3.h"
#pragma import(__use_no_semihosting_swi)

/*---------------------------------------------------------------------------------------
 * Code Area
 *---------------------------------------------------------------------------------------*/
int sendchar(uint32_t tempuartdata)
{
	UART0->DATA = (tempuartdata & 0x000000FF);                                    // Send data to UART buffer
	while(!(READ_BIT(UART0->STS, UART_STS_TX_DONE_FLAG)));                        // Wait until transmission is done.
	CLEAR_FLAG(UART0->STS, UART_STS_TX_DONE_MSK, UART_STS_TX_DONE_FLAG);	         // Clear transmission flag after transmission is completed.
	return 0;
}

int getkey(void)
{
	while(!(READ_BIT(UART0->STS,  UART_STS_RX_DAT_NEMP_FLAG)));                    // Wait for receiving data from PC or any device (for RX incoming data acquire)
	CLEAR_FLAG(UART0->STS, UART_STS_RX_DAT_NEMP_MSK, UART_STS_RX_DAT_NEMP_FLAG);  // After data received, clear UART status data is not empty flag.
	return (int8_t)(READ_BIT(UART0->DATA, 0xFF));                                 // Return key-in data to UART_DATA
}

#if defined(__CC_ARM)
struct __FILE { int handle; /* Add whatever you need here */ };
#endif
FILE __stdout;
FILE __stdin;

/*
int fputc(int ch, FILE *f) {
  return (sendchar(ch));
}

int fgetc(FILE *f) {
  return (sendchar(getkey()));
}*/
int io_putchar(int ch)
{
    sendchar(ch);
    return ch;
}
int io_getchar(void)
{
    int ch;
	sendchar(getkey());
	return ch;
}

#if defined(__CC_ARM)
int ferror(FILE *f) {
  // Your implementation of ferror
  return EOF;
}
#endif
void _ttywrch(int ch) {
  io_putchar (ch);
}

void _sys_exit(int return_code) {
  while (1);    // endless loop
}

#if defined(__GNUC__)
void
__aeabi_memclr (void *dest, size_t n)
{
  memset (dest, 0, n);
}
#endif

/*long
_write_r(	struct _reent *r,
			int fd,
			const void *buf,
			size_t cnt )
{
	const unsigned char *p = (const unsigned char*) buf;
	int i;

	for (i = 0; i < cnt; i++)
	{
		if (*p == '\n' )
			io_putchar('\r');
		io_putchar(*p++);
	}
	return cnt;
}*/

long
_read_r(	struct _reent *r,
			int fd,
			char *buf,
			size_t cnt )
{
	unsigned char *p = (unsigned char*) buf;
	char c;
	int  i;

	for (i = 0; i < cnt; i++)
	{
		c = io_getchar();

		*p++ = c;
		io_putchar(c);

		if (c == '\r' && i <= (cnt - 2))
		{
			*p = '\n';
			io_putchar(*p);
			return i + 2;
		}
	}
	return i;
}

char *heap_ptr;

/*
 * sbrk -- changes heap size size. Get nbytes more
 *         RAM. We just increment a pointer in what's
 *         left of memory on the board.
 */

char _end[1024];
char _stdbuf[1024];

__attribute__ ((weak)) char *
_DEFUN (_sbrk, (nbytes),
       int nbytes)

{
  char        *base;

  if (!heap_ptr)
    heap_ptr = (char *)&_stdbuf[0];
  base = heap_ptr;
  heap_ptr += nbytes;

  return base;
}

__attribute__ ((weak)) void outbyte(char x)
{
    /* Leaves for implemetation */
    /* Users can write a char to UART or serial port */
    io_putchar(x);
}

/*
 * write -- write bytes to the serial port. Ignore fd, since
 *          stdout and stderr are the same. Since we have no filesystem,
 *          open will only return an error.
 */

__attribute__ ((weak)) int
_DEFUN (_write, (fd, buf, nbytes),
       int fd _AND
       char *buf _AND
       int nbytes)
{
  int i;

  for (i = 0; i < nbytes; i++)
  {
    if (*(buf + i) == '\n') {
      outbyte ('\r');
    }
    outbyte (*(buf + i));
  }
  return (nbytes);
}

char _DEFUN_VOID (inbyte) __attribute__((weak));
char inbyte(void)
{
    /* Leaves for implemetation */
    /* Users can get a char from UART or serial port */
}

/*
 * read  -- read bytes from the serial port. Ignore fd, since
 *          we only have stdin.
 */
__attribute__ ((weak)) int
_DEFUN (_read, (fd, buf, nbytes),
       int fd _AND
       char *buf _AND
       int nbytes)
{
  int i = 0;

  for (i = 0; i < nbytes; i++) {
    *(buf + i) = inbyte();
    if ((*(buf + i) == '\n') || (*(buf + i) == '\r')) {
      i++;
      break;
    }
  }
  return (i);
}
/*
 * close -- We don't need to do anything, but pretend we did.
 */
__attribute__ ((weak)) int
_DEFUN (_close ,(fd),
       int fd)
{
  return (0);
}
/*
 * lseek --  Since a serial port is non-seekable, we return an error.
 */
__attribute__ ((weak)) off_t
_DEFUN (_lseek, (fd,  offset, whence),
       int fd _AND
       off_t offset _AND
       int whence)
{
  errno = ESPIPE;
  return ((off_t)-1);
}

/*
 * fstat -- Since we have no file system, we just return an error.
 */
__attribute__ ((weak)) int
_DEFUN (_fstat, (fd, buf),
       int fd _AND
       struct stat *buf)
{
  buf->st_mode = S_IFCHR;	/* Always pretend to be a tty */
  buf->st_blksize = 0;

  return (0);
}
/*
 * isatty -- returns 1 if connected to a terminal device,
 *           returns 0 if not. Since we're hooked up to a
 *           serial port, we'll say yes _AND return a 1.
 */
__attribute__ ((weak)) int
_DEFUN (_isatty, (fd),
       int fd)
{
  return (1);
}

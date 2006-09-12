/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

/** static device driver only. device node do not contains pointer to
    driver functions */
//#define CONFIG_STATIC_DRIVERS

/** TTY support */
#define CONFIG_TTY

/** UART support */
#define CONFIG_UART

/** TTY support use uart */
//#define CONFIG_TTY_UART

/** frame buffer support */
//#define CONFIG_FB

/** timer support */
#define CONFIG_TIMER

/** VGA tty support for ANSI codes */
#define CONFIG_VGATTY_ANSI

/** Multi processor support  */
#define CONFIG_SMP

/** Default entries count in pthread stack */
#define CONFIG_PTHREAD_STACK_SIZE	(1 << 20)

/** Extensive error checking for pthread API */
#define CONFIG_PTHREAD_CHECK

/** Provide pthread_cancel support */
#define CONFIG_PTHREAD_CANCEL

/** Provide joinable pthread support */
#define CONFIG_PTHREAD_JOIN

/** Provide pthread mutex attributes support */
#define CONFIG_PTHREAD_MUTEX_ATTR

/** Libc printf features are reduced */
//#define CONFIG_LIBC_PRINTF_SIMPLE

/** Libc printf support %S and %P extentions to dump buffers */
#define CONFIG_LIBC_PRINTF_EXT

/* Mips processor version (1, 2, 3, 4, 32, 322, 64) */
#define CONFIG_MIPS_VERSION	1

/* Mips ABI */
#define CONFIG_MIPS_ABI_O32
//#define CONFIG_MIPS_ABI_O64
//#define CONFIG_MIPS_ABI_N32
//#define CONFIG_MIPS_ABI_N64
//#define CONFIG_MIPS_ABI_EABI

/* Generate position independant code */
#define CONFIG_PIC

/* Have all device nodes sorted in a tree form */
#define CONFIG_DEVICE_HIERARCHY

/* Network option: force aligned memory accesses */
#define CONFIG_NETWORK_AUTOALIGN

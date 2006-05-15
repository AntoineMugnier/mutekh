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

/** VGA tty support for ANSI codes */
#define CONFIG_VGATTY_ANSI

/** Multi processor support  */
#define CONFIG_SMP

/** Default entries count in pthread stack */
#define CONFIG_PTHREAD_STACK_SIZE	(1 << 16)

/** Extensive error checking for pthread API */
#define CONFIG_PTHREAD_CHECK

/** Provide pthread_cancel support */
#define CONFIG_PTHREAD_CANCEL

/** Provide joinable pthread support */
#define CONFIG_PTHREAD_JOIN

/** Provide pthread mutex attributes support */
#define CONFIG_PTHREAD_MUTEX_ATTR

#define CONFIG_FB_VGA

/** Libc printf features are reduced */
//#define CONFIG_LIBC_PRINTF_SIMPLE

/** Libc printf support %S and %P extentions to dump buffers */
#define CONFIG_LIBC_PRINTF_EXT


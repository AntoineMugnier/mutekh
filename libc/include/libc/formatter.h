/*
    This file is part of MutekH.
    
    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.
    
    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#ifndef MUTEK_PRINTF_ARG_H_
#define MUTEK_PRINTF_ARG_H_

/**
 * @file
 * @module{C library}
 */

#include <stdarg.h>
#include <hexo/types.h>

#define PRINTF_OUTPUT_FUNC(x) void (x)(void *ctx, const char *str, size_t offset, size_t len)

typedef PRINTF_OUTPUT_FUNC(printf_output_func_t);

/** @This is used to produce output of @ref printk and @ref printf
    family of functions. The output function will be called multiple
    times to write the formatted string.

    This function supports the standard printf format string syntax.

    The following non standard conversion specifiers are supported:
    @list
      @item @tt %b output an unsigned integer in binary format
      @item @tt %P output an hexadecimal dump of memory,
            both address and size arguments must be passed.
      @item @tt %S output an string with specified length,
            both address and size arguments must be passed.
    @end list

    When the @ref #CONFIG_LIBC_FORMATTER_SIMPLE configuration token is
    defined, the conversion features are reduced to bare minimum: no
    padding support, only @tt{%d %i %u %s %p %c %x %o} are handled and
    maximum integer value is limited to processor register width.
    The precision is ignored too; this changes behavior of @tt {%.s}.
*/
ssize_t
formatter_printf(void *ctx, printf_output_func_t * const fcn,
                 const char *format, va_list ap);

void
mutek_hexdump_arg(void *ctx, printf_output_func_t * const fcn,
                  uintptr_t address, const void *base, size_t len);

/** @this write a string from a floating point value */
ssize_t
formatter_dtostr(double d, char *buf, size_t maxlen,
                 size_t prec, size_t prec2, ssize_t g);

#endif

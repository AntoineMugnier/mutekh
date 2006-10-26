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

/*

    %config CONFIG_LIBC_PRINTF_SIMPLE
    desc libc printf() features are reduced
    default undefined
    %config end

    %config CONFIG_LIBC_PRINTF_EXT
    desc libc printf support %S and %P extentions to dump buffers
    default defined
    %config end

*/

#include <hexo/types.h>

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

typedef void __printf_out_t(void *ctx, const char *str, size_t offset, size_t len);
typedef int_fast8_t __printf_int_t;

#define PRINTF_INT_BUFFER_LEN	20

struct __printf_str_s
{
  size_t	size;
  char		*out;
};

static inline void
__printf_out_str(void *ctx_, const char *str, size_t offset, size_t len)
{
  struct __printf_str_s	*ctx = ctx_;

  if (ctx->size > offset)
    {
      size_t	size = __MIN(ctx->size - offset, len);

      memcpy(ctx->out + offset, str, size);
    }
}

static inline void
__printf_out_tty(void *ctx, const char *str, size_t offset, size_t len)
{
  __puts(str, len);
}

static inline size_t
__printf_putint(char *buf, __printf_int_t val,
		const char *base, uint_fast8_t basesize)
{
  int_fast8_t	i;

  for (i = PRINTF_INT_BUFFER_LEN; i > 0; )
    {
      buf[--i] = base[val % basesize];

      if (!(val /= basesize))
	break;
    }

  return PRINTF_INT_BUFFER_LEN - i;
}

#ifdef CONFIG_LIBC_PRINTF_EXT
static size_t
__printf_hexdump(char *buf, const uint8_t *val, size_t len)
{
  static const char	*hex = "0123456789abcdef";
  size_t		i;

  if (!len)
    return 0;

  for (i = 0; i < len; i++)
    {
      *buf++ = hex[*val >> 4];
      *buf++ = hex[*val & 15];
      val++;
      *buf++ = ' ';
    }

  return len * 3 - 1;
}
#endif

static ssize_t
__printf_arg(void *ctx, __printf_out_t * const fcn, 
	     const char *format, va_list ap)
{
  size_t	offset = 0;
#ifndef CONFIG_LIBC_PRINTF_SIMPLE
  uint_fast8_t	typesize;
  ssize_t	padding;
  bool_t	zeropad, rightpad;
#endif

 printf_state_main:
  while (*format)
    {
      size_t	i;

      for (i = 0; format[i] && format[i] != '%'; i++)
	;

      if (i)
	{
	  fcn(ctx, format, offset, i);
	  offset += i;
	  format += i;
	}

      if (*format == '%')
	{
	  format++;
	  goto printf_state_modifier;
	}
    }
  return offset;

 printf_state_modifier:
#ifndef CONFIG_LIBC_PRINTF_SIMPLE
  padding = 0;
  zeropad = rightpad = 0;
  typesize = sizeof(int_fast8_t);
#endif

  while (*format)
    {
      switch (*format)
	{
	case ('%'): {
	  fcn(ctx, format++, offset++, 1);
	  goto printf_state_main;
	}

#ifndef CONFIG_LIBC_PRINTF_SIMPLE
	case '-':
	  rightpad = 1;
	  format++;
	  break;

	case '0':
	  if (!padding)
	    zeropad = 1;

	case '1' ... '9':
	  padding = padding * 10 + *format++ - '0';
	  break;

	case 'l':
	  typesize *= 2;
	  format++;
	  break;
#endif

	case 's':
	case 'p':
#ifndef CONFIG_LIBC_PRINTF_SIMPLE
# ifdef CONFIG_LIBC_PRINTF_EXT
	case 'S':
	case 'P':
# endif
	  typesize = sizeof(void *);
#endif
	default:
	  goto printf_state_conv;
	}
    }
  return offset;

 printf_state_conv: {
    __printf_int_t	val;
    char		*buf;
    char		buf_[PRINTF_INT_BUFFER_LEN];
    size_t		len;

#ifndef CONFIG_LIBC_PRINTF_SIMPLE
    switch (typesize)
      {
      case 1:
	val = va_arg(ap, int_fast8_t);
	break;

      case 2:
	val = va_arg(ap, int_fast16_t);
	break;

      case 4:
	val = va_arg(ap, int_fast32_t);
	break;

      default:
      case 8:
	val = va_arg(ap, int_fast64_t);
	break;
      }
#else
    val = va_arg(ap, uintptr_t);
#endif

    switch (*format++)
      {
	/* char conversion */

      case ('c'):
	len = 1;
	buf = buf_;
	buf[0] = val;
	break;

	/* decimal signed integer */

      case ('d'):
      case ('i'):
	if (val < 0)
	  {
	    val = -val;
	    fcn(ctx, "-", offset++, 1);
	  }

	len = __printf_putint(buf_, val, "0123456789", 10);
	buf = buf_ + PRINTF_INT_BUFFER_LEN - len;
	break;

	/* decimal unsigned integer */

      case ('u'):
	len = __printf_putint(buf_, val, "0123456789", 10);
	buf = buf_ + PRINTF_INT_BUFFER_LEN - len;
	break;

	/* hexadecimal unsigned integer */

#ifndef CONFIG_LIBC_PRINTF_SIMPLE
      case ('p'):
	fcn(ctx, "0x", offset, 2);
	offset += 2;
	zeropad = 1;
	padding = sizeof(void*) * 2;
	rightpad = 0;
#endif

      case ('X'):
#ifndef CONFIG_LIBC_PRINTF_SIMPLE
	len = __printf_putint(buf_, val, "0123456789ABCDEF", 16);
	buf = buf_ + PRINTF_INT_BUFFER_LEN - len;
	break;
#endif

      case ('x'):
	len = __printf_putint(buf_, val, "0123456789abcdef", 16);
	buf = buf_ + PRINTF_INT_BUFFER_LEN - len;
	break;

	/* octal integer */

      case ('o'):
	len = __printf_putint(buf_, val, "01234567", 8);
	buf = buf_ + PRINTF_INT_BUFFER_LEN - len;
	break;

	/* string */

      case ('s'):
	len = strlen((char*)val);
	buf = (char*)val;
#ifndef CONFIG_LIBC_PRINTF_SIMPLE
	zeropad = 0;
#endif
	break;

	/* hexdump data buffer */

#ifdef CONFIG_LIBC_PRINTF_EXT
      case ('P'):	
	len = va_arg(ap, size_t);
	buf = __builtin_alloca(len * 3);
	len = __printf_hexdump(buf, (uint8_t*)val, len);
	break;

	/* string data buffer */

      case ('S'):
	len = va_arg(ap, size_t);
	buf = (char*)val;
# ifndef CONFIG_LIBC_PRINTF_SIMPLE
	zeropad = 0;
# endif
	break;
#endif

      default:
	goto printf_state_main;
      }

#ifndef CONFIG_LIBC_PRINTF_SIMPLE
    padding = __MAX((ssize_t)(padding - len), 0);

    if (!rightpad)
      for (; padding; padding--)
	fcn(ctx, zeropad ? "0" : " ", offset++, 1);
#endif

    fcn(ctx, buf, offset, len);
    offset += len;

#ifndef CONFIG_LIBC_PRINTF_SIMPLE
    while (padding--)
      fcn(ctx, " ", offset++, 1);
#endif
  }
  goto printf_state_main;
}

inline ssize_t vsnprintf(char *str, size_t size, const char *format, va_list ap)
{
  ssize_t	res;

  struct __printf_str_s	ctx = {
    .size = size,
    .out = str,
  };

  res = __printf_arg(&ctx, __printf_out_str, format, ap);

  /* add final \0 to output string */
  if (ctx.size > res)
    str[res] = '\0';

  return res + 1;
}

ssize_t snprintf(char *str, size_t size, const char *format, ...)
{
  ssize_t	res;
  va_list	ap;

  va_start(ap, format);
  res = vsnprintf(str, size, format, ap);
  va_end(ap);

  return res;
}

inline ssize_t vprintf(const char *format, va_list ap)
{
  return __printf_arg(0, __printf_out_tty, format, ap);
}

ssize_t printf(const char *format, ...)
{
  ssize_t	res;
  va_list	ap;

  va_start(ap, format);
  res = vprintf(format, ap);
  va_end(ap);

  return res;
}

inline ssize_t vsprintf(char *str, const char *format, va_list ap)
{
  return vsnprintf(str, -1, format, ap);
}

ssize_t sprintf(char *str, const char *format, ...)
{
  ssize_t	res;
  va_list	ap;

  va_start(ap, format);
  res = vsprintf(str, format, ap);
  va_end(ap);

  return res;
}


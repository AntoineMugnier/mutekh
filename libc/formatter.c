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

#include <stdlib.h>
#include <string.h>
#include <hexo/types.h>

#include <libc/formatter.h>

#ifdef CONFIG_LIBC_FORMATTER_SIMPLE
typedef uintptr_t __printf_uint_t;
typedef intptr_t __printf_int_t;
# define PRINTF_SIZEOF_VAL INT_REG_SIZE
#else
typedef uint64_t __printf_uint_t;
typedef int64_t __printf_int_t;
# define PRINTF_SIZEOF_VAL 64
#endif

#ifdef CONFIG_LIBC_FORMATTER_SIMPLE
# define PRINTF_INT_BUFFER_LEN	22 // large enough for 64 bits decimal number with sign and space
#else
# define PRINTF_INT_BUFFER_LEN	PRINTF_SIZEOF_VAL+2 // large enough for binary format with 0b prefix
#endif

static size_t
printf_base10(char *buf, __printf_uint_t x)
{
  int_fast8_t	i;

  for (i = PRINTF_INT_BUFFER_LEN; i > 0; )
    {
#ifdef CONFIG_LIBC_FORMATTER_DIV10
      uint8_t z = x % 10;
#else
      /* B. Arazi and D. Naccache, Binary to Decimal Conversion Based on
         the Divisibility of 255 by 5, Electronic Letters, Vol. 28, Num. 23, 1992 */

      /* mod 255 */
      uint16_t sum;
      __printf_uint_t y;
      for (y = x; y > 255; y = sum)
        for (sum = 0; y != 0; y >>= 8)
          sum += y & 0xff;

      /* mod 5 */
      uint8_t z = y;
# if 1
      z -= ((z * 205) >> 10) * 5;
# else
      z = (z & 15) + (z >> 4);
      z = (z & 15) + (z >> 4);
      z = (z &  3) - (z >> 2);
      z += (((int8_t)z >> 7) & 5);
# endif

      /* mod 10 */
      uint8_t w = ((x ^ z) & 1);
      w |= w << 2;
      z += w;
#endif

      buf[--i] = z + '0';

      x -= z;
      if (x == 0)
        break;

#ifdef CONFIG_LIBC_FORMATTER_DIV10
      x = x / 10;
#else

      /* multiply by 510 / 20 */
# if 0
      x = x / 2 * 3 * 17;   /* 64 bits mul */
# else
      x >>= 1;
      x += x << 1;
      x += x << 4;
# endif

      /* div by 255 */
      __printf_uint_t s = -x;
      __printf_uint_t r = 0;
      uint_fast8_t i;

      for (i = 0; i < sizeof(x); i++)
        {
          r = (r >> 8) | (s << (8 * sizeof(__printf_uint_t) - 8));
          s = ((s & 0xff) + (s >> 8));
        }
      x = r;
#endif
    }

  return PRINTF_INT_BUFFER_LEN - i;
}

static size_t
printf_base_pow2(char *buf, __printf_uint_t val,
                 const char *base, uint_fast8_t base_shift)
{
  int_fast8_t	i;

  for (i = PRINTF_INT_BUFFER_LEN; i > 0; )
    {
      buf[--i] = base[val & ((1ULL << base_shift) - 1)];

      if (!(val >>= base_shift))
	break;
    }

  return PRINTF_INT_BUFFER_LEN - i;
}

#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
static size_t
printf_hexdump(char *buf, const uint8_t *val, size_t len)
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



#if defined(CONFIG_LIBC_FORMATTER_FLOAT)
typedef double __fpmax_t;

ssize_t __dtostr(double d,char *buf,size_t maxlen,size_t prec,size_t prec2,ssize_t g);

static inline
void _printf_float(void *ctx, printf_output_func_t * const fcn, __fpmax_t x)
{
    char buf[64];
    ssize_t len = formatter_dtostr(x, buf, sizeof(buf), 6, 6, 0);
    fcn(ctx, buf, 0, len);
}
#endif



static const char *hex_lower_base = "0123456789abcdef0x";

#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
static const char *hex_upper_base = "0123456789ABCDEF0X";
#else
# define hex_upper_base hex_lower_base
#endif

#define PRINTF_FLAG_SPACE     1
#define PRINTF_FLAG_PLUS      2
#define PRINTF_FLAG_NEGATIVE  4
#define PRINTF_FLAG_ALTERNATE 8
#define PRINTF_FLAG_PRECISION 16

ssize_t
formatter_printf(void *ctx, printf_output_func_t * const fcn,
	     const char *format, va_list ap)
{

  size_t	offset = 0;
  uint_fast8_t  flags;
#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
  uint_fast8_t	typesize, padindex;
  ssize_t	padding[2];
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
  flags = 0;
#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
  padindex = 0;
  zeropad = rightpad = 0;
  padding[0] = padding[1] = 0;
  typesize = INT_REG_SIZE / 8;
#endif

  while (*format)
    {
      switch (*format)
	{
	case ('%'): {
	  fcn(ctx, format++, offset++, 1);
	  goto printf_state_main;
	}

        case '#':
#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
          flags |= PRINTF_FLAG_ALTERNATE; /* alternate form */
#endif
	  format++;
	  break;

	case '-':
#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
	  rightpad = 1;
#endif
	  format++;
	  break;

	case ' ':
#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
	  flags |= PRINTF_FLAG_SPACE;
#endif
          format++;
	  break;

	case '+':
#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
	  flags |= PRINTF_FLAG_PLUS;
#endif
          format++;
	  break;

	case '0':
#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
	  if (!padindex && !padding[padindex])
	    zeropad = 1;
#endif

	case '1' ... '9':
#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
	  padding[padindex] = padding[padindex] * 10 + *format - '0';
#endif
	  format++;
	  break;

	case '.':
#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
	  padindex ^= 1;
#endif
	  format++;
          flags |= PRINTF_FLAG_PRECISION;
	  break;

	case 'l':
#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
	  typesize = format[-1] == 'l' ? CPU_SIZEOF_LONGLONG / 8 : CPU_SIZEOF_LONG / 8;
#endif
	  format++;
	  break;

	case 'h':
#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
	  typesize = format[-1] == 'h' ? 1 : CPU_SIZEOF_SHORT / 8;
#endif
	  format++;
	  break;

        case 'q':
#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
          typesize = CPU_SIZEOF_LONGLONG;
#endif
	  format++;
	  break;

        case 'z':
#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
          typesize = sizeof(size_t);
#endif
	  format++;
	  break;

        case 't':
#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
          typesize = sizeof(ptrdiff_t);
#endif
	  format++;
	  break;

        case 'j':
#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
          typesize = sizeof(intmax_t);
#endif
	  format++;
	  break;

        case 'e':
        case 'E':
        case 'f':
        case 'F':
        case 'g':
        case 'G':
#if defined(CONFIG_LIBC_FORMATTER_FLOAT)
          _printf_float(ctx, fcn, va_arg(ap, double));
#endif
	  format++;
        goto printf_state_main;

#if defined(CONFIG_LIBC_FORMATTER_FLOAT)
	case 's':
	case 'p':
	case 'S':
	case 'P':
	  typesize = sizeof(void *);
#endif
	default:
	  goto printf_state_conv;
	}
    }
  return offset;

 printf_state_conv: {
    __printf_uint_t	val;
    char		*buf = 0;
    char		buf_[PRINTF_INT_BUFFER_LEN];
    size_t		len;

#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
    switch (typesize)
      {
#if INT_REG_SIZE <= 8
      case 1:
	val = va_arg(ap, uint8_t);
	break;
#endif
#if INT_REG_SIZE <= 16
      case 2:
	val = va_arg(ap, uint16_t);
	break;
#endif
#if INT_REG_SIZE <= 32
      case 4:
	val = va_arg(ap, uint32_t);
	break;
#endif
      case 8:
	val = va_arg(ap, uint64_t);
	break;

      default:
	val = va_arg(ap, reg_t);
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
      case ('i'): {
#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
	if (padding[1])
	  {
	    zeropad = 1;
	    padding[0] = padding[1];
	  }

        /* sign extend */
        switch (typesize)
          {
# if PRINTF_SIZEOF_VAL > 8
          case 1:
            val = (__printf_int_t)(int8_t)val;
            break;
# endif
# if PRINTF_SIZEOF_VAL > 16
          case 2:
            val = (__printf_int_t)(int16_t)val;
            break;
# endif
# if PRINTF_SIZEOF_VAL > 32
          case 4:
            val = (__printf_int_t)(int32_t)val;
            break;
# endif
          }
#endif

        if ((__printf_int_t)val < 0)
          {
            val = -val;
            flags |= PRINTF_FLAG_NEGATIVE;
          }

        buf = buf_;

        if ((__printf_int_t)val < 0)
          {
#if PRINTF_SIZEOF_VAL == 8
            static const char max[] = "128";
#elif PRINTF_SIZEOF_VAL == 16
            static const char max[] = "32768";
#elif PRINTF_SIZEOF_VAL == 32
            static const char max[] = "2147483648";
#elif PRINTF_SIZEOF_VAL == 64
            static const char max[] = "9223372036854775808";
#else
# error bad value for PRINTF_SIZEOF_VAL
#endif
            buf = buf_ + PRINTF_INT_BUFFER_LEN - sizeof(max);
            memcpy(buf, max, sizeof(max));
          }
        else
          {
            buf = buf_ + PRINTF_INT_BUFFER_LEN - printf_base10(buf_, val);
          }

#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
        if (flags & (flags > 1)) // PRINTF_FLAG_SPACE == 1
          *--buf = ' ';
#endif
        if (flags & PRINTF_FLAG_NEGATIVE)
          *--buf = '-';
#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
        else if (flags & PRINTF_FLAG_PLUS)
          *--buf = '+';
#endif
        len = PRINTF_INT_BUFFER_LEN - (buf - buf_);

        break;
      }

	/* octal integer */
      case ('o'):
	len = printf_base_pow2(buf_, val, hex_lower_base, 3);
	buf = buf_ + PRINTF_INT_BUFFER_LEN - len;
#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
        if (val && (flags & PRINTF_FLAG_ALTERNATE))
          {
            *--buf = '0';
            len++;
          }
#endif
	break;

	/* binary integer */
      case ('b'):
#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
	len = printf_base_pow2(buf_, val, hex_lower_base, 1);
	buf = buf_ + PRINTF_INT_BUFFER_LEN - len;
        if (zeropad)
          {
            if (val && (flags & PRINTF_FLAG_ALTERNATE))
              {
                fcn(ctx, "0b", offset, 2);
                offset += 2;
                if (padding[0])
                  padding[0]--;
                if (padding[0])
                  padding[0]--;
              }
          }
        else if (!zeropad && (flags & PRINTF_FLAG_ALTERNATE))
          {
            *--buf = 'b'; *--buf = '0';
            len += 2;
          }
	break;
#endif

	/* decimal unsigned integer */
      case ('u'):
#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
	/* FIXME precision should not be handled this way with %u */
	if (padding[1])
	  {
	    zeropad = 1;
	    padding[0] = padding[1];
	  }
#endif
	len = printf_base10(buf_, val);
	buf = buf_ + PRINTF_INT_BUFFER_LEN - len;
	break;

	/* hexadecimal unsigned integer */

      case ('p'):
#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
        flags |= PRINTF_FLAG_ALTERNATE;
	zeropad = 1;
	padding[0] = /* 0x */ 2 + /* ptr digits */ sizeof(void*) * 2;
	rightpad = 0;
#endif

      case ('x'):
      case ('X'): {
#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
        const char *base = format[-1] == 'X' ? hex_upper_base : hex_lower_base;
	len = printf_base_pow2(buf_, val, base, 4);
	buf = buf_ + PRINTF_INT_BUFFER_LEN - len;
        if (zeropad)
          {
            if ((val || format[-1] == 'p') && (flags & PRINTF_FLAG_ALTERNATE))
              {
                fcn(ctx, base + 16, offset, 2);
                offset += 2;
                if (padding[0])
                  padding[0]--;
                if (padding[0])
                  padding[0]--;
              }
          }
        else if (flags & PRINTF_FLAG_ALTERNATE)
          {
            *--buf = base[17]; *--buf = '0';
            len += 2;
          }
#else
	len = printf_base_pow2(buf_, val, hex_lower_base, 4);
	buf = buf_ + PRINTF_INT_BUFFER_LEN - len;
#endif
	break;
      }

	/* string */
      case ('s'): {
#ifdef CONFIG_LIBC_FORMATTER_SIMPLE
        if (flags & PRINTF_FLAG_PRECISION)
          goto printf_state_main;  /* do not handle string length limit */
#endif
	char	*str = (char*)(uintptr_t)val;

        if (!str)
          {
            len = 6;
            buf = "(null)";
            break;
          }

#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
	size_t	maxlen;
	zeropad = 0;

	if ((maxlen = padding[1]))
	  while (maxlen-- && *str)
	    str++;
	else
#endif
	  while (*str)
	    str++;

	buf = (char*)(uintptr_t)val;
	len = str - buf;
      }	break;

	/* hexdump data buffer */
      case ('P'):
	len = va_arg(ap, size_t);
#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
	buf = __builtin_alloca(len * 3);
	len = printf_hexdump(buf, (uint8_t*)(uintptr_t)val, len);
#endif
	break;

	/* string data buffer */

      case ('S'):
	len = va_arg(ap, size_t);
#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
	buf = (char*)(uintptr_t)val;
	zeropad = 0;
#endif
	break;

      default:
	goto printf_state_main;
      }

#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
    size_t padlen = __MAX((ssize_t)(padding[0] - len), 0);

    if (!rightpad)
      {
	while (padlen--)
	  fcn(ctx, zeropad ? "0" : " ", offset++, 1);
      }
#endif

    fcn(ctx, buf, offset, len);
    offset += len;

#ifndef CONFIG_LIBC_FORMATTER_SIMPLE
    if (rightpad)
    {
      while (padlen--)
	fcn(ctx, " ", offset++, 1);
    }
#endif
  }
  goto printf_state_main;
}


/**********************************************************************/

#ifdef CONFIG_LIBC_FORMATTER_HEXDUMP

static inline void hexdump_line_init(
    char *line, size_t line_width, size_t w)
{
    const size_t addrw = 8;

    memset(line, ' ', line_width);

    line[line_width-1] = '\n';
    line[addrw+1] = '|';
    line[addrw+3+w*3+1] = '|';
}

static inline void hexdump_new_line(
    char *line, uintptr_t addr,
    size_t w)
{
    size_t i;
    const size_t addrw = 8;

    for ( i=0; i<addrw; ++i ) {
        line[addrw-1-i] = hex_lower_base[addr & 0xf];
        addr >>= 4;
    }

    memset(line+addrw+3, ' ', w*3);
    memset(line+addrw+3+w*3+3, ' ', w);
}

static inline void hexdump_put_char(
    char *line, size_t index, uint8_t val,
    size_t w)
{
    const size_t addrw = 8;

    line[addrw+3+index*3]   = hex_lower_base[val >> 4];
    line[addrw+3+index*3+1] = hex_lower_base[val & 0xf];
    line[addrw+3+w*3+3+index] = (val >= 32 && val < 128) ? val : '.';
}

void
formatter_hexdump(void *ctx, printf_output_func_t * const fcn,
             uintptr_t address, const void *base, size_t size)
{
    const size_t w = 16;
    // addraddr | xx xx [12] xx xx | ..[12]..\n
    const size_t line_width = 8+3+3*w+3+w+1;
    char line[line_width];

    hexdump_line_init(line, line_width, w);
    hexdump_new_line(line, address & ~(w-1), w);

    const uint8_t *data = base;
    const uint8_t *end = data + size;

    for ( ; data < end; ++data, ++address) {
        size_t index = (uintptr_t)address % w;
        hexdump_put_char(line, index, *data, w);

        if (index == w - 1) {
            fcn(ctx, line, 0, line_width);
            bool_t once = 0;

            while (data + 1 + w < end && memcstcmp(data + 1, 0, w) == 0) {
                data += w;
                address += w;
                once = 1;
            }

            if (once) {
                fcn(ctx, "***\n", 0, 4);
            }

            if (data + 1 < end)
                hexdump_new_line(line, address + 1, w);
        }
                      
    }

    if (address & (w - 1))
        fcn(ctx, line, 0, line_width);
}

#endif


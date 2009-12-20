
#include <stdlib.h>
#include <string.h>
#include <hexo/types.h>

#include <mutek/printf_arg.h>

typedef intptr_t __printf_int_t;

#define PRINTF_INT_BUFFER_LEN	20

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

#ifdef CONFIG_PRINTF_ARG_EXT
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

ssize_t
mutek_printf_arg(void *ctx, printf_output_func_t * const fcn,
	     const char *format, va_list ap)
{
  size_t	offset = 0;
#ifndef CONFIG_PRINTF_ARG_SIMPLE
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
#ifndef CONFIG_PRINTF_ARG_SIMPLE
  padindex = 0;
  zeropad = rightpad = 0;
  padding[0] = padding[1] = 0;
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

#ifndef CONFIG_PRINTF_ARG_SIMPLE
	case '-':
	  rightpad = 1;
	  format++;
	  break;

	case '0':
	  if (!padindex && !padding[padindex])
	    zeropad = 1;

	case '1' ... '9':
	  padding[padindex] = padding[padindex] * 10 + *format++ - '0';
	  break;

	case '.':
	  padindex ^= 1;
	  format++;
	  break;

	case 'l':
	  typesize *= 2;
	  format++;
	  break;
#endif

	case 's':
	case 'p':
#ifndef CONFIG_PRINTF_ARG_SIMPLE
# ifdef CONFIG_PRINTF_ARG_EXT
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

#ifndef CONFIG_PRINTF_ARG_SIMPLE
    switch (typesize)
      {
      case 1:
	val = va_arg(ap, __compiler_sint_t);
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
#ifndef CONFIG_PRINTF_ARG_SIMPLE
	/* FIXME precision should not be handled this way with %d %i */
	if (padding[1])
	  {
	    zeropad = 1;
	    padding[0] = padding[1];
	  }
#endif
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
#ifndef CONFIG_PRINTF_ARG_SIMPLE
	/* FIXME precision should not be handled this way with %u */
	if (padding[1])
	  {
	    zeropad = 1;
	    padding[0] = padding[1];
	  }
#endif
	len = __printf_putint(buf_, val, "0123456789", 10);
	buf = buf_ + PRINTF_INT_BUFFER_LEN - len;
	break;

	/* hexadecimal unsigned integer */

      case ('X'):
#ifndef CONFIG_PRINTF_ARG_SIMPLE
	len = __printf_putint(buf_, val, "0123456789ABCDEF", 16);
	buf = buf_ + PRINTF_INT_BUFFER_LEN - len;
	break;
#endif

#ifndef CONFIG_PRINTF_ARG_SIMPLE
      case ('p'):
	fcn(ctx, "0x", offset, 2);
	offset += 2;
	zeropad = 1;
	padding[0] = sizeof(void*) * 2;
	rightpad = 0;
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

      case ('s'): {
	char	*str = (char*)val;
#ifndef CONFIG_PRINTF_ARG_SIMPLE
	size_t	maxlen;

	zeropad = 0;

	if ((maxlen = padding[1]))
	  while (maxlen-- && *str)
	    str++;
	else
#endif
	  while (*str)
	    str++;

	len = str - (char*)val;
	buf = (char*)val;
      }	break;

	/* hexdump data buffer */
#ifdef CONFIG_PRINTF_ARG_EXT
      case ('P'):
	len = va_arg(ap, size_t);
	buf = __builtin_alloca(len * 3);
	len = __printf_hexdump(buf, (uint8_t*)val, len);
	break;

	/* string data buffer */

      case ('S'):
	len = va_arg(ap, size_t);
	buf = (char*)val;
# ifndef CONFIG_PRINTF_ARG_SIMPLE
	zeropad = 0;
# endif
	break;
#endif

      default:
	goto printf_state_main;
      }

#ifndef CONFIG_PRINTF_ARG_SIMPLE
    size_t padlen = __MAX((ssize_t)(padding[0] - len), 0);

    if (!rightpad)
      {
	while (padlen--)
	  fcn(ctx, zeropad ? "0" : " ", offset++, 1);
      }
#endif

    fcn(ctx, buf, offset, len);
    offset += len;

#ifndef CONFIG_PRINTF_ARG_SIMPLE
    if (rightpad)
    {
      while (padlen--)
	fcn(ctx, " ", offset++, 1);
    }
#endif
  }
  goto printf_state_main;
}

static inline char n2hex(uint8_t n)
{
    if ( n<10 )
        return n+'0';
    return n-10+'a';
}

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
        line[addrw-1-i] = n2hex(addr & 0xf);
        addr >>= 4;
    }

    memset(line+addrw+3, ' ', w*3);
    memset(line+addrw+3+w*3+3, '.', w);
}

static inline void hexdump_put_char(
    char *line, size_t index, uint8_t val,
    size_t w)
{
    const size_t addrw = 8;

    line[addrw+3+index*3]   = n2hex(val >> 4);
    line[addrw+3+index*3+1] = n2hex(val & 0xf);

    if ( (val >= 32) && (val < 128) )
        line[addrw+3+w*3+3+index] = val;
}

void
mutek_hexdump_arg(void *ctx, printf_output_func_t * const fcn,
                  uintptr_t address, const void *base, size_t size)
{
    const size_t w = 16;
    // addraddr | xx xx [12] xx xx | ..[12]..\n
    static const size_t line_width = 8+3+3*w+3+w+1;
    char line[line_width];

    hexdump_line_init(line, line_width, w);
    hexdump_new_line(line, address & ~(w-1), w);

    const uint8_t *data = base;
    const uint8_t *end = data + size;

    for ( ; data < end; ++data, ++address ) {
        size_t index = (uintptr_t)address % w;
        hexdump_put_char(line, index, *data, w);

        if ( index == w-1 ) {
            fcn(ctx, line, 0, line_width);
            bool_t once = 0;

            while ( memcstcmp(data+1, 0, w) == 0 ) {
                data += w;
                address += w;
                once = 1;
            }

            if ( once ) {
                fcn(ctx, "***\n", 0, 4);
            }

            if ( data+1 < end )
                hexdump_new_line(line, address+1, w);
        }
                      
    }

    if ( (uintptr_t)data & (w-1) )
        fcn(ctx, line, 0, line_width);
}


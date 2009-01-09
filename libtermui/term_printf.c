/*
    This file is part of libtermui.

    libtermui is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    libtermui is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with libtermui.  If not, see <http://www.gnu.org/licenses/>.

    Copyright 2006, Alexandre Becoulet <alexandre.becoulet@free.fr>

*/

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#ifdef HAVE_LOCALTIME_R
#include <time.h>
#endif

#include "term_pv.h"

#define TERM_PRINTF_FCN(f)	ssize_t f(struct term_s *tm, int_fast32_t tag, \
					  void *pdata, uint64_t idata)

typedef TERM_PRINTF_FCN(term_printf_t);

struct				term_printf_s
{
  term_printf_t			*fcn;
  uint8_t			has_pdata:1,
				has_idata:1;
};

static TERM_PRINTF_FCN(term_format_attrib)
{
  term_attrib(tm, tag);

  return 0;
}

#ifdef HAVE_LOCALTIME_R
static TERM_PRINTF_FCN(term_format_date)
{
  time_t	ti = idata;
  struct tm	*t, t_;
  int_fast16_t	res = 0;

  if (ti && (t = localtime_r(&ti, &t_)))
    {
      if (~tag & 1)
	res += stream_nprintf(tm->out, 16, "%02u:%02u:%02u",
			 t->tm_hour, t->tm_min, t->tm_sec);

      if (!tag)
	stream_putc(' ', tm->out);

      if (~tag & 2)
	res += stream_nprintf(tm->out, 16, "%02u/%02u/%02u",
			 t->tm_mon + 1, t->tm_mday, t->tm_year - 100);
    }

  return res;
}
#endif

static TERM_PRINTF_FCN(term_format_string)
{
  char			fmt[16] = "%s";

  if (tag)
    sprintf(fmt, "%%%is", tag);
  return stream_nprintf(tm->out, 1024, fmt, pdata);
}

static TERM_PRINTF_FCN(term_format_int)
{
  char			fmt[16] = "%i";

  if (tag)
    sprintf(fmt, "%%%illi", tag);
  return stream_nprintf(tm->out, 32, fmt, idata);
}

static TERM_PRINTF_FCN(term_format_uint)
{
  char			fmt[16] = "%u";

  if (tag)
    sprintf(fmt, "%%%illu", tag);
  return stream_nprintf(tm->out, 32, fmt, idata);
}

static TERM_PRINTF_FCN(term_format_hexa)
{
  char			fmt[16] = "%x";

  if (tag)
    sprintf(fmt, "%%0%illx", tag);
  return stream_nprintf(tm->out, 32, fmt, idata);
}

static TERM_PRINTF_FCN(term_format_hexastr)
{
  int_fast16_t		i;
  const unsigned char	*h = pdata;

  for (i = 0; i < tag; i++)
    stream_nprintf(tm->out, 8, "%02x", *h++);

  return tag * 2;
}

#if 0				/* no float support yet */
static TERM_PRINTF_FCN(term_format_decimal)
{
  char			fmt[16];
  static const char	prefix[9] = " KMGTPEZY";
  const char		*f = " %s%%.%if ";
  const char		*p = prefix;
  float			val = idata;

  while (val >= 1000)
    {
      f = "%s%%.%if %%c";
      p++;
      val /= 1000;
    }

  if (tag && val >= 100.0f)
    tag--;
  if (tag && val >= 10.0f)
    tag--;

  sprintf(fmt, f, tag ? "" : " ", tag);
  return stream_nprintf(tm->out, 32, fmt, val, *p);
}
#endif

static struct term_printf_s	term_format[255] = 
  {
    ['A'] = { .fcn = term_format_attrib },
#ifdef HAVE_LOCALTIME_R
    ['D'] = { .fcn = term_format_date, .has_idata = 1 },
#endif
#if 0
    ['d'] = { .fcn = term_format_decimal, .has_idata = 1 },
#endif
    ['s'] = { .fcn = term_format_string, .has_pdata = 1 },
    ['i'] = { .fcn = term_format_int, .has_idata = 1 },
    ['u'] = { .fcn = term_format_uint, .has_idata = 1 },
    ['x'] = { .fcn = term_format_hexa, .has_idata = 1 },
    ['m'] = { .fcn = term_format_hexastr, .has_pdata = 1 },
  };

ssize_t term_printf_va(struct term_s *tm,
		   const char *fmt,
		   va_list list)
{
  ssize_t		res = 0;

  while (1)
    {
      switch (*fmt)
	{
	case ('\0'):
	  goto end;

	case ('\n'):
	  term_newline(tm);
	  fmt++;
	  continue;

	case ('%'): {
	  struct term_printf_s	*f;
	  int_fast32_t			tag;
	  void				*pdata = NULL;
	  uint64_t			idata = 0;
	  bool_t			is_long = 0;

	  tag = strtol(fmt + 1, (char**)&fmt, 10);

	  while (*fmt == 'l')
	    {
	      is_long++;
	      fmt++;
	    }

	  if (*fmt == '\0')
	    goto end;

	  f = term_format + *fmt;

	  if (!f->fcn)
	    break;

	  if (f->has_pdata)
	    pdata = va_arg(list, void*);

	  if (f->has_idata)
	    switch (is_long)
	      {
	      case 0:
		idata = va_arg(list, __compiler_sint_t); break;
	      case 1:
		idata = va_arg(list, __compiler_slong_t); break;
	      default: 
		idata = va_arg(list, __compiler_slonglong_t); break;
	      }

	  res += f->fcn(tm, tag, pdata, idata);

	  fmt++;
	  continue;
	}

	default:
	  if (*fmt >= ' ')
	    {
	      stream_putc(*fmt, tm->out);
	      res++;
	    }

	  fmt++;
	}
    }

 end:

  return res;
}

ssize_t term_printf(struct term_s *tm,
		    const char *fmt, ...)
{
  ssize_t	res;
  va_list	list;

  va_start(list, fmt);
  res = term_printf_va(tm, fmt, list);
  va_end(list);

  return res;
}


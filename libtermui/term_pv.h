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

#ifndef TERM_PV_H_
#define TERM_PV_H_

#include <device/char.h>
#include <hexo/device.h>
#include <device/driver.h>

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include <termui/term.h>

/************************************************************************/
/* terminal methods */

#define	TERM_FCN_RESET(f)	error_t f(struct term_s *tm)
#define TERM_FCN_GETSIZE(f)	error_t f(struct term_s *tm, int_fast16_t *x, int_fast16_t *y)
#define	TERM_FCN_MOVE(f)	error_t f(struct term_s *tm, enum term_direction_e dir, int_fast16_t n)
#define	TERM_FCN_SETPOS(f)	error_t f(struct term_s *tm, int_fast16_t x, int_fast16_t y)
#define	TERM_FCN_GETPOS(f)	error_t f(struct term_s *tm, int_fast16_t *x, int_fast16_t *y)
#define TERM_FCN_ATTRIB(f)	error_t f(struct term_s *tm, enum term_attrib_e attr)
#define TERM_FCN_ERASE(f)	error_t f(struct term_s *tm, enum term_direction_e dir)
#define TERM_FCN_BEEP(f)	error_t f(struct term_s *tm)
#define TERM_FCN_ERASELINE(f)	error_t f(struct term_s *tm, enum term_direction_e dir)
#define TERM_FCN_DELCHAR(f)	error_t f(struct term_s *tm, int_fast16_t n)
#define TERM_FCN_DELLINE(f)	error_t f(struct term_s *tm, enum term_direction_e dir, int_fast16_t n)
#define TERM_FCN_INSSTR(f)	error_t f(struct term_s *tm, const char * str, int_fast16_t n)
#define TERM_FCN_WRITESTR(f)	error_t f(struct term_s *tm, const char * str, int_fast16_t n)
#define TERM_FCN_WRITECHAR(f)	error_t f(struct term_s *tm, const char c, int_fast16_t n)
#define TERM_FCN_NEWLINE(f)	error_t f(struct term_s *tm)
#define TERM_FCN_READKEY(f)	error_t f(struct term_s *tm)

typedef	TERM_FCN_RESET(term_reset_t);
typedef TERM_FCN_GETSIZE(term_getsize_t);
typedef	TERM_FCN_MOVE(term_move_t);
typedef	TERM_FCN_SETPOS(term_setpos_t);
typedef	TERM_FCN_GETPOS(term_getpos_t);
typedef TERM_FCN_ATTRIB(term_attrib_t);
typedef TERM_FCN_ERASE(term_erase_t);
typedef TERM_FCN_BEEP(term_beep_t);
typedef TERM_FCN_ERASELINE(term_eraseline_t);
typedef TERM_FCN_DELCHAR(term_delchar_t);
typedef TERM_FCN_DELLINE(term_delline_t);
typedef TERM_FCN_INSSTR(term_insstr_t);
typedef TERM_FCN_WRITESTR(term_writestr_t);
typedef TERM_FCN_WRITECHAR(term_writechar_t);
typedef TERM_FCN_NEWLINE(term_newline_t);
typedef TERM_FCN_READKEY(term_readkey_t);

struct term_methods_s
{
  term_reset_t		*reset; /* mandatory */
  term_getsize_t	*getsize; /* mandatory */
  term_move_t		*move;
  term_setpos_t		*setpos;
  term_getpos_t		*getpos;
  term_attrib_t		*attrib;
  term_erase_t		*erase;
  term_beep_t		*beep;  
  term_eraseline_t	*eraseline;
  term_delchar_t	*delchar;
  term_delline_t	*delline;
  term_insstr_t		*insstr;
  term_writestr_t	*writestr; /* mandatory */
  term_writechar_t	*writechar; /* mandatory */
  term_newline_t	*newline; /* mandatory */
  term_readkey_t	*readkey; /* mandatory */
};

struct term_s
{
  term_stream_t in;
  term_stream_t out;

  struct term_methods_s	mt;

  void *private;
};

#define TERM_TRY(fcn, ...)	(!(fcn) || fcn(__VA_ARGS__))

/************************************************************************/

/* terminal types settings */

error_t term_init(struct term_s *tm, term_stream_t in, term_stream_t out, void *private);
void term_cleanup(struct term_s *tm);

ssize_t term_printf_va(struct term_s *tm, const char *fmt, va_list list);

error_t term_set_none(struct term_s *tm);
error_t term_set_vt100(struct term_s *tm);
error_t term_set_vt102(struct term_s *tm);
error_t term_set_xterm(struct term_s *tm);

static inline ssize_t
stream_nprintf(term_stream_t stream, size_t n, const char *fmt, ...)
{
  char buf[n];
  ssize_t res;
  va_list list;

  va_start(list, fmt);
  res = vsnprintf(buf, n, fmt, list);
  dev_char_wait_write(stream, (void*)buf, res > n ? n : res);
  va_end(list);

  return res;
}

static inline ssize_t
stream_puts(const char *str, term_stream_t stream)
{
  size_t len = strlen(str);
  return dev_char_wait_write(stream, (void*)str, len);
}

static inline ssize_t
stream_putc(char c, term_stream_t stream)
{
  return dev_char_wait_write(stream, (void*)&c, 1);
}

static inline int_fast16_t
stream_getc(term_stream_t stream)
{
  unsigned char c;
  ssize_t res = dev_char_wait_read(stream, &c, 1);
  return res == 1 ? c : -1;
}

static inline int_fast16_t
stream_geti(term_stream_t stream, int_fast16_t *result)
{
  int_fast16_t c, res;

  for (res = 0; (c = stream_getc(stream)) >= '0' && c <= '9'; 
       res = res * 10 + c - '0')
    ;

  *result = res;
  return c;
}

static inline ssize_t
stream_write(term_stream_t stream, const void *data, size_t len)
{
  return dev_char_wait_write(stream, data, len);
}

static inline ssize_t
stream_read(term_stream_t stream, void *data, size_t len)
{
  return dev_char_wait_read(stream, data, len);
}

#endif


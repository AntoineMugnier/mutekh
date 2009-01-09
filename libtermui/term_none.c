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

#include <string.h>

#include <termui/term_keys.h>
#include "term_pv.h"

static TERM_FCN_RESET(term_none_reset)
{
  return 0;
}

static TERM_FCN_GETSIZE(term_none_getsize)
{
  *x = 80;
  *y = 24;

  return 0;
}

static TERM_FCN_WRITESTR(term_none_writestr)
{
  !stream_write(tm->out, str, n);

  return 0;
}

static TERM_FCN_NEWLINE(term_none_newline)
{
  stream_puts("\r\n", tm->out);

  return 0;
}

static TERM_FCN_WRITECHAR(term_none_writechar)
{
  int_fast16_t	i;

  for (i = 0; i < n; i++)
    stream_putc(c, tm->out);

  return 0;
}

static TERM_FCN_READKEY(term_none_readkey)
{
  int_fast16_t k = stream_getc(tm->in);

  if (k >= 0)
    return k % TERM_MAX_KEY;

  return TERM_RET_IOERROR;
}

error_t term_set_none(struct term_s *tm)
{
  memset(&tm->mt, 0, sizeof(tm->mt));

  tm->mt.reset = term_none_reset;
  tm->mt.getsize = term_none_getsize;
  tm->mt.writestr = term_none_writestr;
  tm->mt.writechar = term_none_writechar;
  tm->mt.newline = term_none_newline;
  tm->mt.readkey = term_none_readkey;

  return 0;
}


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

#include "term_pv.h"

#define ESC	"\x1b["

static TERM_FCN_INSSTR(term_vt102_insstr)
{
  stream_puts(ESC "4h", tm->out);
  !stream_write(tm->out, str, n);
  stream_puts(ESC "4l", tm->out);

  return 0;
}

static TERM_FCN_DELCHAR(term_vt102_delchar)
{
  stream_nprintf(tm->out, 32, ESC "%uP", n);

  return 0;
}

error_t term_set_vt102(struct term_s *tm)
{
  term_set_vt100(tm);

  tm->mt.insstr = term_vt102_insstr;
  tm->mt.delchar = term_vt102_delchar;

  return 0;
}


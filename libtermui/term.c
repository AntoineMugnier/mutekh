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
#include <stdlib.h>

#include "term_pv.h"
#include <termui/bhv.h>

error_t term_reset(struct term_s *tm)
{
  return tm->mt.reset(tm);
}

error_t term_getsize(struct term_s *tm, int_fast16_t *x, int_fast16_t *y)
{
  return tm->mt.getsize(tm, x, y);
}

error_t term_move(struct term_s *tm, enum term_direction_e dir, int_fast16_t n)
{
  return tm->mt.move ? tm->mt.move(tm, dir, n) : TERM_RET_INVALID;
}

error_t term_setpos(struct term_s *tm, int_fast16_t x, int_fast16_t y)
{
  return tm->mt.setpos ? tm->mt.setpos(tm, x, y) : TERM_RET_INVALID;
}

error_t term_getpos(struct term_s *tm, int_fast16_t *x, int_fast16_t *y)
{
  return tm->mt.getpos ? tm->mt.getpos(tm, x, y) : TERM_RET_INVALID;
}

error_t term_attrib(struct term_s *tm, enum term_attrib_e attr)
{
  return tm->mt.attrib ? tm->mt.attrib(tm, attr) : TERM_RET_INVALID;
}

error_t term_erase(struct term_s *tm, enum term_direction_e dir)
{
  return tm->mt.erase ? tm->mt.erase(tm, dir) : TERM_RET_INVALID;
}

error_t term_beep(struct term_s *tm)
{
  return tm->mt.beep ? tm->mt.beep(tm) : TERM_RET_INVALID;
}

error_t term_eraseline(struct term_s *tm, enum term_direction_e dir)
{
  return tm->mt.eraseline ? tm->mt.eraseline(tm, dir) : TERM_RET_INVALID;
}

error_t term_delchar(struct term_s *tm, int_fast16_t n)
{
  return tm->mt.delchar ? tm->mt.delchar(tm, n) : TERM_RET_INVALID;
}

error_t term_delline(struct term_s *tm, enum term_direction_e dir, int_fast16_t n)
{
  return tm->mt.delline ? tm->mt.delline(tm, dir, n) : TERM_RET_INVALID;
}

error_t term_insstr(struct term_s *tm, const char * str, int_fast16_t n)
{
  return tm->mt.insstr ? tm->mt.insstr(tm, str, n) : TERM_RET_INVALID;
}

error_t term_writestr(struct term_s *tm, const char * str, int_fast16_t n)
{
  return tm->mt.writestr(tm, str, n);
}

error_t term_writechar(struct term_s *tm, const char c, int_fast16_t n)
{
  return tm->mt.writechar(tm, c, n);
}

error_t term_newline(struct term_s *tm)
{
  return tm->mt.newline(tm);
}

error_t term_readkey(struct term_s *tm)
{
  return tm->mt.readkey(tm);
}

error_t
term_init(struct term_s *tm, 
	  term_stream_t in, term_stream_t out,
	  void *private)
{
  tm->in = in;
  tm->out = out;
  tm->private = private;

  term_set_none(tm);

  return TERM_RET_OK;
}

struct term_s *
term_alloc(term_stream_t in, term_stream_t out, void *private)
{
  struct term_s *tm;

  if ((tm = malloc(sizeof (struct term_s))))
    term_init(tm, in, out, private);

  return tm;
}

error_t
term_set(struct term_s *tm,
	 const char *type)
{
  term_set_none(tm);

  if (!type)
    return TERM_RET_INVALID;

  if (!strcmp(type, "xterm") || !strcmp(type, "rxvt"))
    return term_set_xterm(tm);

  if (!strcmp(type, "vt100"))
    return term_set_vt100(tm);

  if (!strcmp(type, "vt102"))
    return term_set_vt102(tm);

  return TERM_RET_INVALID;
}

void
term_free(struct term_s *tm)
{
  free(tm);
}

void *term_private(struct term_s *tm)
{
  return tm->private;
}

/* behavior process */

error_t
term_behave(struct term_behavior_s *bhv)
{
  struct term_s		*tm = bhv->tm;
  int_fast16_t		k;

  /* pre-initialise context */
  if (bhv->bhvstart)
    {
      error_t	res = bhv->bhvstart(bhv);

      if (res < 0)
	return res;
    }

  while (1)
    {
      switch (k = term_readkey(tm))
	{
	default: {
	  term_keyevent_t	*ev = bhv->keyevent[k];

	  if (ev)
	    {
	      int_fast16_t	res = ev(k, bhv);

	      if (res <= 0)
		return res;

	      bhv->lastkey = k;
	    }

	} break;

	case (TERM_RET_IOERROR):
	  return TERM_RET_IOERROR;

	case (TERM_RET_INVALID):
	  break;
	}
    }
}


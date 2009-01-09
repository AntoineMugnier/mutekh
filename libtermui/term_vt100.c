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

#include <termui/term_keys.h>

#include "term_pv.h"

#define ESC	"\x1b["

static TERM_FCN_RESET(term_vt100_reset)
{
  int_fast16_t c;

  stream_puts(ESC "c", tm->out);	/* reset term */
  do {
    c = stream_getc(tm->in);
  } while (c != 'c' && c >= 0);

  if (c < 0)
    return TERM_RET_IOERROR;

  stream_puts(ESC "?7l", tm->out);	/* disable wrap mode */

  return 0;
}

static TERM_FCN_MOVE(term_vt100_move)
{
  if (!n)
    return 0;

  switch (dir)
    {
    case term_dir_up:	stream_nprintf(tm->out, 32, ESC "%uA", n); return 0;
    case term_dir_down:	stream_nprintf(tm->out, 32, ESC "%uB", n); return 0;
    case term_dir_right:stream_nprintf(tm->out, 32, ESC "%uC", n); return 0;
    case term_dir_left:	stream_nprintf(tm->out, 32, ESC "%uD", n); return 0;

    default:
      return -1;
    }
}

static TERM_FCN_SETPOS(term_vt100_setpos)
{
  stream_nprintf(tm->out, 32, ESC "%u;%uH", y, x);

  return 0;
}

static TERM_FCN_GETPOS(term_vt100_getpos)
{ 
  stream_puts(ESC "6n", tm->out);

  if (stream_getc(tm->in) != 27 || stream_getc(tm->in) != 91)
    return TERM_RET_IOERROR;

  if (stream_geti(tm->in, y) != ';')
    return TERM_RET_IOERROR;

  if (stream_geti(tm->in, x) != 'R')
    return TERM_RET_IOERROR;

  return 0;
}

static TERM_FCN_GETSIZE(term_vt100_getsize)
{
  int_fast16_t oldx, oldy;

  return
    term_vt100_getpos(tm, &oldx, &oldy) ||
    term_vt100_move(tm, term_dir_down, 999) ||
    term_vt100_move(tm, term_dir_right, 999) ||
    term_vt100_getpos(tm, x, y) ||
    term_vt100_setpos(tm, oldx, oldy);
}

static TERM_FCN_WRITESTR(term_vt100_writestr)
{
  int_fast16_t		i;

  for (i = 0; i < n; i++)
    switch (str[i])
      {
      case '\n':
	stream_putc('\r', tm->out);
      default:
	stream_putc(str[i], tm->out);
      }

  return 0;
}

static TERM_FCN_ATTRIB(term_vt100_attrib)
{
  switch (attr)
    {
    case term_attr_none:
    case term_attr_under:
    case term_attr_blink:
    case term_attr_reverse:
    case term_attr_bright:
      stream_nprintf(tm->out, 32, ESC "%um", attr);
      return 0;

    default:
      return -1;
    }
}

static TERM_FCN_ERASE(term_vt100_erase)
{
  switch (dir)
    {
    case term_dir_down:	stream_puts(ESC "0J", tm->out); return 0;
    case term_dir_up:	stream_puts(ESC "1J", tm->out); return 0;
    case term_dir_any:	stream_puts(ESC "2J", tm->out); return 0;

    default:
      return -1;
    }
}

static TERM_FCN_BEEP(term_vt100_beep)
{
  stream_putc(007, tm->out);

  return 0;
}

static TERM_FCN_ERASELINE(term_vt100_eraseline)
{
  switch (dir)
    {
    case term_dir_right:stream_puts(ESC "0K", tm->out); return 0;
    case term_dir_left:	stream_puts(ESC "1K", tm->out); return 0;
    case term_dir_any:	stream_puts(ESC "2K", tm->out); return 0;

    default:
      return -1;
    }

  return -1;  
}

static TERM_FCN_NEWLINE(term_vt100_newline)
{
  stream_puts("\r\n", tm->out);

  return 0;
}

static TERM_FCN_READKEY(term_vt100_readkey)
{
  int_fast16_t	k;

  switch (k = stream_getc(tm->in))
    {
    case (-1): return TERM_RET_IOERROR;

    case (033):			/* ESC */

      switch (k = stream_getc(tm->in))
	{
	case (-1): return TERM_RET_IOERROR;

	case ('['):
	case ('O'):

	  switch (stream_getc(tm->in))
	    {
	    case (-1): return TERM_RET_IOERROR;
	    case ('A'): return TERM_KEY_UP;
	    case ('B'): return TERM_KEY_DOWN;
	    case ('C'): return TERM_KEY_RIGHT;
	    case ('D'): return TERM_KEY_LEFT;
	    }
	  break;

	default:
	  return TERM_KEY_META(k);
	}
      break;

    default:
      if (k <= 255)		/* control & ascii codes */
	return k;

      break;
    }

  return TERM_RET_INVALID;
}

error_t term_set_vt100(struct term_s *tm)
{
  tm->mt.reset = term_vt100_reset;
  tm->mt.getsize = term_vt100_getsize;
  tm->mt.writestr = term_vt100_writestr;
  tm->mt.move = term_vt100_move;
  tm->mt.setpos = term_vt100_setpos;
  tm->mt.getpos = term_vt100_getpos;
  tm->mt.attrib = term_vt100_attrib;
  tm->mt.erase = term_vt100_erase;
  tm->mt.beep = term_vt100_beep;
  tm->mt.eraseline = term_vt100_eraseline;
  tm->mt.newline = term_vt100_newline;
  tm->mt.readkey = term_vt100_readkey;

  return 0;
}


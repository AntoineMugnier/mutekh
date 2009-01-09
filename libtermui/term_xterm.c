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

#include <ctype.h>

#include <termui/term_keys.h>

#include "term_pv.h"

#define ESC	"\x1b["

static TERM_FCN_ATTRIB(term_xterm_attrib)
{
  stream_nprintf(tm->out, 32, ESC "%um", attr);

  return 0;
}

static inline int_fast16_t 
term_xterm_fgetkey(term_stream_t i)
{
  int_fast16_t	res = stream_getc(i);

  return res;
}

static TERM_FCN_READKEY(term_xterm_readkey)
{
  int_fast16_t	k;

  switch (k = term_xterm_fgetkey(tm->in))
    {
    case (-1): return TERM_RET_IOERROR;

    case (033):			/* ESC */

      switch (k = term_xterm_fgetkey(tm->in))
	{
	  int_fast16_t	n;

	case (-1): return TERM_RET_IOERROR;

	case ('O'):
	  switch (k = term_xterm_fgetkey(tm->in))
	    {
	    case (-1): return TERM_RET_IOERROR;
	    case ('A'): return TERM_KEY_UP;
	    case ('B'): return TERM_KEY_DOWN;
	    case ('C'): return TERM_KEY_RIGHT;
	    case ('D'): return TERM_KEY_LEFT;
	    case ('P'): return TERM_KEY_FCN(1);
	    case ('Q'): return TERM_KEY_FCN(2);
	    case ('R'): return TERM_KEY_FCN(3);
	    case ('S'): return TERM_KEY_FCN(4);
	    }
	  break;

	case ('['):

	  n = 0;
	  while (isdigit(k = term_xterm_fgetkey(tm->in)))
	    n = n * 10 + k - '0';

	  switch (k)
	    {
	    case (-1): return TERM_RET_IOERROR;
	    case ('A'): return TERM_KEY_UP;
	    case ('B'): return TERM_KEY_DOWN;
	    case ('C'): return TERM_KEY_RIGHT;
	    case ('D'): return TERM_KEY_LEFT;
	    case ('F'): return TERM_KEY_END;
	    case ('H'): return TERM_KEY_HOME;
	    case ('Z'): return TERM_KEY_UNTAB;

	    case ('~'):
	      switch (n)
		{
		case (1): return TERM_KEY_HOME;
		case (2): return TERM_KEY_INSERT;
		case (3): return TERM_KEY_REMOVE;
		case (4): return TERM_KEY_END;
		case (5): return TERM_KEY_PGUP;
		case (6): return TERM_KEY_PGDN;
		case (15): return TERM_KEY_FCN(5);
		case (17): return TERM_KEY_FCN(6);
		case (18): return TERM_KEY_FCN(7);
		case (19): return TERM_KEY_FCN(8);
		case (20): return TERM_KEY_FCN(9);
		case (21): return TERM_KEY_FCN(10);
		case (23): return TERM_KEY_FCN(11);
		case (24): return TERM_KEY_FCN(12);
		}
	      break;
	    }
	  break;

	default:
	  return TERM_KEY_META(k);
	}
      break;

    default:
      if (k >= 0 && k <= 255)		/* control & ascii codes */
	return k;

      break;
    }

  return TERM_RET_INVALID;
}

error_t term_set_xterm(struct term_s *tm)
{
  term_set_vt102(tm);

  tm->mt.attrib = term_xterm_attrib;
  tm->mt.readkey = term_xterm_readkey;

  return 0;
}


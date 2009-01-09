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

#ifndef BHV_H_
#define BHV_H_

#include <hexo/types.h>

#include <termui/term_keys.h>

/************************************************************************/
/* terminal behaviors */

struct			term_s;
struct			term_behavior_s;

#define TERM_FCN_EVENT(f)	int_fast16_t f(struct term_behavior_s *bhv)
#define TERM_FCN_KEYEVENT(f)	int_fast16_t f(int_fast16_t key, struct term_behavior_s *bhv)

typedef TERM_FCN_EVENT(term_event_t);
typedef TERM_FCN_KEYEVENT(term_keyevent_t);

struct			term_behavior_s
{
  struct term_s		*tm;
  void			*bhvctx;
  int_fast16_t		lastkey;

  term_event_t		*bhvstart;
  term_event_t		*resize;
  term_keyevent_t	*keyevent[TERM_MAX_KEY];
};

#endif


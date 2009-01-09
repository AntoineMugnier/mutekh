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
#include <ctype.h>

#include <termui/bhv.h>
#include <termui/term.h>
#include "getline_pv.h"

static TERM_FCN_KEYEVENT(bhv_key_complete)
{
  struct getline_s	*rl = bhv->bhvctx;

  rl->complete(bhv, term_private(bhv->tm));

  return TERM_RET_CONTINUE;
}

error_t getline_complete_init(struct term_behavior_s *bhv,
			      getline_complete_t *f)
{
  struct getline_s	*rl = bhv->bhvctx;

  if (term_move(bhv->tm, term_dir_right, 0) != TERM_RET_OK)
    return TERM_RET_INVALID;

  rl->complete = f;
  bhv->keyevent[TERM_KEY_HT] = bhv_key_complete;  

  return TERM_RET_OK;
}


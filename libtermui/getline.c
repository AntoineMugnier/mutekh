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

#include <stdlib.h>
#include <string.h>

#include <termui/bhv.h>
#include <termui/term.h>

#include "getline_pv.h"

/* insert string at cursor position */

error_t
getline_insert(struct term_behavior_s *bhv,
	       const char *str, int_fast16_t len)
{
  struct term_s		*tm = bhv->tm;
  struct getline_s	*rl = bhv->bhvctx;
  int_fast16_t		tail = rl->end - rl->cursor;

  /* check for buffer overflow */
  if (len > rl->max - rl->end)
    {
      term_beep(tm);

      if (!(len = rl->max - rl->end))
	return TERM_RET_INVALID;
    }

  if (!tail)
    {
      term_writestr(tm, str, len);
    }
  else if (term_insstr(tm, str, len) != TERM_RET_OK)
    {
      term_writestr(tm, str, len);
      term_writestr(tm, rl->cursor, tail);
      term_move(tm, term_dir_left, tail);
    }

  memmove(rl->cursor + len, rl->cursor, tail + 1);
  memcpy(rl->cursor, str, len);

  rl->end += len;
  rl->cursor += len;

  return TERM_RET_OK;
}

/* delete string at cursor position */

error_t
getline_delete(struct term_behavior_s *bhv,
	       int_fast16_t len)
{
  struct term_s		*tm = bhv->tm;
  struct getline_s	*rl = bhv->bhvctx;
  int_fast16_t		tail = rl->end - rl->cursor;

  /* are we at end of line ? */

  if (!len || len > tail)
    {
      term_beep(tm);
      return TERM_RET_INVALID;
    }

  /* shift line content */
  if (term_delchar(tm, len))
    {
      term_writestr(tm, rl->cursor + len, tail - len);
      term_writechar(tm, ' ', len);
      term_move(tm, term_dir_left, tail);
    }

  memmove(rl->cursor, rl->cursor + len, tail - len + 1);
  rl->end -= len;

  return TERM_RET_OK;
}

error_t
getline_move_forward(struct term_behavior_s *bhv,
		     int_fast16_t len)
{
  struct term_s		*tm = bhv->tm;
  struct getline_s	*rl = bhv->bhvctx;
  int_fast16_t	tail = rl->end - rl->cursor;

  if (!len || len > tail)
    {
      term_beep(tm);
      return TERM_RET_INVALID;
    }

  rl->cursor += len;
  term_move(tm, term_dir_right, len);

  return TERM_RET_OK;
}

error_t
getline_move_backward(struct term_behavior_s *bhv,
			   int_fast16_t len)
{
  struct term_s		*tm = bhv->tm;
  struct getline_s	*rl = bhv->bhvctx;
  int_fast16_t	head = rl->cursor - rl->buf;

  if (!len || len > head)
    {
      term_beep(tm);
      return TERM_RET_INVALID;
    }

  rl->cursor -= len;
  term_move(tm, term_dir_left, len);

  return TERM_RET_OK;
}

void
getline_empty(struct term_behavior_s *bhv)
{
  struct term_s		*tm = bhv->tm;
  struct getline_s	*rl = bhv->bhvctx;
  int_fast16_t	head = rl->cursor - rl->buf;
  int_fast16_t	len = rl->end - rl->buf;

  term_move(tm, term_dir_left, head);

  if (term_eraseline(tm, term_dir_right))
    {
      term_writechar(tm, ' ', len);
      term_move(tm, term_dir_left, len);
    }
}

void
getline_rewrite(struct term_behavior_s *bhv)
{
  struct term_s		*tm = bhv->tm;
  struct getline_s	*rl = bhv->bhvctx;
  int_fast16_t	head = rl->cursor - rl->buf;
  int_fast16_t	len = rl->end - rl->buf;

  term_writestr(tm, rl->buf, len);  
  term_move(tm, term_dir_left, len - head);
}

void
getline_reprompt(struct term_behavior_s *bhv)
{
  struct term_s		*tm = bhv->tm;
  struct getline_s	*rl = bhv->bhvctx;

  /* rewrite prompt */
  rl->prompt(tm, term_private(tm));

  /* rewrite line content */
  getline_rewrite(bhv);
}

/* initialize context for new line reading */

static TERM_FCN_EVENT(getline_bhvstart)
{
  struct term_s		*tm = bhv->tm;
  struct getline_s	*rl = bhv->bhvctx;

  /* allocate new line buffer if none available */
  if (!rl->line && !(rl->line = malloc(rl->size)))
    return TERM_RET_IOERROR;

  rl->max = rl->line + rl->size - 1;
  rl->cursor = rl->end = rl->buf = rl->line;
  rl->buf[0] = '\0';

  if (rl->hist)
    {
      rl->hist[0] = rl->line;
      rl->hist_cur = 0;
    }

#if 0
  if (term_getsize(tm, &rl->width, &height))
    rl->width = 80;

  rl->offset = rl->prompt(tm, term_private(tm)) % rl->width;
#else
  rl->prompt(tm, term_private(tm));
#endif

  return TERM_RET_OK;
}

const char *
getline_line_start(struct term_behavior_s *bhv)
{
  struct getline_s	*rl = bhv->bhvctx;

  return rl->buf;
}

const char *
getline_line_cursor(struct term_behavior_s *bhv)
{
  struct getline_s	*rl = bhv->bhvctx;

  return rl->cursor;
}

static GETLINE_FCN_PROMPT(getline_default_prompt)
{
  return 0;
}

error_t
getline_init(struct term_s *tm,
	     struct term_behavior_s *bhv,
	     int_fast16_t size)
{
  struct getline_s	*rl;

  /* allocate readline context */

  if (!(rl = malloc(sizeof(struct getline_s))))
    return TERM_RET_IOERROR;

  bhv->tm = tm;
  bhv->bhvctx = rl;

  bhv->bhvstart = getline_bhvstart;
  bhv->lastkey = TERM_RET_INVALID;

  rl->size = size;
  rl->copy = NULL;
  rl->prompt = getline_default_prompt;

  rl->line = NULL;
  rl->hist = NULL;

  /* define base readline keys actions */

  memset(bhv->keyevent, 0, sizeof(bhv->keyevent));

  getline_edit_init(bhv);

  return TERM_RET_OK;
}

struct term_behavior_s * getline_alloc(struct term_s *tm,
					int_fast16_t size)
{
  struct term_behavior_s	*bhv;

  if ((bhv = malloc(sizeof (struct term_behavior_s))))
    getline_init(tm, bhv, size);

  return bhv;
}

void
getline_setprompt(struct term_behavior_s *bhv,
		  getline_prompt_t *p)
{
  struct getline_s	*rl = bhv->bhvctx;

  rl->prompt = p;
}

const char *
getline_process(struct term_behavior_s *bhv)
{
  struct getline_s	*rl = bhv->bhvctx;

  switch (term_behave(bhv)) /* use behavior with terminal */
    {
    case TERM_RET_OK:	/* valid line entered */
      return rl->line;

    default:
      return NULL;
    }
}

void
getline_cleanup(struct term_behavior_s *bhv)
{
  struct getline_s	*rl = bhv->bhvctx;

  if (rl->hist)
    getline_history_cleanup(bhv);

  if (rl->copy)
    free(rl->copy);

  if (rl->line)
    free(rl->line);

  free(rl);
}

void getline_free(struct term_behavior_s *bhv)
{
  getline_cleanup(bhv);
  free(bhv);
}


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

static inline int_fast16_t
getline_prev_word(struct getline_s *rl)
{
  char		*word;

  for (word = rl->cursor; word > rl->buf && !isalnum(word[-1]); word--)
    ;

  for ( ; word > rl->buf && isalnum(word[-1]); word--)
    ;

  return rl->cursor - word;
}

static inline int_fast16_t
getline_next_word(struct getline_s *rl)
{
  char		*word = rl->cursor;

  word += strspn(word, " \t");
  word += strcspn(word, " \t");

  return word - rl->cursor;
}

static inline void
getline_copy(struct getline_s *rl,
	     char *str, int_fast16_t len)
{
  if ((rl->copy = realloc(rl->copy, len + 1)))
    {
      memcpy(rl->copy, str, len);
      rl->copy[len] = '\0';
    }
}

static inline void
getline_copy_head(struct getline_s *rl,
		  char *str, int_fast16_t len)
{
  int_fast16_t	clen = 0;

  if (rl->copy)
    clen = strlen(rl->copy);

  if ((rl->copy = realloc(rl->copy, clen + len + 1)))
    {
      memcpy(rl->copy + clen, str, len);
      rl->copy[clen + len] = '\0';
    }
}

static inline void
getline_copy_tail(struct getline_s *rl,
		  char *str, int_fast16_t len)
{
  int_fast16_t	clen = 0;

  if (rl->copy)
    clen = strlen(rl->copy);

  if ((rl->copy = realloc(rl->copy, clen + len + 1)))
    {
      memmove(rl->copy + len, rl->copy, clen);
      memcpy(rl->copy, str, len);
      rl->copy[clen + len] = '\0';
    }
}

/************************************************************************/

/* Return key pressed or CR in input */

static TERM_FCN_KEYEVENT(bhv_key_valid)
{
  struct term_s		*tm = bhv->tm;

  term_newline(tm);

  return TERM_RET_OK;
}

/* Ascii key pressed, adding to buffer */

static TERM_FCN_KEYEVENT(bhv_key_ascii)
{
  char			kchar = key;

  getline_insert(bhv, &kchar, 1);

  return TERM_RET_CONTINUE;
}

/* Left key pressed, move cursor */

static TERM_FCN_KEYEVENT(bhv_key_backward)
{
  getline_move_backward(bhv, 1);

  return TERM_RET_CONTINUE;
}

/* Right key pressed, move cursor */

static TERM_FCN_KEYEVENT(bhv_key_forward)
{
  getline_move_forward(bhv, 1);

  return TERM_RET_CONTINUE;
}

/* Move cursor to end of line */

static TERM_FCN_KEYEVENT(bhv_key_end)
{
  struct getline_s	*rl = bhv->bhvctx;

  getline_move_forward(bhv, rl->end - rl->cursor);

  return TERM_RET_CONTINUE;
}

/* Move cursor to head of line */

static TERM_FCN_KEYEVENT(bhv_key_home)
{
  struct getline_s	*rl = bhv->bhvctx;

  getline_move_backward(bhv, rl->cursor - rl->buf);

  return TERM_RET_CONTINUE;
}

/* Delete next character */

static TERM_FCN_KEYEVENT(bhv_key_delete_char)
{
  getline_delete(bhv, 1);

  return TERM_RET_CONTINUE;
}

/* Go to next word */

static TERM_FCN_KEYEVENT(bhv_key_next_word)
{
  struct getline_s	*rl = bhv->bhvctx;

  getline_move_forward(bhv, getline_next_word(rl));

  return TERM_RET_CONTINUE;
}

/* Go to previous word */

static TERM_FCN_KEYEVENT(bhv_key_prev_word)
{
  struct getline_s	*rl = bhv->bhvctx;

  getline_move_backward(bhv, getline_prev_word(rl));

  return TERM_RET_CONTINUE;
}

/* Delete next word */

static TERM_FCN_KEYEVENT(bhv_key_delete_word)
{
  struct getline_s	*rl = bhv->bhvctx;
  int_fast16_t		len;

  if ((len = getline_next_word(rl)))
    {
      if (key == bhv->lastkey)
	getline_copy_head(rl, rl->cursor, len);
      else
	getline_copy(rl, rl->cursor, len);

      getline_delete(bhv, len);
    }

  return TERM_RET_CONTINUE;
}

/* Delete next word */

static TERM_FCN_KEYEVENT(bhv_key_backspace_word)
{
  struct getline_s	*rl = bhv->bhvctx;
  int_fast16_t		len;

  if ((len = getline_prev_word(rl)))
    {
      getline_move_backward(bhv, len);

      if (key == bhv->lastkey)
	getline_copy_tail(rl, rl->cursor, len);
      else
	getline_copy(rl, rl->cursor, len);

      getline_delete(bhv, len);
    }

  return TERM_RET_CONTINUE;
}

/* Ctrl-D pressed, EOF */

static TERM_FCN_KEYEVENT(bhv_key_eof)
{
  struct term_s		*tm = bhv->tm;

  term_newline(tm);

  return TERM_RET_IOERROR;
}

/* Ctrl-D key pressed, delete or EOF if nothing to delete */

static TERM_FCN_KEYEVENT(bhv_key_eot)
{
  struct term_s		*tm = bhv->tm;
  struct getline_s	*rl = bhv->bhvctx;

  if (rl->buf == rl->end)
    {
      term_newline(tm);
      return TERM_RET_IOERROR;
    }

  getline_delete(bhv, 1);

  return TERM_RET_CONTINUE;
}

/* BackSpace key pressed move backward and delete */

static TERM_FCN_KEYEVENT(bhv_key_backspace)
{
  if (!getline_move_backward(bhv, 1))
    getline_delete(bhv, 1);

  return TERM_RET_CONTINUE;
}

/* Delete characters from cursor to end of line */

static TERM_FCN_KEYEVENT(bhv_key_killtail)
{
  struct getline_s	*rl = bhv->bhvctx;
  int_fast16_t		tail = rl->end - rl->cursor;

  if (tail)
    getline_copy(rl, rl->cursor, tail);

  getline_delete(bhv, tail);

  return TERM_RET_CONTINUE;
}

/* Delete characters to cursor */

static TERM_FCN_KEYEVENT(bhv_key_killhead)
{
  struct getline_s	*rl = bhv->bhvctx;
  int_fast16_t		head = rl->cursor - rl->buf;

  if (head)
    getline_copy(rl, rl->buf, head);

  if (!getline_move_backward(bhv, head))
    getline_delete(bhv, head);

  return TERM_RET_CONTINUE;
}

/* Paste characters present in copy/paste storage */

static TERM_FCN_KEYEVENT(bhv_key_paste)
{
  struct getline_s	*rl = bhv->bhvctx;

  /* paste buffer is empty */
  if (rl->copy)
    getline_insert(bhv, rl->copy, strlen(rl->copy));

  return TERM_RET_CONTINUE;
}

error_t
getline_edit_init(struct term_behavior_s *bhv)
{
  int_fast16_t	k;

  /* define base readline keys actions */

  for (k = 32; k < 127; k++)
    bhv->keyevent[k] = bhv_key_ascii;

  bhv->keyevent[TERM_KEY_RETURN] = bhv_key_valid;
  bhv->keyevent[TERM_KEY_LF] = bhv_key_valid;
  bhv->keyevent[TERM_KEY_EOT] = bhv_key_eof;

  if (term_move(bhv->tm, term_dir_right, 0) != TERM_RET_OK)
    return TERM_RET_OK;

  /* define more readline keys actions */

  bhv->keyevent[TERM_KEY_LEFT] = bhv_key_backward;
  bhv->keyevent[TERM_KEY_STX] = bhv_key_backward;
  bhv->keyevent[TERM_KEY_RIGHT] = bhv_key_forward;
  bhv->keyevent[TERM_KEY_ACK] = bhv_key_forward;

  bhv->keyevent[TERM_KEY_REMOVE] = bhv_key_delete_char;
  bhv->keyevent[TERM_KEY_DELETE] = bhv_key_backspace;

  bhv->keyevent[TERM_KEY_META('d')] = bhv_key_delete_word;
  bhv->keyevent[TERM_KEY_META('D')] = bhv_key_delete_word;

  bhv->keyevent[TERM_KEY_META('b')] = bhv_key_prev_word;
  bhv->keyevent[TERM_KEY_META('B')] = bhv_key_prev_word;

  bhv->keyevent[TERM_KEY_META('f')] = bhv_key_next_word;
  bhv->keyevent[TERM_KEY_META('F')] = bhv_key_next_word;

  bhv->keyevent[TERM_KEY_META(TERM_KEY_DELETE)] = bhv_key_backspace_word;
  bhv->keyevent[TERM_KEY_ETB] = bhv_key_backspace_word;
  bhv->keyevent[TERM_KEY_VT] = bhv_key_killtail;
  bhv->keyevent[TERM_KEY_NAK] = bhv_key_killhead;

  bhv->keyevent[TERM_KEY_EM] = bhv_key_paste;

  bhv->keyevent[TERM_KEY_SOH] = bhv_key_home;
  bhv->keyevent[TERM_KEY_HOME] = bhv_key_home;
  bhv->keyevent[TERM_KEY_ENQ] = bhv_key_end;
  bhv->keyevent[TERM_KEY_END] = bhv_key_end;

  bhv->keyevent[TERM_KEY_EOT] = bhv_key_eot;

  return TERM_RET_OK;
}


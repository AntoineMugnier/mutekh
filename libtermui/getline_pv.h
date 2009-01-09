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

#ifndef GETLINE_PV_H_
#define GETLINE_PV_H_

#include <termui/getline.h>
#include <termui/bhv.h>

struct				getline_s
{
  int_fast16_t			width;		/* term width */
  int_fast16_t			offset;		/* prompt len % term width */
  char				*copy;		/* copy and paste buffer */

  char				**hist;		/* history table */
  int_fast16_t			hist_cur;
  int_fast16_t			hist_count;	/* current entry count */
  int_fast16_t			hist_size;	/* max entry count */

  int_fast16_t			size;		/* line buffer size */
  char				*line;		/* available line buffer */

  /* edit buffer content pointers */
  char				*buf;		/* current edit buffer start */
  char				*cursor;	/* cursor position */
  char				*end;		/* end of current line */
  char				*max;		/* end of max line size */

  getline_complete_t		*complete;
  getline_prompt_t		*prompt;
};

/* internal */

error_t getline_init(struct term_s *tm,
		     struct term_behavior_s *bhv,
		     int_fast16_t size);

void getline_cleanup(struct term_behavior_s *bhv);

error_t getline_edit_init(struct term_behavior_s *bhv);

void getline_empty(struct term_behavior_s *bhv);

void getline_rewrite(struct term_behavior_s *bhv);

#endif


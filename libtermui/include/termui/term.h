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

#ifndef TERM_UI_TERM_H_
#define TERM_UI_TERM_H_

#include <hexo/device.h>

typedef struct device_s * term_stream_t;

#define TERM_RET_CONTINUE	1
#define TERM_RET_OK		0
#define TERM_RET_INVALID	-1
#define TERM_RET_IOERROR	-2

/************************************************************************/
/* terminal methods */

enum term_direction_e
  {
    term_dir_any,
    term_dir_up,
    term_dir_down,
    term_dir_right,
    term_dir_left
  };

enum term_attrib_e
  {
    /* base attributes */
    term_attr_none		= 0,
    term_attr_bright		= 1,
    term_attr_under		= 4,
    term_attr_blink		= 5,
    term_attr_reverse		= 7,

    /* colors */
    term_attr_black		= 30,
    term_attr_red		= 31,
    term_attr_green		= 32,
    term_attr_yellow		= 33,
    term_attr_blue		= 34,
    term_attr_magenta		= 35,
    term_attr_cyan		= 36,
    term_attr_white		= 37,
  };

struct				term_s;
struct				term_behavior_s;

/*
 *  terminal API
 */

struct term_s *term_alloc(term_stream_t in, term_stream_t out, void *private);
error_t term_set(struct term_s *tm, const char *type);
error_t term_behave(struct term_behavior_s *bhv);
void term_free(struct term_s *tm);
void *term_private(struct term_s *tm);

ssize_t term_printf(struct term_s *tm, const char *fmt, ...);

/*
 *  direct terminal access
 */

error_t term_reset(struct term_s *tm);
error_t term_getsize(struct term_s *tm, int_fast16_t *x, int_fast16_t *y);
error_t term_move(struct term_s *tm, enum term_direction_e dir, int_fast16_t n);
error_t term_setpos(struct term_s *tm, int_fast16_t x, int_fast16_t y);
error_t term_getpos(struct term_s *tm, int_fast16_t *x, int_fast16_t *y);
error_t term_attrib(struct term_s *tm, enum term_attrib_e attr);
error_t term_erase(struct term_s *tm, enum term_direction_e dir);
error_t term_beep(struct term_s *tm);
error_t term_eraseline(struct term_s *tm, enum term_direction_e dir);
error_t term_delchar(struct term_s *tm, int_fast16_t n);
error_t term_delline(struct term_s *tm, enum term_direction_e dir, int_fast16_t n);
error_t term_insstr(struct term_s *tm, const char * str, int_fast16_t n);
error_t term_writestr(struct term_s *tm, const char * str, int_fast16_t n);
error_t term_writechar(struct term_s *tm, const char c, int_fast16_t n);
error_t term_newline(struct term_s *tm);
error_t term_readkey(struct term_s *tm);

/*
 * telnet protocol
 */

/* send basic telnet client initialisation commands */

error_t term_telnet_send_setup(struct term_s *tm);

/* add telnet protocol response handling to behavior */

error_t term_telnet_bhv_init(struct term_behavior_s *bhv);

#endif


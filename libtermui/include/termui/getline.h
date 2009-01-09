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

#ifndef TERM_UI_GETLINE_H_
#define TERM_UI_GETLINE_H_

struct term_s;
struct term_behavior_s;


/** Create a getline behavior for specified terminal, with given
    maximum line buffer size */
struct term_behavior_s * getline_alloc(struct term_s *tm, int_fast16_t size);

/** Destroy getline behavior object */
void getline_free(struct term_behavior_s *bhv);

/** Wait for user input and return edited line */
const char *getline_process(struct term_behavior_s *bhv);



/** Prompt display function prototype. Get pointer to terminal object
    and associated private data */
#define GETLINE_FCN_PROMPT(f) int_fast16_t f(struct term_s *tm, void *private)
typedef GETLINE_FCN_PROMPT(getline_prompt_t);

/** Set prompt display function */
void getline_setprompt(struct term_behavior_s *bhv, getline_prompt_t *p);



/** Enable user entered lines history support */
error_t getline_history_init(struct term_behavior_s *bhv, int_fast16_t size);

/** Append a line to history */
error_t getline_history_add(struct term_behavior_s *bhv, const char *str);

/** Get a line from history */
const char * getline_history_get(struct term_behavior_s *bhv, int_fast16_t index);

/** Append the last entered line to history */
error_t getline_history_addlast(struct term_behavior_s *bhv);

/** Clear all history */
void getline_history_cleanup(struct term_behavior_s *bhv);



/** Completion list function prototype. Get pointer to getline object
    and associated private data */
#define GETLINE_FCN_COMPLETE(f)	void f(struct term_behavior_s *bhv, void *private)
typedef GETLINE_FCN_COMPLETE(getline_complete_t);

/** Enable completion support through specified completion callback function */
error_t getline_complete_init(struct term_behavior_s *bhv, getline_complete_t *f);

/** Get line start pointer in current line buffer. Can be called from completion callback */
const char *getline_line_start(struct term_behavior_s *bhv);

/** Get cursor pointer in current line buffer. Can be called from completion callback */
const char *getline_line_cursor(struct term_behavior_s *bhv);

/** Insert text at given cursor position. Can be called from completion callback */
error_t getline_insert(struct term_behavior_s *bhv, const char *str, int_fast16_t len);

/** Rewrite prompt after completion list display. Can be called from completion callback  */
void getline_reprompt(struct term_behavior_s *bhv);

error_t getline_delete(struct term_behavior_s *bhv, int_fast16_t len);

error_t getline_move_forward(struct term_behavior_s *bhv, int_fast16_t len);

error_t getline_move_backward(struct term_behavior_s *bhv, int_fast16_t len);

#endif


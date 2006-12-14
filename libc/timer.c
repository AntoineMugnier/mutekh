/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Matthieu Bucchianeri <matthieu.bucchianeri@epita.fr> (c) 2006

*/

/* enable orhpan checking in this file */
#define GPCT_ORPHAN_CHECK

#include <timer.h>
#include <hexo/types.h>
#include <hexo/error.h>

#include <hexo/gpct_platform_hexo.h>
#include <hexo/gpct_lock_hexo.h>
#include <gpct/cont_blist.h> /* XXX change to b-list */

CONTAINER_FUNC(inline, timer, DLIST, timer, HEXO_SPIN);

/*
 * add a delay timer.
 */

error_t			timer_add_event(struct timer_s		*timer,
					struct timer_event_s	*event)
{
  struct timer_event_s	*e;
  timer_delay_t		t;
  error_t		err = 0;

  event->timer = timer;

  /* set the start time */
  t = event->start = timer->ticks;
  t += event->delay;

  /* insert the timer in the list (sorted) */
  for (e = timer_head(&timer->root);
       e != NULL && t > e->start + e->delay;
       e = timer_next(&timer->root, e))
    ;
  if (e == NULL)
    err = !timer_pushback(&timer->root, event);
  else
    err = !timer_insert_pre(&timer->root, e, event);

  return 0;
}

/*
 * cancel a timer.
 */

error_t	timer_cancel_event(struct timer_event_s	*event,
			   bool_t		callback)
{
  /* remove the timer */
  if (timer_remove(&event->timer->root, event))
    return -ENOENT;

  /* perform the callback if requested by user */
  if (callback)
    event->callback(event, event->pv);

  return 0;
}

/*
 * called by the clock to increment the current tick count.
 */

void			timer_inc_ticks(struct timer_s		*timer,
					timer_delay_t		ticks)
{
  struct timer_event_s	*event;

  /* adjust current tick count */
  timer->ticks += ticks;

  /* XXX use FOREACH ?! */
  /* the timers are sorted by expire time */
  while ((event = timer_head(&timer->root)) != NULL &&
	 event->start + event->delay <= timer->ticks)
    {
      /* remove the timer */
      timer_pop(&timer->root);

      /* perform the callback */
      event->callback(event, event->pv);
    }
}

/*
 * get the current tick count.
 */

inline timer_delay_t	timer_get_tick(struct timer_s		*timer)
{
  return timer->ticks;
}

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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2007
    Copyright Nicola Pouillon <nipo@ssji.net> (c) 2007

*/

#include "srl-priv.h"

void srl_lock_reset(srl_lock_t l)
{
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  sched_queue_wrlock(&l->wait);

  while (sched_wake(&l->wait))
    l->count--;

  sched_queue_unlock(&l->wait);
  CPU_INTERRUPT_RESTORESTATE;
}

void srl_lock_lock(srl_lock_t l)
{
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  sched_queue_wrlock(&l->wait);

  /* check current mutex state */
  if (l->count)
    {
      /* add current thread in mutex wait queue */
      sched_wait_unlock(&l->wait);
    }
  else
    {
      /* mark mutex as used */
      l->count++;
      sched_queue_unlock(&l->wait);
    }

  CPU_INTERRUPT_RESTORESTATE;
}

void srl_lock_unlock(srl_lock_t l)
{
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  sched_queue_wrlock(&l->wait);

  if (!sched_wake(&l->wait))
    l->count--;

  sched_queue_unlock(&l->wait);
  CPU_INTERRUPT_RESTORESTATE;
}

void srl_lock_lock_b(srl_lock_t l)
{
  CPU_INTERRUPT_SAVESTATE_DISABLE;

  sched_queue_wrlock(&l->wait);

  /* check current mutex state */
  while (l->count)
    {
      sched_queue_unlock(&l->wait);
      sched_queue_wrlock(&l->wait);
    }

  /* mark mutex as used */
  l->count++;
  sched_queue_unlock(&l->wait);

  CPU_INTERRUPT_RESTORESTATE;
}

error_t srl_lock_trylock(srl_lock_t l)
{
  error_t	res = 1;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  sched_queue_wrlock(&l->wait);

  /* check current mutex state */
  if (!l->count)
    {
      l->count++;
      res = 0;
    }

  sched_queue_unlock(&l->wait);
  CPU_INTERRUPT_RESTORESTATE;

  return res;
}

#endif

void srl_mwmr_reset(srl_mwmr_t mwmr)
{
}

#ifdef CONFIG_SRL_MWMR_SOFTONLY

void srl_mwmr_read(srl_mwmr_t mwmr, void *data, srl_word_t size)
{
  struct srl_mwmr_status_s fifo = mwmr->status;
  fifo_mwmr_word_t *ptr = mem;

  CPU_INTERRUPT_SAVESTATE_DISABLE;

  while (size > 0)
    {
      srl_word_t *data;

      /* wait for fifo not empty */

      sched_queue_wrlock(&fifo->wait);

      if (!fifo->usage)
	{
	  sched_wait_unlock(&fifo->wait);
	  sched_queue_wrlock(&fifo->wait);
	}

      /* get pointer to data and update fifo state */

      data = mwmr->buffer + fifo->readp;
      fifo->readp += fifo->width;

      if (fifo->readp == fifo->depth)
	fifo->readp = 0;

      /* copy data from fifo */

      memcpy(ptr, data, fifo->width * sizeof(fifo_mwmr_word_t));
      ptr += fifo->width;
      count--;

      /* wake up exactly one writing thread if fifo was full */

      if (fifo->usage-- == fifo->depth)
	sched_wake(&fifo->wait);

      sched_queue_unlock(&fifo->wait);
    }

  CPU_INTERRUPT_RESTORESTATE;
}

void srl_mwmr_write(srl_mwmr_t mwmr, const void *data, srl_word_t size)
{
}

srl_word_t srl_mwmr_try_read(srl_mwmr_t mwmr, void *data, srl_word_t size)
{
}

srl_word_t srl_mwmr_try_write(srl_mwmr_t mwmr, const void *data, srl_word_t size)
{
}

#endif

void srl_barrier_reset(srl_barrier_t t)
{
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  sched_queue_wrlock(&t->wait);

  while (sched_wake(&t->wait) != NULL)
    t->count++;

  sched_queue_unlock(&l->wait);
  CPU_INTERRUPT_RESTORESTATE;
}

void srl_barrier_wait(srl_barrier_t t)
{
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  sched_queue_wrlock(&t->wait);

  assert(t->count >= 1);

  if (t->count == 1)
    {
      while (sched_wake(&t->wait) != NULL)
	t->count++;
      sched_queue_unlock(&t->wait);
    }
  else
    {
      t->count--;
      sched_wait_unlock(&t->wait);
    }

  CPU_INTERRUPT_RESTORESTATE;
}


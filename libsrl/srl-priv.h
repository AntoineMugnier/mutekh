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

#ifndef SRL_PRIV_H_
#define SRL_PRIV_H_

#include <srl.h>

struct srl_memspace_s
{
  void *base;
  size_t size;
};

#ifndef CONFIG_SRL_SPINLOCKS

struct srl_lock_s
{
  uint_fast8_t		count;
  sched_queue_root_t	wait;
};

struct srl_mwmr_status_s
{
#ifdef CONFIG_SRL_MWMR_HARWARE
  srl_lock_t lock;
#endif
#ifdef CONFIG_SRL_MWMR_SOFTONLY
  sched_queue_root_t wait;
#endif
  srl_word_t usage;
  srl_word_t readp;
  srl_word_t writep;
};

struct srl_mwmr_s
{
  struct srl_mwmr_status_s	*status;
  srl_word_t			width;
  srl_word_t			depth;
  srl_word_t			*buffer;
};

struct srl_barrier_s
{
  int_fast8_t count;
  /** blocked threads waiting for read */
  sched_queue_root_t wait;
};


#endif


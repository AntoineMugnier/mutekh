/*
    This file is part of MutekH.
    
    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.
    
    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#include <mutek/kroutine.h>

extern inline bool_t kroutine_exec(struct kroutine_s *kr);

#ifdef CONFIG_MUTEK_KROUTINE_TRIGGER
extern inline bool_t kroutine_trigger(struct kroutine_s *kr, enum kroutine_policy_e policy);
#endif

#ifdef CONFIG_MUTEK_KROUTINE_QUEUE
GCT_CONTAINER_PROTOTYPES(kroutine_list, extern inline, kroutine_list,
                         init, destroy, head, pushback, pop, wrlock, unlock);

GCT_CONTAINER_PROTOTYPES(kroutine_list, extern inline, kroutine_list_nolock,
                         head, pushback, pop, remove);

bool_t kroutine_queue_process(struct kroutine_queue_s *queue)
{
  struct kroutine_s *kr = kroutine_list_pop(&queue->list);
  bool_t done = kr != NULL;
  if (done)
    {
      kr->queue = queue;
      atomic_set(&kr->state, KROUTINE_INVALID);
      kr->exec(kr, KROUTINE_EXEC_DEFERRED);
    }
  return done;
}

#endif

#ifdef CONFIG_MUTEK_KROUTINE_SEMAPHORE

# include <mutek/semaphore.h>

bool_t kroutine_queue_wait(struct kroutine_queue_s *queue)
{
  semaphore_take(queue->sem, 1);
  return kroutine_queue_process(queue);
}

#endif


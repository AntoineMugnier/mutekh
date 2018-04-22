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

extern inline bool_t
kroutine_exec_flags(struct kroutine_s *kr, uint8_t user_flags);

#ifdef CONFIG_MUTEK_KROUTINE_TRIGGER
extern inline bool_t
kroutine_trigger(struct kroutine_s *kr, enum kroutine_policy_e policy);

extern inline void
kroutine_init_trigger(struct kroutine_s *kr,
                      kroutine_exec_t *exec);

extern inline bool_t
kroutine_postpone(struct kroutine_s *kr, enum kroutine_policy_e policy);

#endif

#ifdef CONFIG_MUTEK_KROUTINE_SCHED
extern inline void
kroutine_init_deferred(struct kroutine_s *kr,
                       kroutine_exec_t *exec);

extern inline void
kroutine_init_deferred_local(struct kroutine_s *kr,
                             kroutine_exec_t *exec);

extern inline void
kroutine_init_deferred_seq(struct kroutine_s *kr,
                           kroutine_exec_t *exec,
                           struct kroutine_sequence_s *seq);

extern inline void
kroutine_init_sched_switch(struct kroutine_s *kr,
                           kroutine_exec_t *exec);

extern inline void
kroutine_init_sched_switch_local(struct kroutine_s *kr,
                                 kroutine_exec_t *exec);

extern inline void
kroutine_init_sched_switch_seq(struct kroutine_s *kr,
                               kroutine_exec_t *exec,
                               struct kroutine_sequence_s *seq);

extern inline void
kroutine_init_interruptible(struct kroutine_s *kr,
                            kroutine_exec_t *exec);

extern inline void
kroutine_init_interruptible_local(struct kroutine_s *kr,
                                  kroutine_exec_t *exec);

extern inline void
kroutine_init_interruptible_seq(struct kroutine_s *kr,
                                kroutine_exec_t *exec,
                                struct kroutine_sequence_s *seq);
#endif

#ifdef CONFIG_MUTEK_KROUTINE_IDLE

extern inline void
kroutine_init_idle(struct kroutine_s *kr,
                   kroutine_exec_t *exec);

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
      atomic_fast8_set(&kr->state, KROUTINE_INVALID);
      kr->exec(kr, KROUTINE_EXEC_DEFERRED);
    }
  return done;
}

extern inline void
kroutine_init_queue(struct kroutine_s *kr,
                    kroutine_exec_t *exec,
                    struct kroutine_queue_s *queue);

extern inline error_t
kroutine_queue_init(struct kroutine_queue_s *q,
                    struct semaphore_s *sem);

extern inline void
kroutine_queue_cleanup(struct kroutine_queue_s *q);

extern inline error_t
kroutine_seq_init(struct kroutine_sequence_s *seq);

#endif

#ifdef CONFIG_MUTEK_KROUTINE_SEMAPHORE

# include <mutek/semaphore.h>

bool_t kroutine_queue_wait(struct kroutine_queue_s *queue)
{
  semaphore_take(queue->sem, 1);
  return kroutine_queue_process(queue);
}

#endif

#if defined(CONFIG_ARCH_SMP) && defined(CONFIG_MUTEK_KROUTINE_SCHED)

#include <device/class/cpu.h>

void kroutine_init_deferred_cpu(struct kroutine_s *kr,
                                kroutine_exec_t *exec,
                                struct cpu_tree_s *cpu)
{
  kr->policy = KROUTINE_CPU_DEFERRED;
  kr->exec = exec;
  atomic_fast8_set(&kr->state, KROUTINE_INVALID);
  kr->cls = cpu->cls;
#if CONFIG_MUTEK_SCHED_PRIORITIES > 1
  kr->priority = KROUTINE_DEFAULT_PRIORITY;
#endif
}

void kroutine_init_sched_switch_cpu(struct kroutine_s *kr,
                                    kroutine_exec_t *exec,
                                    struct cpu_tree_s *cpu)
{
  kr->policy = KROUTINE_CPU_SCHED_SWITCH;
  kr->exec = exec;
  atomic_fast8_set(&kr->state, KROUTINE_INVALID);
  kr->cls = cpu->cls;
#if CONFIG_MUTEK_SCHED_PRIORITIES > 1
  kr->priority = KROUTINE_DEFAULT_PRIORITY;
#endif
}

void kroutine_init_interruptible_cpu(struct kroutine_s *kr,
                                     kroutine_exec_t *exec,
                                     struct cpu_tree_s *cpu)
{
  kr->policy = KROUTINE_CPU_INTERRUPTIBLE;
  kr->exec = exec;
  atomic_fast8_set(&kr->state, KROUTINE_INVALID);
  kr->cls = cpu->cls;
#if CONFIG_MUTEK_SCHED_PRIORITIES > 1
  kr->priority = KROUTINE_DEFAULT_PRIORITY;
#endif
}

#endif

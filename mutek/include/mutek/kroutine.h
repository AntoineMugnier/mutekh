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

#ifndef MUTEK_KROUTINE_H_
#define MUTEK_KROUTINE_H_

/**
   @file
   @module{Mutek}
   @short Kernel deferred routines

   This header provides a deferred callback function service where the
   actual execution of the function may be scheduled for execution at
   various times depending on the policy requested along with the
   callback function pointer.

   This is mainly used to implement deferred interrupt processing and
   various device request termination callbacks.

   The @ref kroutine_init function is used to initialize a routine and
   the @ref kroutine_exec function must be called when the routine
   have to be executed or scheduled for execution. The @ref
   kroutine_policy_e enum specifies the various available execution
   policies.

   The interruptible parameter indicates if the function is called
   from an interruptible context. The information must be passed
   initially when calling the @ref kroutine_exec function but deferred
   execution policy may result in a call to the the actual routine
   with a different value of the interruptible parameter.
*/

#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_clist.h>

/** @see kroutine_exec_t */
#define KROUTINE_EXEC(n) void (n) (void *kr, bool_t interruptible)
/** Actual routine callback function. */
typedef KROUTINE_EXEC(kroutine_exec_t);

enum kroutine_policy_e
{
  /** The routine is executed immediately when the the @ref
      kroutine_exec function is called. The value of the @tt
      interruptible parameter is passed directly to the callback. */
  KROUTINE_IMMEDIATE,

  /** If the @ref kroutine_exec function is called with the @tt
      interruptible parmeter set, the routine is executed immediately.

      In the other case, the routine is scheduled for execution on the
      next context switch. The routine will be executed with the
      interruptible parameter set on the stack of the processor idle
      context. The execution will take place on the processor where
      the @ref kroutine_exec function has been called.

      If the @ref #CONFIG_MUTEK_KROUTINE_SCHED_SWITCH token is not
      defined, the routine is executed immediately with the @tt
      interruptible parameter cleared.
  */
  KROUTINE_INTERRUPTIBLE,

  /** If the @ref #CONFIG_HEXO_CONTEXT_PREEMPT configuration token is
      not defined, this is the same as @ref KROUTINE_INTERRUPTIBLE.

      In the other case, it has the additional behavior of preempting
      the current context: If the @ref kroutine_exec function is called
      from an interrupt handler, the context switch will occur on
      interrupt return.
  */
  KROUTINE_PREEMPT,

  /** The routine is scheduled for execution on the next context
      switch.  The routine will be executed with interrupts enabled on
      the stack of the processor idle context. The execution will take
      place on the processor where the @ref kroutine_exec function has
      been called.

      If the @ref #CONFIG_MUTEK_KROUTINE_SCHED_SWITCH token is not
      defined, the routine is executed immediately with the @tt
      interruptible parameter cleared.
  */
  KROUTINE_SCHED_SWITCH,

  /** The routine is scheduled for execution when the processor
      becomes idle. The routine will be executed with interrupts
      enabled on the stack of idle context. The execution will take
      place on the processor where the @ref kroutine_exec function has
      been called.

      If the @ref #CONFIG_MUTEK_KROUTINE_IDLE token is not defined,
      the routine is executed immediately with the @tt interruptible
      parameter cleared.
  */
  KROUTINE_IDLE,
};

struct kroutine_s
{
  CONTAINER_ENTRY_TYPE(CLIST) queue_entry;
  kroutine_exec_t             *exec;
  enum kroutine_policy_e       policy;
};

CONTAINER_TYPE       (kroutine_queue, CLIST, struct kroutine_s, queue_entry);
CONTAINER_FUNC       (kroutine_queue, CLIST, static inline, kroutine_queue, queue_entry);

/** @This initializes a routine with the given callback and scheduling policy. */
static inline void kroutine_init(struct kroutine_s *kr,
                                 kroutine_exec_t *exec,
                                 enum kroutine_policy_e policy)
{
  kr->exec = exec;
  kr->policy = policy;
}

/** @This changes the execution policy of a routine. The policy can be
    changed before calling the @ref kroutine_exec or @ref
    kroutine_schedule functions. */
static inline void kroutine_set_policy(struct kroutine_s *kr,
                                       enum kroutine_policy_e policy)
{
  kr->policy = policy;
}

/** @This schedules execution of a routine.

    It is not permitted to schedule execution of a routine multiple
    times. This function must not be called again before the callback
    function is actually executed. It's ok to call this function from
    the callback function itself in order to reschedule a routine.

    This function returns an error code if the current routine policy
    is @ref KROUTINE_IMMEDIATE or if the requested policy is not
    supported with the current configuration.
*/
config_depend_or2(CONFIG_MUTEK_KROUTINE_SCHED_SWITCH, CONFIG_MUTEK_KROUTINE_IDLE)
error_t kroutine_schedule(struct kroutine_s *kr, bool_t interruptible);

/** @This either schedules the given routine by calling @ref
    kroutine_schedule or executes the routine immediately if it's not
    possible to defer its execution. This function returns @em true if
    the routine has been executed immediately. */
static inline bool_t kroutine_exec(struct kroutine_s *kr, bool_t interruptible)
{
  bool_t r = 1;
# if defined(CONFIG_MUTEK_KROUTINE_SCHED_SWITCH) || defined(CONFIG_MUTEK_KROUTINE_IDLE)
  r = (kr->policy == KROUTINE_IMMEDIATE || kroutine_schedule(kr, interruptible));
  if (r)
#endif
    kr->exec(kr, interruptible);
  return r;
}

#endif


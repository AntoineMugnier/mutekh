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

   The @tt interruptible parameter indicates if the function is called
   from an interruptible context. This information must be passed
   when calling the @ref kroutine_exec function but chosen
   policy may result in a defered call to the the actual routine
   with a different value of the @tt interruptible parameter.
*/

#include <gct_platform.h>
#include <gct/container_clist.h>

#include <hexo/atomic.h>

struct kroutine_s;

/** @see kroutine_exec_t */
#define KROUTINE_EXEC(n) void (n) (struct kroutine_s *kr, bool_t interruptible)
/** Actual routine callback function. */
typedef KROUTINE_EXEC(kroutine_exec_t);

/** @This can be used to retreive a pointer to the structure
    containing a @ref kroutine_s object. */
#define KROUTINE_CONTAINER(kr, conttype, field) \
  ((typeof(conttype)*)((uint8_t*)kr - \
   __builtin_offsetof(typeof(conttype), field)))

/** @This specify kroutine defered execution policies. */
enum kroutine_policy_e
{
  /** This policy makes the routine execute immediately when the @ref
      kroutine_exec function is called. The value of the @tt
      interruptible parameter is passed directly to the @ref
      kroutine_exec_t function. */
  KROUTINE_IMMEDIATE,

#ifdef CONFIG_MUTEK_KROUTINE_TRIGGER
  /** This policy requires an additional function call in order to
      allow execution of the routine. This may be used to defer
      execution of a routine up to a point in the code where the
      stack has unwound and avoid cases of deep function call nesting.

      The @ref kroutine_exec function and the @ref kroutine_trigger
      function have both to be called in order for the routine to be
      executed. The @ref kroutine_triggered_1st function can be used
      inside the routine to find which function has been called first.
  */
  KROUTINE_TRIGGER,
#endif

#ifdef CONFIG_MUTEK_KROUTINE_SCHED_SWITCH
  /** If the @ref kroutine_exec function is called with the @tt
      interruptible parameter set, the routine is executed immediately.

      In the other case, the routine is scheduled for execution on the
      next context switch. The routine will be executed with the
      interruptible parameter set on the stack of the processor idle
      context. The execution will take place on the processor where
      the @ref kroutine_exec function has been called.
  */
  KROUTINE_INTERRUPTIBLE,

  /** If the @ref #CONFIG_HEXO_CONTEXT_PREEMPT configuration token is
      not defined, this is the same as @ref KROUTINE_INTERRUPTIBLE.

      In the other case, it has the additional behavior of preempting
      the current context: If the @ref kroutine_exec function is called
      from an interrupt handler, a context switch to the processor idle
      context will occur on interrupt return in order to execute the
      kroutine.
  */
  KROUTINE_PREEMPT_INTERRUPTIBLE,

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

  /** If the @ref #CONFIG_HEXO_CONTEXT_PREEMPT configuration token is
      not defined, this is the same as @ref KROUTINE_SCHED_SWITCH.

      In the other case, it has the additional behavior of preempting
      the current context: If the @ref kroutine_exec function is called
      from an interrupt handler, a context switch to the processor idle
      context will occur on interrupt return in order to execute the
      kroutine.
  */
  KROUTINE_PREEMPT,

  /** @multiple If the @ref #CONFIG_MUTEK_KROUTINE_SCHED_SWITCH token
      is not defined, the policy defaults to KROUTINE_IMMEDIATE. */
  KROUTINE_INTERRUPTIBLE_OR_IMMEDIATE          = KROUTINE_INTERRUPTIBLE,
  KROUTINE_PREEMPT_INTERRUPTIBLE_OR_IMMEDIATE  = KROUTINE_PREEMPT_INTERRUPTIBLE,
  KROUTINE_SCHED_SWITCH_OR_IMMEDIATE           = KROUTINE_SCHED_SWITCH,
  KROUTINE_PREEMPT_OR_IMMEDIATE                = KROUTINE_PREEMPT,

# ifdef CONFIG_MUTEK_KROUTINE_TRIGGER
  /** @multiple If the @ref #CONFIG_MUTEK_KROUTINE_SCHED_SWITCH token
      is not defined, the policy defaults to KROUTINE_TRIGGER. */
  KROUTINE_INTERRUPTIBLE_OR_TRIGGER            = KROUTINE_INTERRUPTIBLE,
  KROUTINE_PREEMPT_INTERRUPTIBLE_OR_TRIGGER    = KROUTINE_PREEMPT_INTERRUPTIBLE,
  KROUTINE_SCHED_SWITCH_OR_TRIGGER             = KROUTINE_SCHED_SWITCH,
  KROUTINE_PREEMPT_OR_TRIGGER                  = KROUTINE_PREEMPT,     
# endif
#else
  KROUTINE_INTERRUPTIBLE_OR_IMMEDIATE          = KROUTINE_IMMEDIATE,
  KROUTINE_PREEMPT_INTERRUPTIBLE_OR_IMMEDIATE  = KROUTINE_IMMEDIATE,
  KROUTINE_SCHED_SWITCH_OR_IMMEDIATE           = KROUTINE_IMMEDIATE,
  KROUTINE_PREEMPT_OR_IMMEDIATE                = KROUTINE_IMMEDIATE,

# ifdef CONFIG_MUTEK_KROUTINE_TRIGGER
  KROUTINE_INTERRUPTIBLE_OR_TRIGGER            = KROUTINE_TRIGGER,
  KROUTINE_PREEMPT_INTERRUPTIBLE_OR_TRIGGER    = KROUTINE_TRIGGER,
  KROUTINE_SCHED_SWITCH_OR_TRIGGER             = KROUTINE_TRIGGER,
  KROUTINE_PREEMPT_OR_TRIGGER                  = KROUTINE_TRIGGER,
# endif
#endif

#ifdef CONFIG_MUTEK_KROUTINE_IDLE
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

  /** @multiple If the @ref #CONFIG_MUTEK_KROUTINE_IDLE token
      is not defined, the policy defaults to KROUTINE_IMMEDIATE. */
  KROUTINE_IDLE_OR_IMMEDIATE            = KROUTINE_IDLE,
# ifdef CONFIG_MUTEK_KROUTINE_TRIGGER
  /** @multiple If the @ref #CONFIG_MUTEK_KROUTINE_IDLE token
      is not defined, the policy defaults to KROUTINE_TRIGGER. */
  KROUTINE_IDLE_OR_TRIGGER              = KROUTINE_IDLE,
# endif
#else
  KROUTINE_IDLE_OR_IMMEDIATE            = KROUTINE_IMMEDIATE,
# ifdef CONFIG_MUTEK_KROUTINE_TRIGGER
  KROUTINE_IDLE_OR_TRIGGER              = KROUTINE_TRIGGER,
# endif
#endif
};

#define GCT_CONTAINER_ALGO_kroutine_queue CLIST

struct kroutine_s
{
  union {
#if defined(CONFIG_MUTEK_KROUTINE_SCHED_SWITCH) || defined(CONFIG_MUTEK_KROUTINE_IDLE)
    GCT_CONTAINER_ENTRY(kroutine_queue, queue_entry);
#endif
#ifdef CONFIG_MUTEK_KROUTINE_TRIGGER
    atomic_t                    state;
#endif
  };
  kroutine_exec_t              *exec;
  enum kroutine_policy_e       policy;
};

#if defined(CONFIG_MUTEK_KROUTINE_SCHED_SWITCH) || defined(CONFIG_MUTEK_KROUTINE_IDLE)
GCT_CONTAINER_TYPES       (kroutine_queue, struct kroutine_s, queue_entry);
GCT_CONTAINER_FCNS       (kroutine_queue, static inline, kroutine_queue, queue_entry,
                          init, destroy, head, pushback, pop);
#endif

/** @This changes the scheduling policy of a kroutine . */
static inline void kroutine_set_policy(struct kroutine_s *kr,
                                       enum kroutine_policy_e policy)
{
  kr->policy = policy;
#ifdef CONFIG_MUTEK_KROUTINE_TRIGGER
  if (policy == KROUTINE_TRIGGER)
    atomic_set(&kr->state, 0);
#endif
}

/** @This initializes a routine with the given callback and scheduling policy. */
static inline void kroutine_init(struct kroutine_s *kr,
                                 kroutine_exec_t *exec,
                                 enum kroutine_policy_e policy)
{
  kr->exec = exec;
  kroutine_set_policy(kr, policy);
}

/** @This either schedules the given routine by calling @ref
    kroutine_schedule or executes the routine immediately if it's not
    possible to defer its execution. This function returns @em true if
    the routine has been executed immediately. */
static inline bool_t kroutine_exec(struct kroutine_s *kr, bool_t interruptible)
{
  error_t kroutine_schedule(struct kroutine_s *kr, bool_t interruptible);
  bool_t r = 1;

  switch (kr->policy)
    {
#ifdef CONFIG_MUTEK_KROUTINE_TRIGGER
    case KROUTINE_TRIGGER:
      r = !atomic_compare_and_swap(&kr->state, 0, 1);
      if (r)
        atomic_set(&kr->state, 0); /* reset state */
      break;
#endif
#ifdef CONFIG_MUTEK_KROUTINE_IDLE
    case KROUTINE_IDLE:
#endif
#ifdef CONFIG_MUTEK_KROUTINE_SCHED_SWITCH
    case KROUTINE_PREEMPT:
    case KROUTINE_INTERRUPTIBLE:
    case KROUTINE_SCHED_SWITCH:
#endif
#if defined(CONFIG_MUTEK_KROUTINE_SCHED_SWITCH) || defined(CONFIG_MUTEK_KROUTINE_IDLE)
      r = kroutine_schedule(kr, interruptible);
#endif
    default:
      break;
    }
  if (r)
    kr->exec(kr, interruptible);
  return r;
}

/** When the kroutine has been initialized using the @ref
    KROUTINE_TRIGGER policy, this function permit execution of the
    routine by the @ref kroutine_exec function. @This returns 0 if
    the call to @ref kroutine_exec has not been performed yet.

    When a different policy is in use, this function does nothing and
    returns 0.
*/
#ifdef CONFIG_MUTEK_KROUTINE_TRIGGER
static inline bool_t kroutine_trigger(struct kroutine_s *kr, bool_t interruptible)
{
  bool_t r = kr->policy == KROUTINE_TRIGGER &&
    !atomic_compare_and_swap(&kr->state, 0, 2);
  if (r)
    {
      kr->exec(kr, interruptible);
      atomic_set(&kr->state, 0); /* reset state */
    }
  return r;
}
#endif

config_depend(CONFIG_MUTEK_KROUTINE_TRIGGER)
bool_t kroutine_trigger(struct kroutine_s *kr, bool_t interruptible);

/** @This is designed to be called from within the @ref kroutine_exec_t
    handler function.

    This function returns 1 if the @ref kroutine_trigger function has
    been called before the @ref kroutine_exec function. It also returns
    1 when the kroutine policy is not @ref KROUTINE_TRIGGER. */
#ifdef CONFIG_MUTEK_KROUTINE_TRIGGER
static inline bool_t kroutine_triggered_1st(struct kroutine_s *kr)
{
  return kr->policy != KROUTINE_TRIGGER || atomic_get(&kr->state) != 1;
}
#endif

config_depend(CONFIG_MUTEK_KROUTINE_TRIGGER)
bool_t kroutine_triggered_1st(struct kroutine_s *kr);

config_depend(CONFIG_MUTEK_KROUTINE_TRIGGER)
bool_t kroutine_triggered_1st(struct kroutine_s *kr);

#endif



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
  KROUTINE_INVALID = 0,

  /** No routine is executed. */
  KROUTINE_NONE,

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
    atomic_t                    state;
  };
  kroutine_exec_t              *exec;
  enum kroutine_policy_e       policy;
};

#if defined(CONFIG_MUTEK_KROUTINE_SCHED_SWITCH) || defined(CONFIG_MUTEK_KROUTINE_IDLE)
GCT_CONTAINER_TYPES       (kroutine_queue, struct kroutine_s *, queue_entry);
GCT_CONTAINER_FCNS       (kroutine_queue, inline, kroutine_queue,
                          init, destroy, head, pushback, pop);
#endif

/** @This changes the scheduling policy of a kroutine . */
ALWAYS_INLINE void kroutine_set_policy(struct kroutine_s *kr,
                                       enum kroutine_policy_e policy)
{
  kr->policy = policy;
  atomic_set(&kr->state, KROUTINE_INVALID);
}

/** @This initializes a routine with the given callback and scheduling policy. */
ALWAYS_INLINE void kroutine_init(struct kroutine_s *kr,
                                 kroutine_exec_t *exec,
                                 enum kroutine_policy_e policy)
{
  kr->exec = exec;
  kroutine_set_policy(kr, policy);
}

error_t kroutine_schedule(struct kroutine_s *kr, bool_t interruptible,
                          enum kroutine_policy_e policy);

/** @This either schedules execution of the routine or executes the
    routine immediately depending on the policy and value of the @tt
    interruptible argument. This function returns @em true if the
    routine has been executed immediately.

    For policies other than @ref KROUTINE_TRIGGER, it is ok to call
    this function multiple times before the routine is actually
    executed. When deferred execution is used, the routine may be
    executed only once in this case.
*/
inline bool_t kroutine_exec(struct kroutine_s *kr, bool_t interruptible)
{
  enum kroutine_policy_e policy = kr->policy;

  assert(policy != KROUTINE_INVALID);
  switch (policy)
    {
    default:
      UNREACHABLE();
    case KROUTINE_NONE:
      return 0;
#ifdef CONFIG_MUTEK_KROUTINE_TRIGGER
    case KROUTINE_TRIGGER:
      if (atomic_compare_and_swap(&kr->state, KROUTINE_INVALID, KROUTINE_TRIGGER))
        return 0;
      policy = atomic_get(&kr->state);
      assert(policy != KROUTINE_TRIGGER && policy != KROUTINE_NONE);
# if defined(CONFIG_MUTEK_KROUTINE_SCHED_SWITCH) || defined(CONFIG_MUTEK_KROUTINE_IDLE)
      if (policy != KROUTINE_IMMEDIATE)
        goto sched;
# endif
      goto imm;
#endif
#if defined(CONFIG_MUTEK_KROUTINE_SCHED_SWITCH) || defined(CONFIG_MUTEK_KROUTINE_IDLE)
# ifdef CONFIG_MUTEK_KROUTINE_IDLE
    case KROUTINE_IDLE:
# endif
# ifdef CONFIG_MUTEK_KROUTINE_SCHED_SWITCH
    case KROUTINE_PREEMPT:
    case KROUTINE_PREEMPT_INTERRUPTIBLE:
    case KROUTINE_INTERRUPTIBLE:
    case KROUTINE_SCHED_SWITCH:
# endif
      if (!atomic_compare_and_swap(&kr->state, KROUTINE_INVALID, !KROUTINE_INVALID))
        return 0;
    sched:
      if (!kroutine_schedule(kr, interruptible, policy))
        return 0;
#endif
    imm:
      atomic_set(&kr->state, KROUTINE_INVALID); /* reset state */
    case KROUTINE_IMMEDIATE:
      kr->exec(kr, interruptible);
      return 1;
    }
}

/** @This should be used on a kroutine initialized with the @ref
    KROUTINE_TRIGGER policy. If it is not the case, this function does
    nothing and returns @em false.

    @This makes the next call to the @ref kroutine_exec function to
    handle the kroutine using the @tt policy passed as argument. This
    function return @em false if the @ref kroutine_exec function has
    not been called yet.

    If the @ref kroutine_exec function has already been called,
    nothing happened yet and this function will take care of handling
    the kroutine according to the values of the @tt interruptible and
    @tt policy arguments. @em true is returned in this case. */
config_depend_inline(CONFIG_MUTEK_KROUTINE_TRIGGER,
bool_t kroutine_trigger(struct kroutine_s *kr, bool_t interruptible,
                        enum kroutine_policy_e policy),
{
  if (kr->policy != KROUTINE_TRIGGER ||
      atomic_compare_and_swap(&kr->state, KROUTINE_INVALID, policy))
    return 0;

  assert(atomic_get(&kr->state) == KROUTINE_TRIGGER);
  switch (policy)
    {
    default:
      UNREACHABLE();
    case KROUTINE_NONE:
      break;
#if defined(CONFIG_MUTEK_KROUTINE_SCHED_SWITCH) || defined(CONFIG_MUTEK_KROUTINE_IDLE)
# ifdef CONFIG_MUTEK_KROUTINE_IDLE
    case KROUTINE_IDLE:
# endif
# ifdef CONFIG_MUTEK_KROUTINE_SCHED_SWITCH
    case KROUTINE_PREEMPT:
    case KROUTINE_PREEMPT_INTERRUPTIBLE:
    case KROUTINE_INTERRUPTIBLE:
    case KROUTINE_SCHED_SWITCH:
# endif
      if (!kroutine_schedule(kr, interruptible, policy))
        break;
#endif
    case KROUTINE_IMMEDIATE:
      kr->exec(kr, interruptible);
       /* reset state after the call so that the
          kroutine_triggered_1st function can test the state value. */
      atomic_set(&kr->state, KROUTINE_INVALID);
    }
  return 1;
});

/** @This function is similar to @ref kroutine_trigger but does
    nothing when the @ref kroutine_exec function has already been
    called. The routine will only be executed if the call to @ref
    kroutine_exec if performed after the call to this function.

    This can be used to leave then resume a FSM loop when an
    asynchronous operation as not already completed. */
config_depend_alwaysinline(CONFIG_MUTEK_KROUTINE_TRIGGER,
bool_t kroutine_postpone(struct kroutine_s *kr, enum kroutine_policy_e policy),
{
  assert(policy != KROUTINE_TRIGGER && kr->policy == KROUTINE_TRIGGER);
  bool_t r = atomic_compare_and_swap(&kr->state, KROUTINE_INVALID, policy);
  if (!r)
    atomic_set(&kr->state, KROUTINE_INVALID); /* reset state */
  return r;
});

/** @This is designed to be called from within the @ref kroutine_exec_t
    handler function.

    This function returns @em true if the @ref kroutine_trigger
    function has been called before the @ref kroutine_exec
    function. It also returns @em true when the kroutine policy is not
    @ref KROUTINE_TRIGGER. */
config_depend_alwaysinline(CONFIG_MUTEK_KROUTINE_TRIGGER,
bool_t kroutine_triggered_1st(struct kroutine_s *kr),
{
  return kr->policy != KROUTINE_TRIGGER ||
    atomic_get(&kr->state) != KROUTINE_TRIGGER;
});

#endif


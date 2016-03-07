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
   @module{Kernel services}
   @short Kernel deferred routines

   This header provides a deferred callback function service where the
   actual execution of the function may be scheduled for execution at
   various times depending on the policy requested along with the
   callback function pointer.

   This is mainly used to implement deferred interrupt processing and
   various device request termination callbacks.

   The @tt kroutine_init* functions are used to initialize a routine and
   the @ref kroutine_exec function must be called when the routine
   have to be executed or scheduled for execution. The @ref
   kroutine_policy_e enum specifies the various available execution
   policies.
*/

#include <gct_platform.h>
#include <gct_lock_hexo_lock.h>
#include <gct/container_clist.h>

#include <hexo/atomic.h>

struct kroutine_s;
struct kroutine_sequence_s;

/** @This flags are passed to the @ref kroutine_exec_t handler */
enum kroutine_exec_flags_e
{
  /** This indicates that the kroutine handler is executed from the
      @ref kroutine_trigger function. This implies that the
      kroutine_exec function has been called previously and the
      retained policy and context allow immediate execution of the
      kroutine handler. */
  KROUTINE_EXEC_TRIGGERED = 1,
  /** This indicates that the kroutine handler has been deferred and
      is therefore executed from an interruptible context. */
  KROUTINE_EXEC_DEFERRED = 2,
};

/** @see kroutine_exec_t */
#define KROUTINE_EXEC(n) void (n) (struct kroutine_s *kr,               \
                                   enum kroutine_exec_flags_e flags)
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
      kroutine_exec function is called. */
  KROUTINE_IMMEDIATE,

#ifdef CONFIG_MUTEK_KROUTINE_TRIGGER
  /** This policy requires an additional function call in order to
      allow execution of the kroutine handler. This may be used to defer
      execution of the handler up to a point in the code where the
      stack has unwound and avoid cases of deep function call nesting.

      The @ref kroutine_exec function and the @ref kroutine_trigger
      function have both to be called in order for the handler to be
      executed.
  */
  KROUTINE_TRIGGER,
#endif

#ifdef CONFIG_MUTEK_KROUTINE_QUEUE
  /** This policy allows scheduling of a kroutine on a user defined
      queue when @ref kroutine_exec is called. The actual routine
      execution will occur when either the @ref kroutine_queue_process
      or kroutine_queue_wait function is called. */
  KROUTINE_QUEUE,
#endif

#ifdef CONFIG_MUTEK_KROUTINE_SCHED

  /** The kroutine is scheduled for defered execution with interrupts
      enabled on the stack of the idle context. The execution will not
      occur before the next context switch regardless of the
      priorities of the kroutine and current context.

      This policy can be used as soon as the scheduler as been
      initialized, before it is actually started. */
  KROUTINE_SCHED_SWITCH,

  /** The kroutine is scheduled for defered execution with interrupts
      enabled on the stack of the idle context. This will occur as
      soon as possible depending on the priorities of the kroutine and
      current context as well as build configuration.

      The @ref #CONFIG_HEXO_CONTEXT_PREEMPT and @ref
      #CONFIG_HEXO_CONTEXT_IRQEN features allow switching to the idle
      context without waiting for the next context switch.

      This policy can be used as soon as the scheduler as been
      initialized, before it is actually started but the handler
      execution will be postponed. */
  KROUTINE_DEFERRED,

  /** If the @ref kroutine_exec function is called with interrupts
      enabled, the kroutine handler is executed immediately on the
      current stack. In the other case, the kroutine is handled as if
      the policy were @ref KROUTINE_DEFERRED. */
  KROUTINE_INTERRUPTIBLE,

# ifdef CONFIG_MUTEK_KROUTINE_IDLE
  /** The kroutine is scheduled for execution when the last processor
      of the platform becomes idle. The kroutine handler will be
      executed with interrupts enabled on the stack of idle context.

      This policy can be used as soon as the scheduler as been
      initialized, before it is actually started. */
  KROUTINE_IDLE,
# endif

# ifdef CONFIG_ARCH_SMP
  /** This is similar to @ref KROUTINE_INTERRUPTIBLE. On SMP platforms
      the execution is scheduled for execution on a specific
      processor. */
  KROUTINE_CPU_INTERRUPTIBLE,
  /** This is similar to @ref KROUTINE_SCHED_SWITCH. On SMP platforms
      the execution is scheduled for execution on a specific
      processor. */
  KROUTINE_CPU_SCHED_SWITCH,
  /** This is similar to @ref KROUTINE_DEFERRED. On SMP platforms
      the execution is scheduled for execution on a specific
      processor. */
  KROUTINE_CPU_DEFERRED,

  /** This is similar to @ref KROUTINE_INTERRUPTIBLE. Execution of the
      handler is guaranteed not to run concurrently with kroutines
      associated to the same sequence. */
  KROUTINE_SEQ_INTERRUPTIBLE,
  /** This is similar to @ref KROUTINE_SCHED_SWITCH. Execution of the
      handler is guaranteed not to run concurrently with kroutines
      associated to the same sequence. */
  KROUTINE_SEQ_SCHED_SWITCH,
  /** This is similar to @ref KROUTINE_DEFERRED. Execution of the
      handler is guaranteed not to run concurrently with kroutines
      associated to the same sequence. */
  KROUTINE_SEQ_DEFERRED,
# else
  KROUTINE_CPU_INTERRUPTIBLE = KROUTINE_INTERRUPTIBLE,
  KROUTINE_CPU_SCHED_SWITCH = KROUTINE_SCHED_SWITCH,
  KROUTINE_CPU_DEFERRED = KROUTINE_DEFERRED,
  KROUTINE_SEQ_INTERRUPTIBLE = KROUTINE_INTERRUPTIBLE,
  KROUTINE_SEQ_SCHED_SWITCH = KROUTINE_SCHED_SWITCH,
  KROUTINE_SEQ_DEFERRED = KROUTINE_DEFERRED,
# endif

#endif
};

#define GCT_CONTAINER_ALGO_kroutine_list CLIST
#define GCT_CONTAINER_LOCK_kroutine_list HEXO_LOCK

struct semaphore_s;
struct kroutine_queue_s;

#ifdef CONFIG_MUTEK_KROUTINE_QUEUE
GCT_CONTAINER_TYPES       (kroutine_list,
#endif
struct kroutine_s
{
#ifdef CONFIG_MUTEK_KROUTINE_QUEUE
  union {
    GCT_CONTAINER_ENTRY(kroutine_list, queue_entry);
# if defined(CONFIG_ARCH_SMP) && defined(CONFIG_MUTEK_KROUTINE_SCHED)
    void *cls;
# endif
    struct kroutine_queue_s *queue;
  };
#endif
#ifdef CONFIG_ARCH_SMP
  struct kroutine_sequence_s   *seq;
#endif
  kroutine_exec_t              *exec;
  enum kroutine_policy_e       policy;
  atomic_t                     state;
#if CONFIG_MUTEK_SCHED_PRIORITIES > 1 && defined(CONFIG_MUTEK_KROUTINE_SCHED)
  uint8_t                      priority;
#endif
}
#ifdef CONFIG_MUTEK_KROUTINE_QUEUE
                           *, queue_entry)
#endif
;

/** @see kroutine_seq_init */
struct kroutine_sequence_s
{
#ifdef CONFIG_ARCH_SMP
  atomic_t state;
#endif
};

#ifdef CONFIG_MUTEK_KROUTINE_QUEUE
GCT_CONTAINER_FCNS       (kroutine_list, inline, kroutine_list,
                          init, destroy, head, pushback, pop, wrlock, unlock);
GCT_CONTAINER_NOLOCK_FCNS(kroutine_list, inline, kroutine_list_nolock,
                          head, pushback, pop, remove);

/** User defined kroutine scheduling queue. @see KROUTINE_QUEUE */

#else
typedef void kroutine_list_root_t;
#endif

config_depend(CONFIG_MUTEK_KROUTINE_QUEUE)
struct kroutine_queue_s
{
#ifdef CONFIG_MUTEK_KROUTINE_QUEUE
  kroutine_list_root_t list;
#endif
#ifdef CONFIG_MUTEK_KROUTINE_SEMAPHORE
  struct semaphore_s *sem;
#endif
};

#define KROUTINE_DEFAULT_PRIORITY 0

/** @This changes the scheduler priority associated to a kroutine. */
config_depend_alwaysinline(CONFIG_MUTEK_KROUTINE_SCHED,
void kroutine_set_priority(struct kroutine_s *kr,
                           uint8_t priority),
{
  assert(priority < CONFIG_MUTEK_SCHED_PRIORITIES);
#if CONFIG_MUTEK_SCHED_PRIORITIES > 1
  kr->priority = priority;
#endif
});

/** @This initializes a routine with the given callback and scheduling policy. */
__attribute__((deprecated))
ALWAYS_INLINE void kroutine_init(struct kroutine_s *kr,
                                 kroutine_exec_t *exec,
                                 enum kroutine_policy_e policy)
{
  kr->policy = policy;
  kr->exec = exec;
#ifdef CONFIG_MUTEK_KROUTINE_SCHED
# ifdef CONFIG_ARCH_SMP
  kr->cls = NULL;
# endif
# if CONFIG_MUTEK_SCHED_PRIORITIES > 1
  kr->priority = KROUTINE_DEFAULT_PRIORITY;
# endif
#endif
}

/** @This initializes a kroutine with the @ref KROUTINE_IMMEDIATE policy. */
ALWAYS_INLINE void kroutine_init_immediate(struct kroutine_s *kr,
                                           kroutine_exec_t *exec)
{
  kr->policy = KROUTINE_IMMEDIATE;
  kr->exec = exec;
}

/** @This initializes a kroutine with the @ref KROUTINE_TRIGGER policy. */
config_depend_alwaysinline(CONFIG_MUTEK_KROUTINE_TRIGGER,
void kroutine_init_trigger(struct kroutine_s *kr,
                           kroutine_exec_t *exec),
{
  kr->policy = KROUTINE_TRIGGER;
  kr->exec = exec;
  atomic_set(&kr->state, KROUTINE_INVALID);
#if CONFIG_MUTEK_SCHED_PRIORITIES > 1 && defined(CONFIG_MUTEK_KROUTINE_SCHED)
  kr->priority = KROUTINE_DEFAULT_PRIORITY;
#endif
});

/** @This initializes a kroutine with the @ref KROUTINE_DEFERRED policy. */
config_depend_alwaysinline(CONFIG_MUTEK_KROUTINE_SCHED,
void kroutine_init_deferred(struct kroutine_s *kr,
                            kroutine_exec_t *exec),
{
  kr->policy = KROUTINE_DEFERRED;
  kr->exec = exec;
  atomic_set(&kr->state, KROUTINE_INVALID);
#ifdef CONFIG_ARCH_SMP
  kr->cls = NULL;
#endif
#if CONFIG_MUTEK_SCHED_PRIORITIES > 1
  kr->priority = KROUTINE_DEFAULT_PRIORITY;
#endif
});

/** @This initializes a kroutine with the @ref KROUTINE_CPU_DEFERRED
    policy. The target processor used for execution must be specified. */
config_depend_and2_alwaysinline(CONFIG_ARCH_SMP, CONFIG_MUTEK_KROUTINE_SCHED,
void kroutine_init_deferred_cpu(struct kroutine_s *kr,
                                kroutine_exec_t *exec,
                                struct cpu_tree_s *cpu),
{
  kr->policy = KROUTINE_CPU_DEFERRED;
  kr->exec = exec;
  atomic_set(&kr->state, KROUTINE_INVALID);
  kr->cls = cpu->cls;
#if CONFIG_MUTEK_SCHED_PRIORITIES > 1
  kr->priority = KROUTINE_DEFAULT_PRIORITY;
#endif
});

/** @This initializes a kroutine with the @ref KROUTINE_CPU_DEFERRED policy.
    The execution will take place on the current processor. */
config_depend_alwaysinline(CONFIG_MUTEK_KROUTINE_SCHED,
void kroutine_init_deferred_local(struct kroutine_s *kr,
                                  kroutine_exec_t *exec),
{
  kr->policy = KROUTINE_CPU_DEFERRED;
  kr->exec = exec;
  atomic_set(&kr->state, KROUTINE_INVALID);
#ifdef CONFIG_ARCH_SMP
  kr->cls = (void*)CPU_GET_CLS();
#endif
#if CONFIG_MUTEK_SCHED_PRIORITIES > 1
  kr->priority = KROUTINE_DEFAULT_PRIORITY;
#endif
});

/** @This initializes a kroutine with the @ref KROUTINE_CPU_DEFERRED policy.
    The execution will take place on the current processor. */
config_depend_alwaysinline(CONFIG_MUTEK_KROUTINE_SCHED,
void kroutine_init_deferred_seq(struct kroutine_s *kr,
                                kroutine_exec_t *exec,
                                struct kroutine_sequence_s *seq),
{
  kr->policy = KROUTINE_SEQ_DEFERRED;
  kr->exec = exec;
  atomic_set(&kr->state, KROUTINE_INVALID);
#ifdef CONFIG_ARCH_SMP
  kr->seq = seq;
#endif
#if CONFIG_MUTEK_SCHED_PRIORITIES > 1
  kr->priority = KROUTINE_DEFAULT_PRIORITY;
#endif
});

/** @This initializes a kroutine with the @ref KROUTINE_SCHED_SWITCH policy. */
config_depend_alwaysinline(CONFIG_MUTEK_KROUTINE_SCHED,
void kroutine_init_sched_switch(struct kroutine_s *kr,
                                kroutine_exec_t *exec),
{
  kr->policy = KROUTINE_SCHED_SWITCH;
  kr->exec = exec;
  atomic_set(&kr->state, KROUTINE_INVALID);
#ifdef CONFIG_ARCH_SMP
  kr->cls = NULL;
#endif
#if CONFIG_MUTEK_SCHED_PRIORITIES > 1
  kr->priority = KROUTINE_DEFAULT_PRIORITY;
#endif
});

/** @This initializes a kroutine with the @ref KROUTINE_CPU_SCHED_SWITCH policy.
    The target processor used for execution must be specified. */
config_depend_and2_alwaysinline(CONFIG_ARCH_SMP, CONFIG_MUTEK_KROUTINE_SCHED,
void kroutine_init_sched_switch_cpu(struct kroutine_s *kr,
                                    kroutine_exec_t *exec,
                                    struct cpu_tree_s *cpu),
{
  kr->policy = KROUTINE_CPU_SCHED_SWITCH;
  kr->exec = exec;
  atomic_set(&kr->state, KROUTINE_INVALID);
  kr->cls = cpu->cls;
#if CONFIG_MUTEK_SCHED_PRIORITIES > 1
  kr->priority = KROUTINE_DEFAULT_PRIORITY;
#endif
});

/** @This initializes a kroutine with the @ref KROUTINE_CPU_SCHED_SWITCH policy.
    The execution will take place on the current processor. */
config_depend_alwaysinline(CONFIG_MUTEK_KROUTINE_SCHED,
void kroutine_init_sched_switch_local(struct kroutine_s *kr,
                                      kroutine_exec_t *exec),
{
  kr->policy = KROUTINE_CPU_SCHED_SWITCH;
  kr->exec = exec;
  atomic_set(&kr->state, KROUTINE_INVALID);
#ifdef CONFIG_ARCH_SMP
  kr->cls = (void*)CPU_GET_CLS();
#endif
#if CONFIG_MUTEK_SCHED_PRIORITIES > 1
  kr->priority = KROUTINE_DEFAULT_PRIORITY;
#endif
});

/** @This initializes a kroutine with the @ref KROUTINE_CPU_SCHED_SWITCH policy.
    The execution will take place on the current processor. */
config_depend_alwaysinline(CONFIG_MUTEK_KROUTINE_SCHED,
void kroutine_init_sched_switch_seq(struct kroutine_s *kr,
                                    kroutine_exec_t *exec,
                                    struct kroutine_sequence_s *seq),
{
  kr->policy = KROUTINE_SEQ_SCHED_SWITCH;
  kr->exec = exec;
  atomic_set(&kr->state, KROUTINE_INVALID);
#ifdef CONFIG_ARCH_SMP
  kr->seq = seq;
#endif
#if CONFIG_MUTEK_SCHED_PRIORITIES > 1
  kr->priority = KROUTINE_DEFAULT_PRIORITY;
#endif
});

/** @This initializes a kroutine with the @ref KROUTINE_INTERRUPTIBLE policy. */
config_depend_alwaysinline(CONFIG_MUTEK_KROUTINE_SCHED,
void kroutine_init_interruptible(struct kroutine_s *kr,
                                 kroutine_exec_t *exec),
{
  kr->policy = KROUTINE_INTERRUPTIBLE;
  kr->exec = exec;
  atomic_set(&kr->state, KROUTINE_INVALID);
#ifdef CONFIG_ARCH_SMP
  kr->cls = NULL;
#endif
#if CONFIG_MUTEK_SCHED_PRIORITIES > 1
  kr->priority = KROUTINE_DEFAULT_PRIORITY;
#endif
});

/** @This initializes a kroutine with the @ref KROUTINE_CPU_INTERRUPTIBLE
    policy. The target processor used for execution must be specified. */
config_depend_and2_alwaysinline(CONFIG_ARCH_SMP, CONFIG_MUTEK_KROUTINE_SCHED,
void kroutine_init_interruptible_cpu(struct kroutine_s *kr,
                                     kroutine_exec_t *exec,
                                     struct cpu_tree_s *cpu),
{
  kr->policy = KROUTINE_CPU_INTERRUPTIBLE;
  kr->exec = exec;
  atomic_set(&kr->state, KROUTINE_INVALID);
  kr->cls = cpu->cls;
#if CONFIG_MUTEK_SCHED_PRIORITIES > 1
  kr->priority = KROUTINE_DEFAULT_PRIORITY;
#endif
});

/** @This initializes a kroutine with the @ref KROUTINE_CPU_INTERRUPTIBLE policy.
    The execution will take place on the current processor. */
config_depend_alwaysinline(CONFIG_MUTEK_KROUTINE_SCHED,
void kroutine_init_interruptible_local(struct kroutine_s *kr,
                                       kroutine_exec_t *exec),
{
  kr->policy = KROUTINE_CPU_INTERRUPTIBLE;
  kr->exec = exec;
  atomic_set(&kr->state, KROUTINE_INVALID);
#ifdef CONFIG_ARCH_SMP
  kr->cls = (void*)CPU_GET_CLS();
#endif
#if CONFIG_MUTEK_SCHED_PRIORITIES > 1
  kr->priority = KROUTINE_DEFAULT_PRIORITY;
#endif
});

/** @This initializes a kroutine with the @ref KROUTINE_SEQ_INTERRUPTIBLE policy.
    The handler will not run concurrently with kroutines associated to the same
    sequence. */
config_depend_alwaysinline(CONFIG_MUTEK_KROUTINE_SCHED,
void kroutine_init_interruptible_seq(struct kroutine_s *kr,
                                     kroutine_exec_t *exec,
                                     struct kroutine_sequence_s *seq),
{
  kr->policy = KROUTINE_SEQ_INTERRUPTIBLE;
  kr->exec = exec;
  atomic_set(&kr->state, KROUTINE_INVALID);
#ifdef CONFIG_ARCH_SMP
  kr->seq = seq;
#endif
#if CONFIG_MUTEK_SCHED_PRIORITIES > 1
  kr->priority = KROUTINE_DEFAULT_PRIORITY;
#endif
});

/** @This initializes a kroutine with the @ref KROUTINE_IDLE policy. */
config_depend_alwaysinline(CONFIG_MUTEK_KROUTINE_IDLE,
void kroutine_init_idle(struct kroutine_s *kr,
                        kroutine_exec_t *exec),
{
  kr->policy = KROUTINE_IDLE;
  kr->exec = exec;
  atomic_set(&kr->state, KROUTINE_INVALID);
});

/** @This initializes a kroutine with the @ref KROUTINE_QUEUE policy. */
config_depend_alwaysinline(CONFIG_MUTEK_KROUTINE_QUEUE,
void kroutine_init_queue(struct kroutine_s *kr,
                         kroutine_exec_t *exec,
                         struct kroutine_queue_s *queue),
{
  kr->policy = KROUTINE_QUEUE;
  kr->exec = exec;
  kr->queue = queue;
  atomic_set(&kr->state, KROUTINE_INVALID);
});

/** @This initializes a queue used to push kroutine initialized with
    the @ref KROUTINE_QUEUE policy. */
config_depend_alwaysinline(CONFIG_MUTEK_KROUTINE_QUEUE,
error_t kroutine_queue_init(struct kroutine_queue_s *q,
                            struct semaphore_s *sem),
{
  kroutine_list_init(&q->list);
#ifdef CONFIG_MUTEK_KROUTINE_SEMAPHORE
  q->sem = sem;
#else
  assert(sem == NULL);
#endif
  return 0;
});

/** @This releases resources associated to a kroutine sequence. */
config_depend_alwaysinline(CONFIG_MUTEK_KROUTINE_QUEUE,
void kroutine_queue_cleanup(struct kroutine_queue_s *q),
{
  kroutine_list_destroy(&q->list);
});

/** @internal */
error_t kroutine_schedule(struct kroutine_s *kr, enum kroutine_policy_e policy);

/** @This either schedules execution of the kroutine or executes its
    handler immediately depending on the policy. This function returns
    @em true if the handler has been executed immediately.

    For policies other than @ref KROUTINE_TRIGGER, it is ok to call
    this function multiple times before the handler is actually
    executed. When deferred execution is used, the handler may be
    executed only once in this case.
*/
inline bool_t kroutine_exec(struct kroutine_s *kr)
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
# if defined(CONFIG_MUTEK_KROUTINE_QUEUE)
      if (policy != KROUTINE_IMMEDIATE)
        goto sched;
# endif
      goto imm;
#endif
#if defined(CONFIG_MUTEK_KROUTINE_QUEUE)
    case KROUTINE_QUEUE:
# ifdef CONFIG_MUTEK_KROUTINE_IDLE
    case KROUTINE_IDLE:
# endif
# ifdef CONFIG_MUTEK_KROUTINE_SCHED
    case KROUTINE_INTERRUPTIBLE:
    case KROUTINE_SCHED_SWITCH:
    case KROUTINE_DEFERRED:
#  ifdef CONFIG_ARCH_SMP
    case KROUTINE_CPU_INTERRUPTIBLE:
    case KROUTINE_CPU_SCHED_SWITCH:
    case KROUTINE_CPU_DEFERRED:
    case KROUTINE_SEQ_INTERRUPTIBLE:
    case KROUTINE_SEQ_SCHED_SWITCH:
    case KROUTINE_SEQ_DEFERRED:
#  endif
# endif
      if (!atomic_compare_and_swap(&kr->state, KROUTINE_INVALID, !KROUTINE_INVALID))
        return 0;
    sched:
      if (!kroutine_schedule(kr, policy))
        return 0;
#endif
    imm:
      atomic_set(&kr->state, KROUTINE_INVALID); /* reset state */
    case KROUTINE_IMMEDIATE:
      kr->exec(kr, 0);
      return 1;
    }
}

/** @This pops a kroutine from the queue and executes its handler.
    The kroutine policy is KROUTINE_QUEUE and it has been pushed on
    the kroutine_exec function call.

    This function returns false if the queue is empty. */
config_depend(CONFIG_MUTEK_KROUTINE_QUEUE)
bool_t kroutine_queue_process(struct kroutine_queue_s *queue);

/** @This waits on the semaphore associated to the kroutine queue
    then invokes the @ref kroutine_queue_process function. */
config_depend(CONFIG_MUTEK_KROUTINE_SEMAPHORE)
bool_t kroutine_queue_wait(struct kroutine_queue_s *queue);

/** @This should be used on a kroutine initialized with the @ref
    KROUTINE_TRIGGER policy. If it is not the case, this function does
    nothing and returns @em false.

    @This lets the next call to the @ref kroutine_exec function handle
    the kroutine using the @tt policy passed as argument. This
    function returns @em false if the @ref kroutine_exec function has
    not been called yet.

    If the @ref kroutine_exec function has already been called, this
    function will take care of handling the kroutine according to the
    values of the @tt policy argument. @em true is returned in this
    case. */
config_depend_inline(CONFIG_MUTEK_KROUTINE_TRIGGER,
bool_t kroutine_trigger(struct kroutine_s *kr, enum kroutine_policy_e policy),
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
#if defined(CONFIG_MUTEK_KROUTINE_QUEUE)
    case KROUTINE_QUEUE:
# ifdef CONFIG_MUTEK_KROUTINE_IDLE
    case KROUTINE_IDLE:
# endif
# ifdef CONFIG_MUTEK_KROUTINE_SCHED
    case KROUTINE_INTERRUPTIBLE:
    case KROUTINE_SCHED_SWITCH:
    case KROUTINE_DEFERRED:
#  ifdef CONFIG_ARCH_SMP
    case KROUTINE_CPU_INTERRUPTIBLE:
    case KROUTINE_CPU_SCHED_SWITCH:
    case KROUTINE_CPU_DEFERRED:
    case KROUTINE_SEQ_INTERRUPTIBLE:
    case KROUTINE_SEQ_SCHED_SWITCH:
    case KROUTINE_SEQ_DEFERRED:
#  endif
# endif
      if (!kroutine_schedule(kr, policy))
        break;
#endif
    case KROUTINE_IMMEDIATE:
      atomic_set(&kr->state, KROUTINE_INVALID);
      kr->exec(kr, KROUTINE_EXEC_TRIGGERED);
    }
  return 1;
});

/** @This function is similar to @ref kroutine_trigger but does
    nothing when the @ref kroutine_exec function has already been
    called. The routine will only be executed if the call to @ref
    kroutine_exec is performed after the call to this function.

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

/** @This initializes a kroutine sequence. */
config_depend_alwaysinline(CONFIG_MUTEK_KROUTINE_QUEUE,
error_t kroutine_seq_init(struct kroutine_sequence_s *seq),
{
#ifdef CONFIG_ARCH_SMP
  atomic_set(&seq->state, 0);
#endif
  return 0;
});

/** @This releases resources associated to a kroutine sequence. */
config_depend_alwaysinline(CONFIG_MUTEK_KROUTINE_QUEUE,
void kroutine_seq_cleanup(struct kroutine_sequence_s *seq),
{
});

#endif


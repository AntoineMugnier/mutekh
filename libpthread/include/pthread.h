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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#ifndef PTHREAD_H_
#define PTHREAD_H_

/**
 * @file
 * @module{Pthread library}
 */

#ifndef CONFIG_PTHREAD
# warning pthread support is not enabled in configuration file
#else

#include <hexo/types.h>
#include <hexo/error.h>
#include <hexo/local.h>
#include <hexo/lock.h>
#include <hexo/atomic.h>
#include <hexo/context.h>
#include <mutek/scheduler.h>
#include <hexo/interrupt.h>

/************************************************************************
		PThread types
************************************************************************/

typedef void * pthread_start_routine_t(void *arg);

struct pthread_s;
typedef struct pthread_s * pthread_t;

struct pthread_attr_s;
typedef struct pthread_attr_s pthread_attr_t;


/************************************************************************
		PThread private objects
************************************************************************/

/** @internal pointer to current thread */
extern CONTEXT_LOCAL pthread_t __pthread_current;

/************************************************************************
		PThread Thread related public API
************************************************************************/

/** @internal pthread descriptor structure */
struct pthread_s
{
  /** context */
  struct sched_context_s	sched_ctx;

  /* thread state flags (detached, joinable, canceled ...) */
  atomic_t                      state;
  lock_t lock;

#ifdef CONFIG_PTHREAD_JOIN
  /** pointer to thread waiting for termination */
  struct pthread_s              *joined;
  /** joined thread exit value */
  void				*joined_retval;
#endif

  /** start routine argument */
  void				*arg;
  /** start routine pointer */
  pthread_start_routine_t	*start_routine;
};

#define _PTHREAD_STATE_DETACHED         0 //< thread is marked as detached
#define _PTHREAD_STATE_JOINABLE         1 //< thread is joinable
#define _PTHREAD_STATE_CANCELED         2 //< thread has been canceled
#define _PTHREAD_STATE_NOCANCEL         3 //< thread ignore cancel
#define _PTHREAD_STATE_CANCELASYNC      4 //< thread use asynchronous cancelation

#define _PTHREAD_ATTRFLAG_AFFINITY	0x01
#define _PTHREAD_ATTRFLAG_STACK		0x02

/** @internal pthread attributes structure */
struct pthread_attr_s
{
#ifdef CONFIG_PTHREAD_ATTRIBUTES
  uint8_t flags;
  cpu_id_t cpucount;
  cpu_id_t cpulist[CONFIG_CPU_MAXCOUNT];
  void *stack_buf;
  size_t stack_size;
#endif
};

/** @this creates a new pthread attribute */
error_t
pthread_attr_init(pthread_attr_t *attr);

/** @this destroys a new pthread attribute */
error_t
pthread_attr_destroy(pthread_attr_t *attr);

/** @this adds a cpu affinity attribute */
error_t
pthread_attr_affinity(pthread_attr_t *attr, cpu_id_t cpu);

/** @this sets stack buffer attribute */
error_t
pthread_attr_stack(pthread_attr_t *attr, void *stack_buf, size_t stack_size);

/** @this creates a new pthread */
error_t
pthread_create(pthread_t *thread, const pthread_attr_t *attr,
	       pthread_start_routine_t *start_routine, void *arg);

/** @this ends pthread execution */
void pthread_exit(void *retval);

/** @this returns current pthread */
static inline pthread_t
pthread_self(void)
{
  return CONTEXT_LOCAL_GET(__pthread_current);
}

/** @this switchs to next thread */
static inline void
pthread_yield(void)
{
  void __pthread_switch(void);
  __pthread_switch();
}

/** @this compare two thread objects */
static inline error_t
pthread_equal(pthread_t t1, pthread_t t2)
{
  return t1 == t2;
}

#ifdef CONFIG_PTHREAD_JOIN

/** @this detachs a pthread */
error_t
pthread_detach(pthread_t thread);

/** @this waits for thread termination */
error_t
pthread_join(pthread_t thread, void **value_ptr);

#endif

/************************************************************************
		PThread Cancelation related public API
************************************************************************/

#ifdef CONFIG_PTHREAD_CANCEL

/** canceled thread exit value */
#define PTHREAD_CANCELED		((void*)-1)

/** @multiple @this may be used with @ref pthread_setcancelstate */
#define PTHREAD_CANCEL_DISABLE		0
#define PTHREAD_CANCEL_ENABLE		1

/** @multiple @this may be used with @ref pthread_setcanceltype */
#define PTHREAD_CANCEL_DEFERRED		0
#define PTHREAD_CANCEL_ASYNCHRONOUS	1

/** @internal */
typedef void __pthread_cleanup_fcn_t(void*);

/** @internal cancelation cleanup context */
struct __pthread_cleanup_s
{
  __pthread_cleanup_fcn_t	*fcn;
  void				*arg;
  /* pointer to previous cancelation context */
  struct __pthread_cleanup_s	*prev;
};

/** @internal cleanup context linked list */
extern CONTEXT_LOCAL struct __pthread_cleanup_s *__pthread_cleanup_list;

/** @this must be matched with @ref #pthread_cleanup_pop */
#define pthread_cleanup_push(routine_, arg_)		\
{							\
  reg_t				__irq_state;            \
  cpu_interrupt_savestate_disable(&__irq_state);	\
							\
  const struct __pthread_cleanup_s	__cleanup =	\
    {							\
      .fcn = (routine_),				\
      .arg = (arg_),					\
      .prev = CONTEXT_LOCAL_GET(__pthread_cleanup_list),	\
    };							\
							\
  CONTEXT_LOCAL_SET(__pthread_cleanup_list, &__cleanup);	\
							\
  cpu_interrupt_restorestate(&__irq_state);

/** @this must be matched with @ref #pthread_cleanup_push */
#define pthread_cleanup_pop(execute)				\
  cpu_interrupt_savestate_disable(&__irq_state);		\
								\
  if (execute)							\
    __cleanup.fcn(__cleanup.arg);				\
								\
  CONTEXT_LOCAL_SET(__pthread_cleanup_list, __cleanup.prev);	\
								\
  cpu_interrupt_restorestate(&__irq_state);			\
}

static inline void
pthread_testcancel(void)
{
  void __pthread_cancel_self(void);

  if (atomic_bit_test(&pthread_self()->state, _PTHREAD_STATE_CANCELED))
    __pthread_cancel_self();
}

error_t
pthread_setcancelstate(int_fast8_t state, int_fast8_t *oldstate);

error_t
pthread_setcanceltype(int_fast8_t type, int_fast8_t *oldtype);

static inline error_t
pthread_cancel(pthread_t thread)
{
  return atomic_bit_testset(&thread->state, _PTHREAD_STATE_CANCELED);
}

#endif /* CONFIG_PTHREAD_CANCEL */

/************************************************************************
		PThread Mutex related public API
************************************************************************/

#ifdef CONFIG_PTHREAD_MUTEX

typedef struct pthread_mutexattr_s pthread_mutexattr_t;

/** @internal Mutex object structure */
struct				pthread_mutex_s
{
  /** mutex counter */
  uint_fast8_t				count;

# ifdef CONFIG_PTHREAD_MUTEX_ATTR
  /** mutex attributes */
  const struct pthread_mutexattr_s	*attr;

  /** owner thread */
  struct sched_context_s		*owner;
# endif

  /** blocked threads wait queue */
  sched_queue_root_t			wait;
};

typedef struct pthread_mutex_s pthread_mutex_t;

# ifdef CONFIG_PTHREAD_MUTEX_ATTR

/** @internal mutex attributes structure */
struct				pthread_mutexattr_s
{
  /** @internal pointers to lock/trylock/unlock actions depends on type */
  struct {
    error_t (*mutex_lock)	(pthread_mutex_t *mutex);
    error_t (*mutex_trylock)	(pthread_mutex_t *mutex);
    error_t (*mutex_unlock)	(pthread_mutex_t *mutex);
  }				type;
};

/** Normal mutex type identifier */
#  define PTHREAD_MUTEX_NORMAL		0
/** Error checking mutex type identifier */
#  define PTHREAD_MUTEX_ERRORCHECK	1
/** Recurvive mutex type identifier */
#  define PTHREAD_MUTEX_RECURSIVE	2
/** Default mutex type identifier */
#  define PTHREAD_MUTEX_DEFAULT		3

/** @multiple @internal */
extern pthread_mutexattr_t __pthread_mutex_attr_normal;
extern pthread_mutexattr_t __pthread_mutex_attr_errorcheck;
extern pthread_mutexattr_t __pthread_mutex_attr_recursive;

/** @this is the normal mutex object static initializer */
#  define PTHREAD_MUTEX_INITIALIZER						       \
  {										       \
    .wait = CONTAINER_ROOT_INITIALIZER(sched_queue, DLIST), \
    .attr = &__pthread_mutex_attr_normal					       \
  }

/** @this is the recurvive mutex object static initializer */
#  define PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP				      \
  {										       \
    .wait = CONTAINER_ROOT_INITIALIZER(sched_queue, DLIST), \
    .attr = &__pthread_mutex_attr_recursive						\
  }

/** @this is error checking mutex object static initializer */
#  define PTHREAD_ERRORCHECK_MUTEX_INITIALIZER_NP				       \
  {										       \
    .wait = CONTAINER_ROOT_INITIALIZER(sched_queue, DLIST), \
    .attr = &__pthread_mutex_attr_errorcheck					       \
  }

# else

#  define PTHREAD_MUTEX_INITIALIZER						       \
  {										       \
    .wait = CONTAINER_ROOT_INITIALIZER(sched_queue, DLIST), \
  }

# endif

/** @this initializes a mutex */
error_t
pthread_mutex_init(pthread_mutex_t *mutex,
		   const pthread_mutexattr_t *attr);

/** @this destroy a mutex */
error_t
pthread_mutex_destroy(pthread_mutex_t *mutex);

# ifdef CONFIG_PTHREAD_MUTEX_ATTR

/** @this takes a mutex */
static inline error_t
pthread_mutex_lock(pthread_mutex_t *mutex)
{
  return mutex->attr->type.mutex_lock(mutex);
}

/** @this tries to take a mutex */
static inline error_t
pthread_mutex_trylock(pthread_mutex_t *mutex)
{
  return mutex->attr->type.mutex_trylock(mutex);
}

/** @this free a mutex */
static inline error_t
pthread_mutex_unlock(pthread_mutex_t *mutex)
{
  return mutex->attr->type.mutex_unlock(mutex);
}

/** @this sets the mutek type */
error_t
pthread_mutexattr_settype(pthread_mutexattr_t *attr, int_fast8_t type);

/** @this initialize a mutex attribute object */
static inline error_t
pthread_mutexattr_init(pthread_mutexattr_t *attr)
{
  return pthread_mutexattr_settype(attr, PTHREAD_MUTEX_DEFAULT);
}

/** @this destroy a mutex attribute object */
static inline error_t
pthread_mutexattr_destroy(pthread_mutexattr_t *attr)
{
  return 0;
}

# else /* !CONFIG_PTHREAD_MUTEX_ATTR */

static inline error_t
pthread_mutex_lock(pthread_mutex_t *mutex)
{
  error_t __pthread_mutex_normal_lock(pthread_mutex_t *mutex);
  return __pthread_mutex_normal_lock(mutex);
}

static inline error_t
pthread_mutex_trylock(pthread_mutex_t *mutex)
{
  error_t __pthread_mutex_normal_trylock(pthread_mutex_t *mutex);
  return __pthread_mutex_normal_trylock(mutex);
}

static inline error_t
pthread_mutex_unlock(pthread_mutex_t *mutex)
{
  error_t __pthread_mutex_normal_unlock(pthread_mutex_t *mutex);
  return __pthread_mutex_normal_unlock(mutex);
}

# endif

#endif

/************************************************************************
		PThread Cond related public API
************************************************************************/

#ifdef CONFIG_PTHREAD_COND

struct timespec;

/** @internal */
struct pthread_cond_s
{
  /** blocked threads wait queue */
  sched_queue_root_t		wait;
};

typedef struct pthread_cond_s pthread_cond_t;

typedef struct pthread_condattr_s pthread_condattr_t;

error_t
pthread_cond_init(pthread_cond_t *cond,
		  const pthread_condattr_t *attr);

error_t
pthread_cond_destroy(pthread_cond_t *cond);

error_t
pthread_cond_broadcast(pthread_cond_t *cond);

error_t
pthread_cond_signal(pthread_cond_t *cond);

error_t
pthread_cond_timedwait(pthread_cond_t *cond, 
		       pthread_mutex_t *mutex,
		       const struct timespec *delay);

error_t
pthread_cond_wait(pthread_cond_t *cond, pthread_mutex_t *mutex);

/** normal cond object static initializer */
# define PTHREAD_COND_INITIALIZER						       \
  {										       \
    .wait = CONTAINER_ROOT_INITIALIZER(sched_queue, DLIST), \
  }

#endif

/************************************************************************
		PThread RWLock related public API
************************************************************************/

#ifdef CONFIG_PTHREAD_RWLOCK

#include <mutek/rwlock.h>

/** @internal */
struct rwlock_s;

typedef struct pthread_rwlockattr_s pthread_rwlockattr_t;
typedef struct rwlock_s pthread_rwlock_t;

static inline error_t
pthread_rwlock_destroy(pthread_rwlock_t *rwlock)
{
  return rwlock_destroy(rwlock);
}

static inline error_t
pthread_rwlock_init(pthread_rwlock_t *rwlock,
		    const pthread_rwlockattr_t *attr)
{
  return rwlock_init(rwlock);
}

static inline error_t
pthread_rwlock_rdlock(pthread_rwlock_t *rwlock)
{
# ifdef CONFIG_PTHREAD_CANCEL
  pthread_testcancel();
# endif
  return rwlock_rdlock(rwlock);
}

static inline error_t
pthread_rwlock_tryrdlock(pthread_rwlock_t *rwlock)
{
  return rwlock_tryrdlock(rwlock);
}

static inline error_t
pthread_rwlock_wrlock(pthread_rwlock_t *rwlock)
{
# ifdef CONFIG_PTHREAD_CANCEL
  pthread_testcancel();
# endif
  return rwlock_wrlock(rwlock);
}

static inline error_t
pthread_rwlock_trywrlock(pthread_rwlock_t *rwlock)
{
  return rwlock_trywrlock(rwlock);
}

static inline error_t
pthread_rwlock_unlock(pthread_rwlock_t *rwlock)
{
  return rwlock_unlock(rwlock);
}

/** normal rwlock object static initializer */
# define PTHREAD_RWLOCK_INITIALIZER RWLOCK_INITIALIZER

#endif

/************************************************************************
		PThread barrier related public API
************************************************************************/

#ifdef CONFIG_PTHREAD_BARRIER

typedef struct pthread_barrierattr_s pthread_barrierattr_t;

/** @internal */
struct				pthread_barrier_s
{
  int_fast8_t				count;
  /** blocked threads waiting for read */
  sched_queue_root_t			wait;
};

typedef struct pthread_barrier_s pthread_barrier_t;

error_t pthread_barrier_destroy(pthread_barrier_t *barrier);

error_t pthread_barrier_init(pthread_barrier_t *barrier,
			     const pthread_barrierattr_t *attr,
			     unsigned count);

error_t pthread_barrier_wait(pthread_barrier_t *barrier);

/** @this may be returned by @ref pthread_barrier_wait */
#define PTHREAD_BARRIER_SERIAL_THREAD	-1

/** normal rwlock object static initializer */
# define PTHREAD_BARRIER_INITIALIZER(n)							  \
  {											  \
    .wait = CONTAINER_ROOT_INITIALIZER(sched_queue, DLIST),	  \
    .count = (n),									  \
  }

#endif

/************************************************************************
		PThread Spinlock related public API
************************************************************************/

#ifdef CONFIG_PTHREAD_SPIN

typedef struct arch_lock_s pthread_spinlock_t;

static inline error_t
pthread_spin_init(pthread_spinlock_t *spinlock,
		      bool_t pshared)
{
  return arch_lock_init(spinlock);
}

static inline error_t
pthread_spin_destroy(pthread_spinlock_t *spinlock)
{
  arch_lock_destroy(spinlock);
  return 0;
}

static inline error_t
pthread_spin_lock(pthread_spinlock_t *spinlock)
{
  arch_lock_spin(spinlock);
  return 0;
}

static inline error_t
pthread_spin_trylock(pthread_spinlock_t *spinlock)
{
  return arch_lock_try(spinlock) ? -EBUSY : 0;
}

static inline error_t
pthread_spin_unlock(pthread_spinlock_t *spinlock)
{
  arch_lock_release(spinlock);
  return 0;
}

#endif


#endif
#endif


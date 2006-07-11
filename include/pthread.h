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

#include <hexo/types.h>
#include <hexo/error.h>
#include <hexo/local.h>
#include <hexo/lock.h>
#include <hexo/task.h>
#include <hexo/template/cont_clist.h>
#include <hexo/interrupt.h>

/************************************************************************
		PThread types
************************************************************************/

CONTAINER_TYPE_DECL(pthread, CLIST, struct pthread_s, NOLOCK);

typedef void * pthread_start_routine_t(void *arg);

/** pthread pool */
struct pthread_pool_s
{
  lock_t			lock;
  pthread_cont_t		list;
};

struct pthread_s;
typedef struct pthread_s * pthread_t;

struct pthread_attr_s;
typedef struct pthread_attr_s pthread_attr_t;


/************************************************************************
		PThread private objects
************************************************************************/

/** pointer to current thread */
extern TASK_LOCAL pthread_t __pthread_current;

/** init pthread sub system and bootstrap initial thread */
void __pthread_bootstrap(void);

/** switch to next thread */
void __pthread_switch(void);

/************************************************************************
		PThread Thread related public API
************************************************************************/

/** pthread descriptor structure */

struct pthread_s
{
  /** task context */
  struct task_s			task;

#ifdef CONFIG_PTHREAD_JOIN
  /** thread is marked as detached */
  __bool_t			detached:1;
  /** thread has exited and is waiting for join/detach */
  __bool_t			joinable:1;

  /** pointer to thread waiting for termination */
  struct pthread_s		*joined;
  /** joined thread exit value */
  void				*joined_retval;
#endif

#ifdef CONFIG_PTHREAD_CANCEL
  __bool_t			canceled:1;
  __bool_t			cancelstate:1;
  __bool_t			cancelasync:1;
#endif

  /** start routine argument */
  void				*arg;
  /** start routine pointer */
  pthread_start_routine_t	*start_routine;

  /** thread queue entry */
  pthread_entry_t		queue;
};

/** pthread attributes structure */
struct pthread_attr_s
{
};


/** create a new pthread */
error_t
pthread_create(pthread_t *thread, const pthread_attr_t *attr,
	       pthread_start_routine_t *start_routine, void *arg);

/** end pthread execution */
void
pthread_exit(void *retval);

/** return current pthread */
static inline pthread_t
pthread_self(void)
{
  return TASK_LOCAL_GET(__pthread_current);
}

/** switch to next thread */
static inline void
pthread_yield(void)
{
  __pthread_switch();
}


#ifdef CONFIG_PTHREAD_JOIN

/** detach pthread */
error_t
pthread_detach(pthread_t thread);

/** wait for thread termination */
error_t
pthread_join(pthread_t thread, void **value_ptr);

#endif

/** display pthread current runqueue */
void __pthread_dump_runqueue(void);

/************************************************************************
		PThread Mutex related public API
************************************************************************/

typedef struct pthread_mutexattr_s pthread_mutexattr_t;

/** mutex object structure */
typedef struct				pthread_mutex_s
{
  /** struct protection spinlock */
  lock_t				lock;

  /** mutex counter */
  uint_fast8_t				count;

#ifdef CONFIG_PTHREAD_MUTEX_ATTR
  /** mutex attributes */
  const struct pthread_mutexattr_s	*attr;

  /** owner thread */
  struct pthread_s			*owner;
#endif

  /** blocked threads wait queue */
  pthread_cont_t			wait;

}					pthread_mutex_t;

#ifdef CONFIG_PTHREAD_MUTEX_ATTR

/** mutex attributes structure */
struct				pthread_mutexattr_s
{
  /** pointers to lock/trylock/unlock actions depends on type */
  struct {
    error_t (*mutex_lock)	(pthread_mutex_t *mutex);
    error_t (*mutex_trylock)	(pthread_mutex_t *mutex);
    error_t (*mutex_unlock)	(pthread_mutex_t *mutex);
  }				type;
};

/** normal mutex type identifier */
# define PTHREAD_MUTEX_NORMAL		0
/** error checking mutex type identifier */
# define PTHREAD_MUTEX_ERRORCHECK	1
/** recurvive mutex type identifier */
# define PTHREAD_MUTEX_RECURSIVE	2
/** default mutex type identifier */
# define PTHREAD_MUTEX_DEFAULT		3

extern CPUARCH_LOCAL pthread_mutexattr_t __pthread_mutex_attr_normal;
extern CPUARCH_LOCAL pthread_mutexattr_t __pthread_mutex_attr_errorcheck;
extern CPUARCH_LOCAL pthread_mutexattr_t __pthread_mutex_attr_recursive;

/** normal mutex object static initializer */
# define PTHREAD_MUTEX_INITIALIZER				\
  {								\
    .lock = LOCK_INITIALIZER,					\
    .attr = CPUARCH_LOCAL_ADDR(__pthread_mutex_attr_normal)	\
  }

/** recurvive mutex object static initializer */
# define PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP			\
  {								\
    .lock = LOCK_INITIALIZER,					\
    .attr = CPUARCH_LOCAL_ADDR(__pthread_mutex_attr_recursive)	\
  }

/** error checking mutex object static initializer */
# define PTHREAD_ERRORCHECK_MUTEX_INITIALIZER_NP		\
  {								\
    .lock = LOCK_INITIALIZER,					\
    .attr = CPUARCH_LOCAL_ADDR(__pthread_mutex_attr_errorcheck)	\
  }

#else

/** normal mutex object static initializer */
# define PTHREAD_MUTEX_INITIALIZER				\
  {								\
    .lock = LOCK_INITIALIZER,					\
  }

#endif

error_t
pthread_mutex_init(pthread_mutex_t *mutex,
		   const pthread_mutexattr_t *attr);

error_t
pthread_mutex_destroy(pthread_mutex_t *mutex);

#ifdef CONFIG_PTHREAD_MUTEX_ATTR

static inline error_t
pthread_mutex_lock(pthread_mutex_t *mutex)
{
  return mutex->attr->type.mutex_lock(mutex);
}

static inline error_t
pthread_mutex_trylock(pthread_mutex_t *mutex)
{
  return mutex->attr->type.mutex_trylock(mutex);
}

static inline error_t
pthread_mutex_unlock(pthread_mutex_t *mutex)
{
  return mutex->attr->type.mutex_unlock(mutex);
}

error_t
pthread_mutexattr_settype(pthread_mutexattr_t *attr, int_fast8_t type);

static inline error_t
pthread_mutexattr_init(pthread_mutexattr_t *attr)
{
  return pthread_mutexattr_settype(attr, PTHREAD_MUTEX_DEFAULT);
}

static inline error_t
pthread_mutexattr_destroy(pthread_mutexattr_t *attr)
{
  return 0;
}

/*
error_t pthread_mutexattr_gettype(const pthread_mutexattr_t *attr, int *type);
error_t pthread_mutexattr_setprioceiling(pthread_mutexattr_t *attr, int);
error_t pthread_mutexattr_setprotocol(pthread_mutexattr_t *attr, int);
error_t pthread_mutexattr_setpshared(pthread_mutexattr_t *attr, int);
error_t pthread_mutexattr_getprioceiling(const pthread_mutexattr_t *attr, int *);
error_t pthread_mutexattr_getprotocol(const pthread_mutexattr_t *attr, int *);
error_t pthread_mutexattr_getpshared(const pthread_mutexattr_t *attr, int *);
*/

#else /* !CONFIG_PTHREAD_MUTEX_ATTR */

error_t __pthread_mutex_normal_lock(pthread_mutex_t *mutex);
error_t __pthread_mutex_normal_trylock(pthread_mutex_t *mutex);
error_t __pthread_mutex_normal_unlock(pthread_mutex_t *mutex);

static inline error_t
pthread_mutex_lock(pthread_mutex_t *mutex)
{
  return __pthread_mutex_normal_lock(mutex);
}

static inline error_t
pthread_mutex_trylock(pthread_mutex_t *mutex)
{
  return __pthread_mutex_normal_trylock(mutex);
}

static inline error_t
pthread_mutex_unlock(pthread_mutex_t *mutex)
{
  return __pthread_mutex_normal_unlock(mutex);
}

#endif

/************************************************************************
		PThread Cond related public API
************************************************************************/

struct timespec;

typedef struct pthread_cond_s
{
  /** struct protection spinlock */
  lock_t			lock;

  /** blocked threads wait queue */
  pthread_cont_t		wait;

} pthread_cond_t;

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
# define PTHREAD_COND_INITIALIZER				\
  {								\
    .lock = LOCK_INITIALIZER,					\
  }

/************************************************************************
		PThread RWLock related public API
************************************************************************/

typedef struct pthread_rwlockattr_s pthread_rwlockattr_t;

/** mutex object structure */
typedef struct				pthread_rwlock_s
{
  /** struct protection spinlock */
  lock_t				lock;

  /** mutex counter
      == 0: free
      < 0 : write locked
      > 0 : read locked */
  int_fast8_t				count;

  /** blocked threads waiting for read */
  pthread_cont_t			wait_rd;
  /** blocked threads waiting for write */
  pthread_cont_t			wait_wr;
}					pthread_rwlock_t;

error_t
pthread_rwlock_destroy(pthread_rwlock_t *rwlock);

error_t
pthread_rwlock_init(pthread_rwlock_t *rwlock, const pthread_rwlockattr_t *attr);

error_t
pthread_rwlock_rdlock(pthread_rwlock_t *rwlock);

error_t
pthread_rwlock_wrlock(pthread_rwlock_t *rwlock);

error_t
pthread_rwlock_tryrdlock(pthread_rwlock_t *rwlock);

error_t
pthread_rwlock_trywrlock(pthread_rwlock_t *rwlock);

error_t
pthread_rwlock_unlock(pthread_rwlock_t *rwlock);

/** normal rwlock object static initializer */
# define PTHREAD_RWLOCK_INITIALIZER				\
  {								\
    .lock = LOCK_INITIALIZER,					\
  }

/************************************************************************
		PThread Spinlock related public API
************************************************************************/

typedef struct arch_lock_s pthread_spinlock_t;

static inline error_t
pthread_spin_init(pthread_spinlock_t *spinlock,
		      __bool_t pshared)
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

/************************************************************************
		PThread Cancelation related public API
************************************************************************/

#ifdef CONFIG_PTHREAD_CANCEL

/** canceled thread exit value */
#define PTHREAD_CANCELED		((void*)-1)

#define PTHREAD_CANCEL_DISABLE		0
#define PTHREAD_CANCEL_ENABLE		1

#define PTHREAD_CANCEL_DEFERRED		0
#define PTHREAD_CANCEL_ASYNCHRONOUS	1

typedef void __pthread_cleanup_fcn_t(void*);

/** cancelation cleanup context */
struct __pthread_cleanup_s
{
  __pthread_cleanup_fcn_t	*fcn;
  void				*arg;
  /* pointer to previous cancelation context */
  struct __pthread_cleanup_s	*prev;
};

/** cleanup context linked list */
extern TASK_LOCAL struct __pthread_cleanup_s *__pthread_cleanup_list;

#define pthread_cleanup_push(routine_, arg_)		\
{							\
  __reg_t				__irq_state;	\
							\
  cpu_interrupt_savestate_disable(&__irq_state);	\
							\
  const struct __pthread_cleanup_s	__cleanup =	\
    {							\
      .fcn = (routine_),				\
      .arg = (arg_),					\
      .prev = TASK_LOCAL_GET(__pthread_cleanup_list),	\
    };							\
							\
  TASK_LOCAL_SET(__pthread_cleanup_list, &__cleanup);	\
							\
  cpu_interrupt_restorestate(&__irq_state);

#define pthread_cleanup_pop(execute)				\
  cpu_interrupt_savestate_disable(&__irq_state);		\
								\
  if (execute)							\
    __cleanup.fcn(__cleanup.arg);				\
								\
  TASK_LOCAL_SET(__pthread_cleanup_list, __cleanup.prev);	\
								\
  cpu_interrupt_restorestate(&__irq_state);			\
}

void __pthread_cancel_self(void);

static inline void
pthread_testcancel(void)
{
  if (pthread_self()->canceled)
    __pthread_cancel_self();
}

static inline void
pthread_cancel(pthread_t thread)
{
  thread->canceled = 1;
}

static inline int_fast8_t
pthread_setcancelstate(int_fast8_t state, int_fast8_t *oldstate)
{
  struct pthread_s	*self = pthread_self();

  if (oldstate)
    *oldstate = self->cancelstate;

  self->cancelstate = state;

  return 0;
}


static inline int_fast8_t
pthread_setcanceltype(int_fast8_t type, int_fast8_t *oldtype)
{
  struct pthread_s	*self = pthread_self();

  if (oldtype)
    *oldtype = self->cancelasync;

  self->cancelasync = type;

  return 0;
}


#endif /* CONFIG_PTHREAD_CANCEL */

#endif


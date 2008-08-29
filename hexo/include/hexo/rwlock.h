#ifndef __RW_LOCK__
#define __RW_LOCK__


#include <hexo/types.h>
#include <hexo/error.h>
#include <hexo/scheduler.h>
#include <hexo/lock.h>


struct rwlock_s
{
  /** mutex counter
      < 0 : write locked
      > 0 : read locked */
  
  int_fast8_t                           count;

  /** blocked threads waiting for read */
  sched_queue_root_t                    wait_rd;
  /** blocked threads waiting for write */
  sched_queue_root_t                    wait_wr;
};

error_t
rwlock_destroy(struct rwlock_s *rwlock);

error_t
rwlock_init(struct rwlock_s *rwlock);

error_t
rwlock_rdlock(struct rwlock_s *rwlock);

error_t
rwlock_wrlock(struct rwlock_s *rwlock);

error_t
rwlock_tryrdlock(struct rwlock_s *rwlock);

error_t
rwlock_trywrlock(struct rwlock_s *rwlock);

error_t
rwlock_unlock(struct rwlock_s *rwlock);


/** normal rwlock object static initializer */
#define RWLOCK_INITIALIZER                              \
  {                                                             \
     .wait_rd = CONTAINER_ROOT_INITIALIZER(sched_queue, DLIST),	\
     .wait_wr = CONTAINER_ROOT_INITIALIZER(sched_queue, DLIST),	\
  }

#endif

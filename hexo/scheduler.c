
#include <hexo/scheduler.h>
#include <hexo/init.h>
#include <hexo/local.h>

/* processor current scheduler context */
CONTEXT_LOCAL struct sched_context_s *sched_cur;

/* processor idle context */
CPU_LOCAL struct sched_context_s sched_idle;

/************************************************************************/

#if defined (CONFIG_HEXO_SCHED_MIGRATION)

/* return next scheduler candidate */
static inline struct sched_context_s *
__sched_candidate_noidle(sched_queue_root_t *root)
{
  return sched_queue_nolock_pop(root);
}

/* return next scheduler candidate */
static inline struct sched_context_s *
__sched_candidate(sched_queue_root_t *root)
{
  struct sched_context_s	*next;

  if ((next = __sched_candidate_noidle(root)) == NULL)
    next = CPU_LOCAL_ADDR(sched_idle);

  return next;
}

/* scheduler root */
static sched_queue_root_t	sched_root;

static inline sched_queue_root_t *
__sched_root(void)
{
  return &sched_root;
}

static inline sched_queue_root_t *
__sched_root_cpu(cpu_id_t cpu)
{
  return &sched_root;
}

/************************************************************************/

#elif defined (CONFIG_HEXO_SCHED_ALGO_STATIC)

/* return next scheduler candidate */
static inline struct sched_context_s *
__sched_candidate_noidle(sched_queue_root_t *root)
{
  return sched_queue_nolock_pop(root);
}

/* return next scheduler candidate */
static inline struct sched_context_s *
__sched_candidate(sched_queue_root_t *root)
{
  struct sched_context_s	*next;

  if ((next = __sched_candidate_noidle(root)) == NULL)
    next = CPU_LOCAL_ADDR(sched_idle);

  return next;
}

/* scheduler root */
static CPU_LOCAL sched_queue_root_t	sched_root;

static inline sched_queue_root_t *
__sched_root(void)
{
  return CPU_LOCAL_ADDR(sched_root);
}

static inline sched_queue_root_t *
__sched_root_cpu(cpu_id_t cpu)
{
#error check CPU_LOCAL_FOREIGN_ADDR implem
  return CPU_LOCAL_FOREIGN_ADDR(cpu, sched_root);
}

#endif

/************************************************************************/

/* idle context runtime */
static CONTEXT_ENTRY(sched_context_idle)
{
  sched_queue_root_t *root = __sched_root();

  /* release lock acquired in previous sched_context_switch() call */
  sched_unlock();
  cpu_interrupt_enable();

  while (1)
    {
      struct sched_context_s	*next;

      cpu_interrupt_enable();

      /* do not wait if several cpus are running because context may
	 be put in running queue by an other cpu with no interrupt */
#if !defined(CONFIG_SMP) || defined(CONFIG_HEXO_IPI)
      /* CPU sleep waiting for interrupts */
      cpu_interrupt_wait();
#endif

      /* Let enough time for pending interrupts to execute and assume
	 memory is clobbered to force scheduler root queue
	 reloading after interrupts execution. */
      cpu_interrupt_process();

      /* try to switch to next context */
      cpu_interrupt_disable();
      sched_queue_wrlock(root);

      if ((next = __sched_candidate_noidle(root)) != NULL)
	{
	  context_switch_to(&next->context);
	  //	  printf("(c%i idle)", cpu_id());
	}

      sched_queue_unlock(root);
    }
}

/* Switch to next context available in the root queue. This function
   returns if no other context is available, controle is passed
   back to current context rather than Idle context. Must be called
   with interrupts disabled */
static inline
void __sched_pushback_switch(sched_queue_root_t *root)
{
  struct sched_context_s	*next;

  /* push context back in running queue */
  sched_queue_wrlock(root);

  if ((next = __sched_candidate_noidle(root)))
    {
      sched_queue_nolock_pushback(root, CONTEXT_LOCAL_GET(sched_cur));
      context_switch_to(&next->context);
    }

  sched_queue_unlock(root);
}

static inline
void __sched_context_exit(sched_queue_root_t *root)
{
  struct sched_context_s	*next;

  /* get next running context */
  next = __sched_candidate(root);
  context_jump_to(&next->context);
}

/************************************************************************/

/* Must be called with interrupts disabled */
void sched_context_switch(void)
{
  assert(!cpu_interrupt_getstate());

  __sched_pushback_switch(__sched_root());
}

/* Must be called with interrupts disabled and sched locked */
void sched_context_exit(void)
{
  assert(!cpu_interrupt_getstate());

  __sched_context_exit(__sched_root());
}

void sched_lock(void)
{
  assert(!cpu_interrupt_getstate());

  sched_queue_wrlock(__sched_root());
}

void sched_unlock(void)
{
  assert(!cpu_interrupt_getstate());

  sched_queue_unlock(__sched_root());
}

void sched_context_init(struct sched_context_s *sched_ctx)
{
  /* set sched_cur context local variable */
  CONTEXT_LOCAL_FOREIGN_SET(sched_ctx->context.tls,
			    sched_cur, sched_ctx);
}

/* Must be called with interrupts disabled */
void sched_context_start(struct sched_context_s *sched_ctx)
{
  sched_queue_root_t *root;

  assert(!cpu_interrupt_getstate());

#if !defined(CONFIG_HEXO_SCHED_AFFINITY)

# if defined (CONFIG_HEXO_SCHED_ALGO_STATIC)
  static cpu_id_t	next_cpu = 0;
  root = __sched_root_cpu(next_cpu++ % cpu_count());
# else
  root = __sched_root();
# endif

#else

# if defined (CONFIG_HEXO_SCHED_ALGO_STATIC)
  root = __sched_root_cpu(sched_ctx->cpu);
# elif defined (CONFIG_HEXO_SCHED_MIGRATION)
  root = __sched_root();
# endif

#endif

  sched_queue_pushback(root, sched_ctx);
}

/* push current context in the 'queue', unlock it and switch to next
   context available in the 'root' queue. Must be called with
   interrupts disabled */
void sched_wait_unlock(sched_queue_root_t *queue)
{
  sched_queue_root_t *root = __sched_root();
  struct sched_context_s *next;

  assert(!cpu_interrupt_getstate());

  /* add current context to queue, assume queue is already locked */
  sched_queue_nolock_pushback(queue, CONTEXT_LOCAL_GET(sched_cur));
  sched_queue_unlock(queue);

  /* get next running context */
  sched_queue_wrlock(root);
  next = __sched_candidate(root);
  context_switch_to(&next->context);
  sched_queue_unlock(root);
}


/* push current context in the 'queue' ignoring queue lock, unlock the
   given lock and switch to next context available in the 'root'
   queue. Must be called with interrupts disabled */
void sched_wait_unlock2(sched_queue_root_t *queue, lock_t *lock)
{
  sched_queue_root_t *root = __sched_root();
  struct sched_context_s *next;

  assert(!cpu_interrupt_getstate());

  /* add current context to queue, assume queue is already locked */
  sched_queue_nolock_pushback(queue, CONTEXT_LOCAL_GET(sched_cur));
  lock_release(lock);

  /* get next running context */
  sched_queue_wrlock(root);
  next = __sched_candidate(root);
  context_switch_to(&next->context);
  sched_queue_unlock(root);
}


/* Switch to next context available in the 'root' queue, do not put
   current context in any queue. Idle context may be selected if no
   other contexts are available. Must be called with interrupts
   disabled */
void sched_context_stop(void)
{
  sched_queue_root_t *root = __sched_root();
  struct sched_context_s *next;

  assert(!cpu_interrupt_getstate());

  /* get next running context */
  sched_queue_wrlock(root);
  next = __sched_candidate(root);
  context_switch_to(&next->context);
  sched_queue_unlock(root);
}

/* Must be called with interrupts disabled and queue locked */
struct sched_context_s *sched_wake(sched_queue_root_t *queue)
{
  struct sched_context_s	*sched_ctx;

  assert(!cpu_interrupt_getstate());

  if ((sched_ctx = sched_queue_nolock_pop(queue)))
    sched_queue_pushback(__sched_root(), sched_ctx);

  return sched_ctx;
}

void sched_global_init(void)
{
  sched_queue_init(__sched_root());
}

/*

    %config CONFIG_HEXO_SCHED_IDLE_STACK_SIZE
    desc Size of the stack allocated for idle scheduler context.
    desc Stack size is specified in stack words count.
    parent CONFIG_HEXO_SCHED
    default 256
    %config end

*/

void sched_cpu_init(void)
{
  struct sched_context_s *idle = CPU_LOCAL_ADDR(sched_idle);

  int res = context_init(&idle->context, CONFIG_HEXO_SCHED_IDLE_STACK_SIZE, sched_context_idle, 0);

  assert(res != 0);
}

#if !defined(CONFIG_HEXO_SCHED_AFFINITY)

/** scheduler context will run on this cpu */
void sched_affinity_add(struct sched_context_s *sched_ctx, cpu_id_t cpu)
{
}

/** scheduler context will not run on this cpu */
void sched_affinity_remove(struct sched_context_s *sched_ctx, cpu_id_t cpu)
{
}

/** scheduler context will run on a single cpu */
void sched_affinity_single(struct sched_context_s *sched_ctx, cpu_id_t cpu)
{
}

/** scheduler context will run on all cpu */
void sched_affinity_all(struct sched_context_s *sched_ctx)
{
}

#else
# if defined (CONFIG_HEXO_SCHED_MIGRATION)
/** scheduler context will run on this cpu */
void sched_affinity_add(struct sched_context_s *sched_ctx, cpu_id_t cpu)
{
}

/** scheduler context will not run on this cpu */
void sched_affinity_remove(struct sched_context_s *sched_ctx, cpu_id_t cpu)
{
}

/** scheduler context will run on a single cpu */
void sched_affinity_single(struct sched_context_s *sched_ctx, cpu_id_t cpu)
{
}

/** scheduler context will run on all cpu */
void sched_affinity_all(struct sched_context_s *sched_ctx)
{
}

# endif

# if defined (CONFIG_HEXO_SCHED_ALGO_STATIC)
/** scheduler context will run on this cpu */
void sched_affinity_add(struct sched_context_s *sched_ctx, cpu_id_t cpu)
{
  assert(cpu < cpu_count());

  sched_ctx->cpu = cpu;
}

/** scheduler context will not run on this cpu */
void sched_affinity_remove(struct sched_context_s *sched_ctx, cpu_id_t cpu)
{
}

/** scheduler context will run on a single cpu */
void sched_affinity_single(struct sched_context_s *sched_ctx, cpu_id_t cpu)
{
  sched_affinity_add(sched_ctx, cpu);
}

/** scheduler context will run on all cpu */
void sched_affinity_all(struct sched_context_s *sched_ctx)
{
}

# endif

#endif


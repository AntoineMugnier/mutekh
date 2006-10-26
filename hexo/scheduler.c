
#include <hexo/scheduler.h>
#include <hexo/init.h>
#include <hexo/local.h>

/* processor current scheduler context */
CONTEXT_LOCAL struct sched_context_s *sched_cur;

/* processor idle context */
CPU_LOCAL struct sched_context_s sched_idle;


/* scheduler root */
static sched_queue_root_t	sched_root;

static inline sched_queue_root_t *
__sched_root(void)
{
  return &sched_root;
}

/* idle context runtime */
static CONTEXT_ENTRY(sched_context_idle)
{
  /* release lock acquired in previous sched_context_switch() call */
  sched_unlock();
  cpu_interrupt_enable();

  while (1)
    {
      cpu_interrupt_enable();
#if !defined(CONFIG_SMP) || defined(CONFIG_HEXO_IPI)
      /* CPU sleep waiting for interrupts */
      cpu_interrupt_wait();
#endif
      asm volatile("nop"); /* XXX trouver une logique a ca */
      /*
	 on dirait que si on omet le "nop", le cpu n'arrive pas a
	 gerer les int entre le sti et le cli.
      */
      /* try to switch to next context */
      cpu_interrupt_disable();
      sched_context_stop();
    }
}

/* return next scheduler candidate */
static inline struct sched_context_s *
__sched_candidate(sched_queue_root_t *root)
{
  struct sched_context_s	*next;

  if (!(next = sched_queue_nolock_pop(root)))
    next = CPU_LOCAL_ADDR(sched_idle);

  return next;
}

/* switch to next context available in the 'root' queue, do not put
   current thread in any queue */
/* must be called with interrupts disabled */
static inline
void __sched_context_switch(sched_queue_root_t *root)
{
  struct sched_context_s	*next;

  assert(!cpu_interrupt_getstate());

  /* get next running thread */
  sched_queue_wrlock(root);
  next = __sched_candidate(root);
  context_switch_to(&next->context);
  sched_queue_unlock(root);
}

/* switch to next context available in the root queue */
static inline
void __sched_pushback_switch(sched_queue_root_t *root)
{
  struct sched_context_s	*next;

  assert(!cpu_interrupt_getstate());

  /* push thread back in running queue */
  sched_queue_wrlock(root);
  sched_queue_nolock_pushback(root, CONTEXT_LOCAL_GET(sched_cur));

  /* get next running thread */
  next = __sched_candidate(root);
  context_switch_to(&next->context);
  sched_queue_unlock(root);
}

/* switch to next context available in the 'root' queue and push current
   context in the 'queue' */
/* must be called with interrupts disabled */
static inline
void __sched_wait_switch(sched_queue_root_t *root, sched_queue_root_t *wait)
{
  struct sched_context_s	*next;

  assert(!cpu_interrupt_getstate());

  /* add current thread to queue, assume queue is already locked */
  sched_queue_nolock_pushback(wait, CONTEXT_LOCAL_GET(sched_cur));
  sched_queue_unlock(wait);

  /* get next running thread */
  sched_queue_wrlock(root);
  next = __sched_candidate(root);
  context_switch_to(&next->context);
  sched_queue_unlock(root);
}

static inline
void __sched_context_exit(sched_queue_root_t *root)
{
  struct sched_context_s	*next;

  assert(!cpu_interrupt_getstate());

  /* get next running thread */
  next = __sched_candidate(root);
  context_jump_to(&next->context);
}

/* Must be called with interrupts disabled */
void sched_context_switch(void)
{
  assert(!cpu_interrupt_getstate());

  __sched_pushback_switch(__sched_root());
}

/* Must be called with interrupts disabled and sched locked */
void sched_context_exit(void)
{
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
  assert(!cpu_interrupt_getstate());

  sched_queue_pushback(__sched_root(), sched_ctx);
}

/* Must be called with interrupts disabled */
void sched_wait_unlock(sched_queue_root_t *queue)
{
  assert(!cpu_interrupt_getstate());

  __sched_wait_switch(__sched_root(), queue);
}

/* Must be called with interrupts disabled */
void sched_context_stop(void)
{
  assert(!cpu_interrupt_getstate());

  __sched_context_switch(__sched_root());
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

void sched_cpu_init(void)
{
  struct sched_context_s *idle = CPU_LOCAL_ADDR(sched_idle);

  context_init(&idle->context, 128, sched_context_idle, 0);
}


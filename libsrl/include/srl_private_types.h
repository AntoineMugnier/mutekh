
#ifndef SRL_PRIVATE_TYPES_H
#define SRL_PRIVATE_TYPES_H

#include <hexo/types.h>
#include <hexo/error.h>
#include <hexo/lock.h>
#include <hexo/scheduler.h>
#include <srl/srl_public_types.h>
#include <stdint.h>

#include <srl/srl_lock.h>

#include <mwmr/mwmr.h>
#include <soclib/mwmr_controller.h>

typedef struct mwmr_s srl_mwmr_s;
typedef soclib_mwmr_status_s srl_mwmr_status_s;

#define SRL_MWMR_INITIALIZER MWMR_INITIALIZER
#define SRL_MWMR_STATUS_INITIALIZER SOCLIB_MWMR_STATUS_INITIALIZER

#define SRL_CONST_INITIALIZER(x) x

#ifdef CONFIG_PTHREAD

#define SRL_LOCK_INITIALIZER() PTHREAD_MUTEX_INITIALIZER

# define SRL_BARRIER_INITIALIZER PTHREAD_BARRIER_INITIALIZER

#else /* not CONFIG_PTHREAD */

#define SRL_LOCK_INITIALIZER() LOCK_INITIALIZER

typedef struct srl_barrier_s
{
  int_fast8_t count;
  /** blocked threads waiting for read */
  sched_queue_root_t wait;
} srl_barrier_s;

# define SRL_BARRIER_INITIALIZER(n)										\
	{																	\
		.wait = CONTAINER_ROOT_INITIALIZER(sched_queue, __SCHED_CONTAINER_ALGO, HEXO_SPIN),	\
		.count = (n),												\
	}

#endif /* CONFIG_PTHREAD */

typedef void srl_task_func_t( void* );
typedef struct srl_abstract_task_s {
	srl_task_func_t *bootstrap;
	srl_task_func_t *func;
	void *args;
	void *stack;
	size_t stack_size;
	const char *name;
#ifdef CONFIG_PTHREAD
	pthread_t pthread;
#else /* not CONFIG_PTHREAD */
	struct sched_context_s context;
	uint32_t wait_val;
	uint32_t *wait_addr;
#endif
} srl_task_s;

#define SRL_TASK_INITIALIZER(b, f, ss, s, a, n)						   \
	{																   \
		.bootstrap = (srl_task_func_t *)b,							   \
		.func = (srl_task_func_t *)f,								   \
		.args = (void*)a,											   \
		.stack = (void*)s,											   \
		.stack_size = ss / sizeof(reg_t),						   \
		.name = n,											   \
	}

struct srl_abstract_cpustate_s __attribute__((deprecated));
typedef struct srl_abstract_cpustate_s srl_cpustate_s;
struct srl_abstract_cpustate_s {
	int boo;
};

#define SRL_MEMSPACE_INITIALIZER( b, s ) \
{\
	.buffer = b,\
		 .size = s,\
		 }

#define SRL_CPUSTATE_INITIALIZER()									   \
	{																   \
		.boo = 0,													   \
	}

typedef struct srl_abstract_cpudesc_s srl_cpudesc_s;
struct srl_abstract_cpudesc_s {
	size_t ntasks;
	srl_task_s **task_list;
	srl_cpustate_s *state;
};

#define SRL_CPUDESC_INITIALIZER(nt, tl, st)							   \
	{																   \
		.ntasks = nt,												   \
		.task_list = tl,										       \
		.state = st,											       \
	}

#endif

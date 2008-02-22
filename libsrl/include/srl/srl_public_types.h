
#ifndef SRL_PUBLIC_TYPES_H
#define SRL_PUBLIC_TYPES_H

#include <stdint.h>
#include <hexo/types.h>
#include <hexo/lock.h>
#include <mwmr/mwmr.h>

typedef uint32_t srl_const_t;

typedef lock_t *srl_lock_t;

typedef void *srl_buffer_t;

struct srl_memspace_s {
	srl_buffer_t buffer;
	uint32_t size;
};
typedef struct srl_memspace_s *srl_memspace_t;

typedef mwmr_t *srl_mwmr_t;

#ifdef CONFIG_PTHREAD
#include <pthread.h>

typedef pthread_barrier_t *srl_barrier_t;
#else
struct srl_barrier_s;
typedef struct srl_barrier_s *srl_barrier_t;
#endif

#endif

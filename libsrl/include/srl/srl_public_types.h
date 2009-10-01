
#ifndef SRL_PUBLIC_TYPES_H
#define SRL_PUBLIC_TYPES_H

#include <stdint.h>
#include <hexo/types.h>
#include <hexo/lock.h>
#include <mwmr/mwmr.h>

typedef uint32_t srl_const_t;

typedef void *srl_buffer_t;

typedef struct srl_memspace_s {
	srl_buffer_t buffer;
	uint32_t size;
} srl_memspace_s;
typedef srl_memspace_s *srl_memspace_t;

typedef struct mwmr_s *srl_mwmr_t;

#ifdef CONFIG_PTHREAD
#include <pthread.h>

typedef pthread_barrier_t *srl_barrier_t;
typedef pthread_mutex_t *srl_lock_t;
#else
struct srl_barrier_s;
typedef struct srl_barrier_s *srl_barrier_t;
typedef lock_t *srl_lock_t;
#endif

#endif

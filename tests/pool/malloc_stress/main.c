
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

#include <mutek/printk.h>
#include <mutek/mem_alloc.h>

#include <hexo/lock.h>
#include <hexo/endian.h>
#include <hexo/power.h>

#define POOL_SIZE 256		/* number of max different block allocated at the same time */

#ifdef CONFIG_MUTEK_MEMALLOC_SIMPLE
# define MAX_SIZE     256		/* maximum single malloc size */
# define MIN_ALLOCS   4
# define MIN_FREES    0
# define MIN_REALLOC  4
# define MIN_RESERVE  2
# define MIN_ITER     8
# define MAX_ITER     512
#else
# define MAX_SIZE     16384		/* maximum single malloc size */
# define MIN_ALLOCS   8
# define MIN_FREES    4
# define MIN_REALLOC  4
# define MIN_RESERVE  2
# define MIN_ITER     16
# define MAX_ITER     8192
#endif

#define THREAD_COUNT 4

struct block_s
{
  uint32_t size;
  uint8_t *data;
  uint8_t hash[4];
  pthread_mutex_t lock;
};

struct block_s pool[POOL_SIZE] = { };

volatile size_t errors = 0;

static void hash_set(const uint8_t *data, size_t len, uint8_t *hash)
{
  /* FNV hash algorithm */
  uint32_t x = 0x811c9dc5;
  size_t i;
#ifdef CONFIG_SOCLIB_MEMCHECK
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  soclib_mem_check_disable(SOCLIB_MC_CHECK_INIT);
#endif
  for (i = 0; i < len; i++)
    {
      x ^= data[i];
      x *= 0x01000193;
    }
#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_enable(SOCLIB_MC_CHECK_INIT);
  CPU_INTERRUPT_RESTORESTATE;
#endif
  endian_32_na_store(hash, x);
}

static void hash_check(const void *data, size_t len, const uint8_t *hash)
{
  uint8_t result[4];

  hash_set(data, len, result);

  if (memcmp(result, hash, 4)) {
    printk("HASH ERROR at %p, expected %P, got %P\n", data, hash, 4, result, 4);
    hexdumpk((uintptr_t)data, (char*)data - 32, 16);
    errors++;
  }
}

void * thread(void *id_)
{
  size_t id = (size_t)id_;
  size_t i, ac = 0, rc = 0, fc = 0, nc = 0, ar = 0, nr = 0;

  printk("thread %i started\n", id);

  for (i = 0; ac < MIN_ALLOCS || fc < MIN_FREES ||
              rc < MIN_REALLOC || ar < MIN_RESERVE || i < MIN_ITER; i++) {

    uint_fast16_t e = rand() % POOL_SIZE;
    struct block_s *b = pool + e;

    if (i > MAX_ITER) {
      errors++;
      break;
    }

    if (i % 16 == 0) {
      printk("cpu %i thread %i: %i errors, %i alloc, %i free, %i realloc, %i alloc fail, %i reserve, %i reserve fail\n",
             cpu_id(), id, errors, ac, fc, rc, nc, ar, nr);
      mem_check();
    }

    pthread_mutex_lock(&b->lock);

    uint32_t r = rand() % 100;
    switch (r) {

    case 0 ... 30:			/* switch */
      pthread_yield();
      break;

    case 31 ... 70: {			/* free/malloc */

      if (b->data) {
	hash_check(b->data, b->size, b->hash);
	fc++;
	free(b->data);
	b->data = NULL;
	b->size = 0;
      }

      if (r <= 60)              /* malloc */
        {
          size_t size = rand() % MAX_SIZE;
          size_t align = 1 << (rand() % 9);
          void *data = mem_alloc_align(size, align, mem_scope_sys);
          if (data) {
            assert((uintptr_t)data % align == 0);
            b->data = data;
            b->size = size;
            memset(data, rand(), size);
            hash_set(data, size, b->hash);
            ac++;
          } else {
            if (size)
              nc++;
          }
        }
      else                      /* reserve */
        {
          uint8_t *start = (uint8_t*)memory_allocator_region_address(default_region) +
            rand() % memory_allocator_region_size(default_region);
          size_t size = rand() % MAX_SIZE;
          // printk("reserve %p %u\n", start, size);
          void *data = memory_allocator_reserve(default_region, start, size);
          if (data) {
            b->data = data;
            b->size = size;
            memset(data, rand(), size);
            hash_set(data, size, b->hash);
            ar++;
          } else {
            if (size)
              nr++;
          }
        }

      break;
    }

    case 71 ... 100: {			/* realloc */

      if (b->data)
	hash_check(b->data, b->size, b->hash);

      size_t size = rand() % MAX_SIZE;
      void *data = realloc(b->data, size);
      if (data || size == 0) {
	b->data = data;
	b->size = size;
        if (data) {
          memset(data, rand(), size);
          hash_set(data, size, b->hash);
          rc++;
        }
      } else {
        if (size)
          nc++;
      }

      break;
    }

    }

    pthread_mutex_unlock(&b->lock);
  }

  mem_check();

  return NULL;
}

pthread_t threads[THREAD_COUNT];

void main()
{
  size_t i;

  for (i = 0; i < POOL_SIZE; i++)
    pthread_mutex_init(&pool[i].lock, NULL);

  for (i = 0; i < THREAD_COUNT; i++)
    pthread_create(threads + i, NULL, thread, (void*)i);

  for (i = 0; i < THREAD_COUNT; i++)
    pthread_join(threads[i], NULL);

  printk("++DONE++%u++\n", errors);

  power_shutdown();
  power_reboot();
}


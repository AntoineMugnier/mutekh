#ifndef MUTEK_BUFFER_POOL_H_
#define MUTEK_BUFFER_POOL_H_

#include <gct_platform.h>
#include <gct_lock_hexo_lock_irq.h>
#include <gct_atomic.h>

#include <gct/container_slist.h>

#include <hexo/error.h>

#include "slab.h"

/** @internal */
#define GCT_CONTAINER_ALGO_buffer_queue SLIST
#define GCT_CONTAINER_LOCK_buffer_queue HEXO_LOCK_IRQ

struct buffer_pool_s;

struct buffer_s
{
  GCT_CONTAINER_ENTRY(buffer_queue, entry);
  struct buffer_pool_s *pool;

  uint16_t begin;
  uint16_t end;
  uint8_t data[0];
};

GCT_CONTAINER_TYPES(buffer_queue, struct buffer_s *, entry);
GCT_CONTAINER_FCNS(buffer_queue, ALWAYS_INLINE, buffer_queue,
                   init, destroy, push, pop, pushback, next, head, tail, isempty,
                   clear, count);

struct buffer_pool_s {
    struct slab_s slab;
};

void buffer_pool_init(
    struct buffer_pool_s *pool,
    size_t data_size,
    slab_grow_func_t grow,
    enum mem_scope_e scope);

void buffer_pool_cleanup(struct buffer_pool_s *pool);

inline
struct buffer_s *buffer_pool_alloc(struct buffer_pool_s *pool)
{
    struct buffer_s *buffer = slab_alloc(&pool->slab);

    if (!buffer)
        return buffer;

    buffer->pool = pool;
    buffer->begin = 0;
    buffer->end = pool->slab.unit_size - sizeof(struct buffer_s);

    return buffer;
}

inline
void buffer_free(struct buffer_s *buffer)
{
    slab_free(&buffer->pool->slab, buffer);
}

ALWAYS_INLINE
size_t buffer_pool_unit_size(const struct buffer_pool_s *pool)
{
  return pool->slab.unit_size - sizeof(struct buffer_s);
}

ALWAYS_INLINE
size_t buffer_size(const struct buffer_s *buffer)
{
  return buffer_pool_unit_size(buffer->pool);
}

ALWAYS_INLINE
size_t buffer_available(const struct buffer_s *buffer)
{
  return buffer_pool_unit_size(buffer->pool) - buffer->end;
}

inline
error_t buffer_prepend(struct buffer_s *buffer,
                       const uint8_t *data, size_t size)
{
  if (buffer->begin < size)
    return -ENOMEM;

  buffer->begin -= size;
  memcpy(buffer->data + buffer->begin, data, size);

  return 0;
}

inline
error_t buffer_append(struct buffer_s *buffer,
                   const uint8_t *data, size_t size)
{
  if (buffer->end - buffer->begin < size)
    return -ENOMEM;

  buffer->end += size;
  memcpy(buffer->data + buffer->begin, data, size);

  return 0;
}

#endif

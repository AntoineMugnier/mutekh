#ifndef MUTEK_BUFFER_POOL_H_
#define MUTEK_BUFFER_POOL_H_

#include <gct_platform.h>
#include <gct_lock_hexo_lock_irq.h>
#include <gct_atomic.h>

#include <gct/refcount.h>
#include <gct/container_clist.h>

#include <hexo/error.h>

#include "slab.h"

/** @internal */
#define GCT_CONTAINER_ALGO_buffer_queue CLIST
#define GCT_CONTAINER_LOCK_buffer_queue HEXO_LOCK_IRQ
#define GCT_CONTAINER_REFCOUNT_buffer_queue buffer

struct buffer_pool_s;

struct buffer_s
{
  GCT_REFCOUNT_ENTRY(obj_entry);
  GCT_CONTAINER_ENTRY(buffer_queue, entry);

  struct buffer_pool_s *pool;

  uint16_t begin;
  uint16_t end;
  uint8_t data[0];
};

GCT_REFCOUNT(buffer, struct buffer_s *, obj_entry);

GCT_CONTAINER_TYPES(buffer_queue, struct buffer_s *, entry);
GCT_CONTAINER_FCNS(buffer_queue, ALWAYS_INLINE, buffer_queue,
                   init, destroy, push, pop, pushback, next, head, isempty,
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

struct buffer_s *buffer_pool_alloc(struct buffer_pool_s *pool);

ALWAYS_INLINE
size_t buffer_pool_unit_size(const struct buffer_pool_s *pool)
{
  return pool->slab.unit_size - sizeof(struct buffer_s);
}

/** @internal */
void buffer_destroy(struct buffer_s *buffer);

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

ALWAYS_INLINE
error_t buffer_prepend(struct buffer_s *buffer,
                       const uint8_t *data, size_t size)
{
  if (buffer->begin < size)
    return -ENOMEM;

  buffer->begin -= size;
  memcpy(buffer->data + buffer->begin, data, size);

  return 0;
}

ALWAYS_INLINE
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

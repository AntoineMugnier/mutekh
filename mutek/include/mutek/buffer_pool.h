#ifndef MUTEK_BUFFER_POOL_H_
#define MUTEK_BUFFER_POOL_H_

#include <gct_platform.h>
#include <gct_lock_hexo_lock_irq.h>
#include <gct_atomic.h>

#include <gct/refcount.h>

#include "slab.h"

struct buffer_pool_s;

struct buffer_s
{
    GCT_REFCOUNT_ENTRY(obj_entry);
    struct buffer_pool_s *pool;

    uint16_t size;
    uint8_t data[0];
};

GCT_REFCOUNT(buffer, struct buffer_s *, obj_entry);

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

/** @internal */
void buffer_destroy(struct buffer_s *buffer);

#endif

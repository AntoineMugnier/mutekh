/*
    This file is part of MutekH.
    
    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.
    
    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2014
*/

#include "include/mutek/mem_alloc.h"
#include "include/mutek/buffer_pool.h"

void buffer_pool_init(
    struct buffer_pool_s *pool,
    size_t data_size,
    slab_grow_func_t grow,
    enum mem_scope_e scope)
{
    slab_init(&pool->slab, sizeof(struct buffer_s) + data_size,
              grow, scope);
}

void buffer_pool_cleanup(struct buffer_pool_s *pool)
{
    slab_cleanup(&pool->slab);
}

void buffer_destroy(struct buffer_s *buffer)
{
    buffer_refcleanup(buffer);
    slab_free(&buffer->pool->slab, buffer);
}

struct buffer_s *buffer_pool_alloc(struct buffer_pool_s *pool)
{
    struct buffer_s *buffer = slab_alloc(&pool->slab);

    if (!buffer)
        return buffer;

    buffer->pool = pool;
    buffer_refinit(buffer);
    buffer->begin = 0;
    buffer->end = pool->slab.unit_size - sizeof(struct buffer_s);

    return buffer;
}

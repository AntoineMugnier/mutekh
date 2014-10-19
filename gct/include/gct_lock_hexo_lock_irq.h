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
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2014

*/

/**
 * @file
 * @module{Generic C templates}
 * @short Hexo spin lock with IRQ disable and restore for GCT
 */

#ifndef __GCT_LOCK_HEXO_LOCK_IRQ_H__
#define __GCT_LOCK_HEXO_LOCK_IRQ_H__

#pragma GCC system_header

#include <hexo/decls.h>

C_HEADER_BEGIN

#include <gct_platform.h>
#include <gct/_platform.h>

#if !defined(_GCT_LIBRARY_BUILD)
# include <gct/_container_access.h>
#endif

#include <hexo/lock.h>

#define _GCT_LOCK_HEXO_LOCK_IRQ_SPINNING 1
#define _GCT_LOCK_HEXO_LOCK_IRQ_READ_WRITE 0
#define _GCT_LOCK_HEXO_LOCK_IRQ_RECURSIVE 0

#define _GCT_LOCK_HEXO_LOCK_IRQ_INITIALIZER    LOCK_INITIALIZER
#define _GCT_LOCK_HEXO_LOCK_IRQ_STATIC_INIT 1

typedef lock_irq_t _gct_lock_HEXO_LOCK_IRQ_type_t;

_GCT_INTERNAL_INLINE(
  gct_err_t _gct_lock_HEXO_LOCK_IRQ_init(lock_irq_t *lock),
{
  lock_init_irq(lock);
  return 0;
});

_GCT_INTERNAL_INLINE(
  void _gct_lock_HEXO_LOCK_IRQ_destroy(lock_irq_t *lock),
{
  lock_destroy_irq(lock);
});

_GCT_INTERNAL_INLINE(
  void _gct_lock_HEXO_LOCK_IRQ_wrlock(lock_irq_t *lock),
{
  lock_spin_irq(lock);
});

_GCT_INTERNAL_INLINE(
  void _gct_lock_HEXO_LOCK_IRQ_rdlock(lock_irq_t *lock),
{
  lock_spin_irq(lock);
});

_GCT_INTERNAL_INLINE(
  void _gct_lock_HEXO_LOCK_IRQ_rdwrlock(lock_irq_t *lock),
{
  lock_spin_irq(lock);
});

_GCT_INTERNAL_INLINE(
  void _gct_lock_HEXO_LOCK_IRQ_promotelock(lock_irq_t *lock),
{
});

_GCT_INTERNAL_INLINE(
  void _gct_lock_HEXO_LOCK_IRQ_unlock(lock_irq_t *lock),
{
  lock_release_irq(lock);
});

C_HEADER_END

#endif


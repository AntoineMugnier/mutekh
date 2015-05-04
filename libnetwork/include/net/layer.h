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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2015
*/

#ifndef NET_LAYER_H
#define NET_LAYER_H

#include <hexo/decls.h>
#include <hexo/lock.h>

#include <gct_platform.h>
#include <gct_lock_hexo_lock_irq.h>
#include <gct_atomic.h>

#include <gct/refcount.h>
#include <gct/container_clist.h>

#include <assert.h>

#include "scheduler.h"

#define NET_LAYER_TYPE(d, c, b, a) (((a) << 24) | ((b) << 16) | ((c) << 8) | (d))

struct net_task_header_s;
struct net_layer_s;

struct net_layer_context_s
{
  struct net_addr_s addr;
  uint16_t prefix_size;
  uint16_t mtu;
};

struct net_layer_handler_s
{
  void (*destroyed)(
    struct net_layer_s *layer);

  void (*task_handle)(
    struct net_layer_s *layer,
    struct net_task_header_s *task);

  /**
     @this notifies a layer its parent's context changed.

     @this must return whether this layer's context gets updated
     because this call.  Context update notification will be forwarded
     to children layer if so.
   */
  bool_t (*context_updated)(
    struct net_layer_s *layer,
    const struct net_layer_context_s *parent_context);

  error_t (*bound)(
    struct net_layer_s *layer,
    void *addr,
    struct net_layer_s *child);

  void (*unbound)(
    struct net_layer_s *layer,
    struct net_layer_s *child);
};

#define GCT_CONTAINER_ALGO_net_layer_list CLIST
#define GCT_CONTAINER_REFCOUNT_net_layer_list net_layer

GCT_CONTAINER_TYPES(net_layer_list,
struct net_layer_s
{
  GCT_REFCOUNT_ENTRY(obj_entry);
  GCT_CONTAINER_ENTRY(net_layer_list, entry);
  net_layer_list_root_t children;

  struct net_scheduler_s *scheduler;
  const struct net_layer_handler_s *handler;
  struct net_layer_s *parent;
  struct net_layer_context_s context;

  uint32_t type;

  // Rest is done through derivation
} *, entry);

GCT_REFCOUNT(net_layer, struct net_layer_s *, obj_entry);

#if 0
#define net_layer_refinc(x) ({ printk("Layer %S refinc %d\n", &(x)->type, 4, (x)->obj_entry._count.value); net_layer_refinc(x); })
#define net_layer_refdec(x) do{ printk("Layer %S refdec %d\n", &(x)->type, 4, (x)->obj_entry._count.value); net_layer_refdec(x); }while(0)
#endif

GCT_CONTAINER_FCNS(net_layer_list, static inline, net_layer_list,
                   init, destroy, push, pop, pushback, next, head, isempty, remove, foreach);

/* Refcount destroy function */
void net_layer_destroy(
  struct net_layer_s *layer);

error_t net_layer_bind(
  struct net_layer_s *layer,
  void *addr,
  struct net_layer_s *child);

void net_layer_unbind(
  struct net_layer_s *layer,
  struct net_layer_s *child);

/**
   @this allocates a packet for layer.

   @param layer Layer to allocate packet for
   @param size reserved header size for parent layers
   @param size minimal data size to allocate packet for
 */
ALWAYS_INLINE
struct buffer_s *net_layer_packet_alloc(
  struct net_layer_s *layer,
  size_t begin,
  size_t size)
{
  struct buffer_s *pkt = net_scheduler_packet_alloc(layer->scheduler);
  pkt->begin = begin;
  pkt->end = begin + size;

  return pkt;
}

void net_layer_context_changed(struct net_layer_s *layer);

/**
   @this initializes a network layer.

   @param type Layer fourCC type, tells what protocol is implemented by layer.
   @param max_header_size Maximal header size to account for when allocating packet
 */
error_t net_layer_init(
  struct net_layer_s *layer,
  const struct net_layer_handler_s *handler,
  struct net_scheduler_s *scheduler,
  uint32_t type);

#endif

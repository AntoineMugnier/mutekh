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

/**
   @file
   @module{Network stack library}
   @short Network layer common structure

   @section {Description}

   A network layer is a part of a networking stack.  Depending on
   actual needs, layer may be statically or dynamically allocated on a
   per-session basis.

   For instance, in a TCP/IP stack, Ethernet, IP and TCP layers are
   long-lived, whereas TCP session layer (for a single source/target
   IP/port couple) is shorter-lived.

   Layers communicate through tasks that are executed in a unique
   scheduler for a given stack.

   Layers are stacked in a tree fashion.  Root is close to the network
   interfaces, leaves are upper-layer sessions.  A layer has a single
   parent layer in stack, but may have multiple children layers.

   All layers are refcounted.  Parents hold a refcount on children
   (and not the other way around).  An interface layer that gets
   destroyed (because the communication medium disappears) implicitly
   drops references on upper layers which will, in turn, get
   destroyed.

   Each layer should check for parent validity before sending a task
   to it.

   Communication at borders of libnetwork (between libnetwork and
   other modules) is done through a delegate pattern.  Each layer is
   responsible for declaration of its delegate vtable and API.

   @end section
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

struct net_task_header_s;
struct net_layer_s;

/**
   @this defines context a layer can live in.  Context contains
   parameters with respect to parent layers.
 */
struct net_layer_context_s
{
  /** Upper-layer addresses */
  struct net_addr_s addr;
  /** Byte count layer should leave available for upper layers */
  uint16_t prefix_size;
  /** Maximum byte count this layer is allowed to use for its payloads */
  uint16_t mtu;
};

/**
   @this is a vtable for a layer
 */
struct net_layer_handler_s
{
  /**
     @this is called after a layer gets destroyed because its refcount
     dropped down to 0.
   */
  void (*destroyed)(
    struct net_layer_s *layer);

  /**
     @this is called when a task requires handling from the layer.  It
     is responsability from this call to destroy the task (or reuse
     it).
   */
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

  /**
     @this is called when another layer is bound using this one as
     parent.  @tt addr is address context passed to
     @ref{net_layer_bind}, it is context-dependent.
   */
  error_t (*bound)(
    struct net_layer_s *layer,
    void *addr,
    struct net_layer_s *child);

  /**
     @this is called when a layer that was bound before gets unbound.
   */
  void (*unbound)(
    struct net_layer_s *layer,
    struct net_layer_s *child);

  uint32_t type;
  bool_t use_timer;
};

#define GCT_CONTAINER_ALGO_net_layer_list CLIST
#define GCT_CONTAINER_REFCOUNT_net_layer_list net_layer

struct net_layer_delegate_vtable_s
{
  void (*release)(void *delegate, struct net_layer_s *layer);
};

GCT_CONTAINER_TYPES(net_layer_list,
/**
   @this is base structure for a layer.  It should be inherited from
   by actual implementations.
 */
struct net_layer_s
{
  GCT_REFCOUNT_ENTRY(obj_entry);
  GCT_CONTAINER_ENTRY(net_layer_list, entry);
  net_layer_list_root_t children;

  struct net_scheduler_s *scheduler;
  const struct net_layer_handler_s *handler;
  struct net_layer_s *parent;
  struct net_layer_context_s context;
  void *delegate;
  const struct net_layer_delegate_vtable_s *delegate_vtable;

  // Rest is done through derivation
} *, entry);

GCT_REFCOUNT(net_layer, struct net_layer_s *, obj_entry);

GCT_CONTAINER_FCNS(net_layer_list, static inline, net_layer_list,
                   init, destroy, push, pop, pushback, next, head, isempty, remove, foreach);

/* Refcount destroy function. @internal */
void net_layer_destroy(
  struct net_layer_s *layer);

/**
   @this binds a child layer to another layer.

   @param parent Parent layer
   @param addr Address data (parent context specific)
   @param child Child layer

   @returns whether parent accepted layer as a child
 */
error_t net_layer_bind(
  struct net_layer_s *parent,
  void *addr,
  struct net_layer_s *child);

/**
   @this unbinds a child layer from another layer.
*/
void net_layer_unbind(
  struct net_layer_s *layer,
  struct net_layer_s *child);

/**
   @this allocates a packet for layer.

   @param layer Layer to allocate packet for
   @param begin reserved header size for layers headers
          (should be at least @tt{layer->context.prefix_size})
   @param size minimal data size to allocate packet for
          (should be no more than @tt{layer->context.mtu})
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

/**
   @this must be called by layer when @ref{net_layer_context_s}{its
   context} changes.
 */
void net_layer_context_changed(struct net_layer_s *layer);

/**
   @this initializes a network layer.

   @param handler Pointer to a vtable
   @param scheduler Context scheduler this layer runs in
   @param type Layer type as returned by @ref NET_LAYER_TYPE, hints
          about what protocol is implemented by layer
 */
error_t net_layer_init(
  struct net_layer_s *layer,
  const struct net_layer_handler_s *handler,
  struct net_scheduler_s *scheduler,
  void *delegate,
  const struct net_layer_delegate_vtable_s *delegate_vtable);

#endif

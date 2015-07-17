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
   @module {Network stack library}
   @short Network layer base structure

   @this contains all definitions around @ref {net_layer_s} {Network
   layer structure}.
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

#include <mutek/buffer_pool.h>

#include <assert.h>

#include <net/addr.h>

struct net_task_s;
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
  void (*destroyed)(struct net_layer_s *layer);

  /**
     @this is called when a task requires handling from the layer.  It
     is responsability from this call to destroy the task (or reuse
     it).
   */
  void (*task_handle)(struct net_layer_s *layer,
                      struct net_task_s *task);

  /**
     @this is responsible for calculating children's context.
   */
  void (*child_context_adjust)(const struct net_layer_s *layer,
                               struct net_layer_context_s *ctx);

  /**
     @this notifies a layer its own context changed
   */
  void (*context_changed)(struct net_layer_s *layer);

  /**
     @this notifies a layer its got unbound and has no parent any
     more.
   */
  void (*dandling)(struct net_layer_s *layer);

  /**
     @this is called when another layer is bound using this one as
     parent.  @tt addr is address context passed to
     @ref{net_layer_bind}, it is context-dependent.

     If this function returns an error, layer binding is aborted.
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
};

#define GCT_CONTAINER_ALGO_net_layer_list CLIST
#define GCT_CONTAINER_REFCOUNT_net_layer_list net_layer
#define GCT_CONTAINER_ALGO_net_layer_sched_list CLIST

/**
   @this is the base vtable for a delegate.  All delegate must
   implement functions contained herein.

   Specific layer definitions may inherit this vtable as base.
 */
struct net_layer_delegate_vtable_s
{
  /**
     @this notifies delegate the layer got destroyed, implying
     delegate is released from referencing layer.
   */
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
  GCT_CONTAINER_ENTRY(net_layer_sched_list, scheduler_ref);

  struct net_scheduler_s *scheduler;
  const struct net_layer_handler_s *handler;
  struct net_layer_s *parent;
  struct net_layer_context_s context;
  void *delegate;
  const struct net_layer_delegate_vtable_s *delegate_vtable;

  // Rest is done through derivation
} *, entry);

GCT_REFCOUNT(net_layer, struct net_layer_s *, obj_entry);
GCT_CONTAINER_TYPES(net_layer_sched_list, struct net_layer_s *, scheduler_ref);

GCT_CONTAINER_FCNS(net_layer_list, static inline, net_layer_list,
                   init, destroy, push, pop, pushback, next, head, isempty, remove, foreach);

/* Refcount destroy function. Called from refcount
   management. @internal */
void net_layer_destroy(struct net_layer_s *layer);

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
   @this unbinds all children layers from a layer.
*/
void net_layer_unbind_all(struct net_layer_s *layer);

/**
   @this must be called by layer when @ref{net_layer_context_s}{its
   context} changes.
 */
void net_layer_context_changed(struct net_layer_s *layer);

/**
   @this initializes a network layer.

   @param layer Layer to initialize
   @param handler Pointer to a vtable
   @param scheduler Context scheduler this layer runs in
   @param delegate Delegate private data pointer
   @param delegate_vtable Vtable for delegate, mandatory if @tt delegate is set
 */
error_t net_layer_init(
  struct net_layer_s *layer,
  const struct net_layer_handler_s *handler,
  struct net_scheduler_s *scheduler,
  void *delegate,
  const struct net_layer_delegate_vtable_s *delegate_vtable);

#endif

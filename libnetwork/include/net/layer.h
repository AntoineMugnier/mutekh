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
   @module {Libraries::Abstract network stack}
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
struct net_scheduler_s;

/**
   @this defines context a layer can live in.  Context contains
   parameters with respect to parent layers.
 */
struct net_layer_context_s
{
  /** Upper-layer addresses */
  struct net_addr_s addr;
  /** Byte count layer should leave available for upper layers at the
      begining of a packet */
  uint16_t prefix_size;
  /** Maximum byte count this layer is allowed to use for its
      payloads, starting from @tt prefix_size offset in @tt
      buffers. */
  uint16_t mtu;
};

/**
   @this is a vtable for a layer
 */
struct net_layer_handler_s
{
  /**
     @this is called after a layer gets destroyed because its refcount
     dropped down to 0.  This method is mandatory.
   */
  void (*destroyed)(struct net_layer_s *layer);

  /**
     @this is called when a task requires handling from the layer.  It
     is responsability from this call to destroy the task (or reuse
     it).  This method is mandatory.
   */
  void (*task_handle)(struct net_layer_s *layer,
                      struct net_task_s *task);

  /**
     @this is responsible for calculating children's context.  This
     method is optional.
   */
  void (*child_context_adjust)(const struct net_layer_s *layer,
                               struct net_layer_context_s *ctx);

  /**
     @this notifies a layer its own context changed.  This method is
     optional.
   */
  void (*context_changed)(struct net_layer_s *layer);

  /**
     @this notifies a layer its got unbound and has no parent any
     more.  This method is optional.
   */
  void (*dangling)(struct net_layer_s *layer);

  /**
     @this is called when another layer is bound using this one as
     parent.  @tt addr is address context passed to
     @ref{net_layer_bind}, it is context-dependent.

     If this function returns an error, layer binding is aborted.
     This method is optional.  If not implemented, no layer may be
     bound as child of this one.
   */
  error_t (*bound)(
    struct net_layer_s *layer,
    void *addr,
    struct net_layer_s *child);

  /**
     @this is called when a child layer gets unbound.  This method is
     optional.
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
  /** @multiple @internal */
  GCT_REFCOUNT_ENTRY(obj_entry);
  GCT_CONTAINER_ENTRY(net_layer_list, entry);
  net_layer_list_root_t children;
  GCT_CONTAINER_ENTRY(net_layer_sched_list, scheduler_ref);

  const struct net_layer_handler_s *handler;

  /** Layer's scheduler. May be dereferenced for using its timing and
      allocation functions. */
  struct net_scheduler_s *scheduler;
  /** Layer's parent in stack. Should be checked for NULL before
      using. */
  struct net_layer_s *parent;
  /** Layer's context, as set by parent.  Layer is notified of changes
      in this structure through @ref
      {net_layer_handler_s::context_changed} method. */
  struct net_layer_context_s context;

  /** Delegate pointer */
  void *delegate;
  /** Delegate vtable */
  const struct net_layer_delegate_vtable_s *delegate_vtable;

  // Internal data through inheritance
} *, entry);

GCT_REFCOUNT(net_layer, struct net_layer_s *, obj_entry);
GCT_CONTAINER_TYPES(net_layer_sched_list, struct net_layer_s *, scheduler_ref);

GCT_CONTAINER_FCNS(net_layer_list, static inline, net_layer_list,
                   init, destroy, push, pop, pushback, next, head, isempty, remove, foreach);

/** @this is refcount destroy function, called from refcount
   management. @internal */
void net_layer_destroy(struct net_layer_s *layer);

/**
   @this binds a child layer to another layer.  Caller should refdec
   child layer after this call, whatever the return value.  If
   accepted, reference will be retained by new parent, if rejected,
   child layer should be destroyed anyway.

   @param parent Parent layer
   @param addr Address data (parent context specific)
   @param child Child layer

   @returns error if parent refused layer as a child
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

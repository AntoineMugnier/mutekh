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
   @short Network task
  
   @section {Description}
  
   A network task is a work load to be handled by a given layer.  It may
   be of various types among:
   @list
   @item a task happening after a delay,
   @item a packet to handle,
   @item a request to respond to,
   @item a response to a request,
   @item a one-way notification,
   @item a layer-specific task.
   @end list
  
   A task always has a source layer and a target layer.  A layer may
   send a task to itself.
  
   There are various helpers to intialize and send tasks directly.
  
   @end section
 */

#ifndef NET_TASK_H
#define NET_TASK_H

#include <hexo/types.h>

#include <gct_platform.h>
#include <gct_lock_hexo_lock_irq.h>
#include <gct_atomic.h>

#include <gct/refcount.h>
#include <gct/container_clist.h>

#include <device/class/timer.h>

#include <mutek/buffer_pool.h>

#include <net/addr.h>

#define GCT_CONTAINER_ALGO_net_task_queue CLIST
#define GCT_CONTAINER_LOCK_net_task_queue HEXO_LOCK_IRQ

struct net_scheduler_s;
struct net_task_s;
struct net_task_header_s;
struct net_layer_s;

/**
   @this is called when a task gets destroyed.
 */
typedef void net_task_destroy_func_t(
  struct net_task_header_s *task);

/**
   @this is a type for tasks.
 */
enum net_task_type_e
{
  NET_TASK_CUSTOM,
  NET_TASK_INBOUND,
  NET_TASK_TIMEOUT,
  NET_TASK_QUERY,
  NET_TASK_RESPONSE,
  NET_TASK_NOTIFICATION,
};

/**
   @this is a basic task header structure.  Inheriting this structure
   is mandatory for custom types.

   A destroy function must be set for each task.  When a task is
   cleaned up, its destroy function will be called.

   Basic task types are defined in @ref {net_task_s} and handle all
   this structure fields.
 */
struct net_task_header_s
{
  GCT_CONTAINER_ENTRY(net_task_queue, queue_entry);

  /** Destroy function */
  net_task_destroy_func_t *destroy_func;
  /** Destroy function private data */
  void *allocator_data;

  /** Must be filled, must retain a reference.  This is implicitly
      done by standard functions. */
  struct net_layer_s *source;

  /** Must be filled, must retain a reference.  This is implicitly
      done by standard functions. */
  struct net_layer_s *target;

  enum net_task_type_e type;
};

GCT_CONTAINER_TYPES(net_task_queue, struct net_task_header_s *, queue_entry);

/**
   @this pushes a task to a given target, for a given type.

   @param header Task header to push
   @param target Target layer, mandatory
   @param source Source layer, mandatory
   @param type Task type
 */
void net_task_push(struct net_task_header_s *header,
                     struct net_layer_s *target,
                     struct net_layer_s *source,
                     enum net_task_type_e type);

/**
   @this destroys a task.  This calls its destroy function.
 */
void net_task_cleanup(struct net_task_s *task);

/**
   @this is the structure for standard tasks.
 */
struct net_task_s
{
  struct net_task_header_s header;

  union {
    /**
       Inbound task must have a reference on packet, and a reference on
       source layer.

       Inbound task may be forwarded from layer to layer.  If so,
       reference to source and owner layers must be updated accordingly.
    */
    struct {
      dev_timer_value_t timestamp;

      struct net_addr_s src_addr;
      struct net_addr_s dst_addr;

      /** Must be filled, must retain a reference */
      struct buffer_s *buffer;
    } inbound;

    /**
       Deadline is relative to scheduler's timer device.
    */
    struct {
      dev_timer_value_t deadline;
      dev_timer_value_t precision;
    } timeout;

    struct {
      uint32_t opcode;
    } notification;

    struct {
      uint32_t opcode;
    } query;

    struct {
      error_t err;
    } response;
  };
};

STRUCT_COMPOSE(net_task_s, header);

/**
   @this pushes an inbound packet task to a layer.

   @this sets all structure fields and pushes the task.
 */
void net_task_inbound_push(struct net_task_s *task,
                           struct net_layer_s *target,
                           struct net_layer_s *source,
                           dev_timer_value_t timestamp,
                           const struct net_addr_s *src_addr,
                           const struct net_addr_s *dst_addr,
                           struct buffer_s *buffer);

/**
   Forward an inbound task to another layer without changing timestamp
   and buffer.

   Task must not be cleaned up for before calling this function.
*/
void net_task_inbound_forward(struct net_task_s *task,
                              struct net_layer_s *target);

/**
   @this pushes a timetout task to a layer.  Source layer is same as
   target layer.
 */
void net_task_timeout_push(struct net_task_s *task,
                           struct net_layer_s *target,
                           dev_timer_value_t deadline,
                           dev_timer_value_t precision);

/**
   @this pushes a notification task to a layer.  @tt opcode meaning is
   source-layer defined.
 */
void net_task_notification_push(struct net_task_s *task,
                                struct net_layer_s *target,
                                struct net_layer_s *source,
                                uint32_t opcode);

/**
   @this pushes a query to a layer.  @tt opcode is requester-layer
   defined.  Responder should use @ref net_task_query_respond_push
   with the same task for answering.
 */
void net_task_query_push(struct net_task_s *task,
                         struct net_layer_s *target,
                         struct net_layer_s *requester,
                         uint32_t opcode);

/**
   Transform a query task into its response counterpart.  Query target
   and requester fields are used as response responder and target
   fields.
 */
void net_task_query_respond_push(struct net_task_s *task, error_t err);

#endif

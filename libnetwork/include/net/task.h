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
struct net_stack_s;
struct net_layer_s;

void net_scheduler_task_push(
  struct net_scheduler_s *sched,
  struct net_task_header_s *task);

typedef void net_task_destroy_func_t(
  struct net_task_header_s *task);

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
   Tasks are network stack basic workload that must be handled later
   on.  Network stack may call layer with its own task when ready.

   It is up to layers to inherit this structure if they require custom
   tasks.
 */
struct net_task_header_s
{
  GCT_CONTAINER_ENTRY(net_task_queue, queue_entry);

  net_task_destroy_func_t *destroy_func;
  void *allocator_data;

  /** Must be filled, must retain a reference */
  struct net_layer_s *source;

  /** Must be filled, must retain a reference */
  struct net_layer_s *target;

  enum net_task_type_e type;
};

GCT_CONTAINER_TYPES(net_task_queue, struct net_task_header_s *, queue_entry);

void net_task_push(struct net_task_header_s *header,
                     struct net_layer_s *target,
                     struct net_layer_s *source,
                     enum net_task_type_e type);

void net_task_cleanup(struct net_task_s *task);

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

   Tash must not be cleared before calling this task.
*/
void net_task_inbound_forward(struct net_task_s *task,
                                struct net_layer_s *target);

void net_task_timeout_push(struct net_task_s *task,
                           struct net_layer_s *target,
                           dev_timer_value_t deadline,
                             dev_timer_value_t precision);

void net_task_notification_push(struct net_task_s *task,
                                struct net_layer_s *target,
                                struct net_layer_s *source,
                                  uint32_t opcode);

void net_task_query_push(struct net_task_s *task,
                         struct net_layer_s *target,
                         struct net_layer_s *requester,
                           uint32_t opcode);

/**
   Transform a query task into its response counterpart.  Target and
   query requester fields are taken to use as response responder and
   target fields.
 */
void net_task_query_respond_push(struct net_task_s *task, error_t err);

#endif

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

#include <mutek/printk.h>

#include <net/task.h>
#include <net/layer.h>
#include <net/scheduler.h>

void net_task_destroy(struct net_task_s *task)
{
  struct net_layer_s *source = task->source;
  struct net_layer_s *target = task->target;

  switch (task->type) {
  case NET_TASK_INBOUND:
  case NET_TASK_OUTBOUND:
    if (task->packet.buffer)
      buffer_refdec(task->packet.buffer);
    break;

  default:
    break;
  }

  task->packet.buffer = 0x55aa55aa;
  task->target = 0x55aa55aa;

  task->destroy_func(task);

  if (source)
    net_layer_refdec(source);
  if (target)
    net_layer_refdec(target);
}

void net_task_push(struct net_task_s *task,
                   struct net_layer_s *target,
                   struct net_layer_s *source,
                   enum net_task_type_e type)
{
  task->target = net_layer_refinc(target);
  task->source = net_layer_refinc(source);
  task->type = type;

  assert(task->destroy_func);

  //printk("Task %p push %d\n", task, &task->target->handler->type);

  net_scheduler_task_push(target->scheduler, task);
}

void net_task_inbound_push(struct net_task_s *task,
                           struct net_layer_s *target,
                           struct net_layer_s *source,
                           dev_timer_value_t timestamp,
                           const struct net_addr_s *src_addr,
                           const struct net_addr_s *dst_addr,
                           struct buffer_s *buffer)
{
  task->packet.timestamp = timestamp;
  task->packet.buffer = buffer_refinc(buffer);

  assert(buffer);

  if (src_addr)
    task->packet.src_addr = *src_addr;
  else
    memset(&task->packet.src_addr, 0, sizeof(struct net_addr_s));

  if (dst_addr)
    task->packet.dst_addr = *dst_addr;
  else
    memset(&task->packet.dst_addr, 0, sizeof(struct net_addr_s));

  //printk("Task %p forward <- %d\n", task, &source->handler->type);

  net_task_push(task, target, source, NET_TASK_INBOUND);
}

void net_task_outbound_push(struct net_task_s *task,
                           struct net_layer_s *target,
                           struct net_layer_s *source,
                           dev_timer_value_t timestamp,
                           const struct net_addr_s *src_addr,
                           const struct net_addr_s *dst_addr,
                           struct buffer_s *buffer)
{
  task->packet.timestamp = timestamp;
  task->packet.buffer = buffer_refinc(buffer);

  assert(buffer);

  if (src_addr)
    task->packet.src_addr = *src_addr;
  else
    memset(&task->packet.src_addr, 0, sizeof(struct net_addr_s));

  if (dst_addr)
    task->packet.dst_addr = *dst_addr;
  else
    memset(&task->packet.dst_addr, 0, sizeof(struct net_addr_s));

  //printk("Task %p forward <- %d\n", task, &source->handler->type);

  net_task_push(task, target, source, NET_TASK_OUTBOUND);
}

void net_task_packet_forward(struct net_task_s *task,
                              struct net_layer_s *target)
{
  struct net_layer_s *old_source = task->source;
  struct net_layer_s *old_target = task->target;

  //printk("Task %p forward <- %d\n", task, &task->packet.source->handler->type);

  net_task_push(task, target, old_source, task->type);

  net_layer_refdec(old_source);
  net_layer_refdec(old_target);
}

void net_task_timeout_push(struct net_task_s *task,
                           struct net_layer_s *target,
                           dev_timer_value_t deadline,
                           dev_timer_value_t precision)
{
  task->timeout.deadline = deadline;
  task->timeout.precision = precision;

  net_task_push(task, target, target, NET_TASK_TIMEOUT);
}

void net_task_notification_push(struct net_task_s *task,
                                struct net_layer_s *target,
                                struct net_layer_s *source,
                                uint32_t opcode)
{
  task->notification.opcode = opcode;

  net_task_push(task, target, source, NET_TASK_NOTIFICATION);
}

void net_task_query_push(struct net_task_s *task,
                         struct net_layer_s *target,
                         struct net_layer_s *source,
                         uint32_t opcode)
{
  task->query.opcode = opcode;

  net_task_push(task, target, source, NET_TASK_QUERY);
}

void net_task_query_respond_push(struct net_task_s *task,
                                 error_t err)
{
  struct net_layer_s *old_source = task->source;
  struct net_layer_s *old_target = task->target;

  assert(task->type == NET_TASK_QUERY);

  task->query.err = err;

  net_task_push(task, old_source, old_target, NET_TASK_RESPONSE);

  net_layer_refdec(old_source);
  net_layer_refdec(old_target);
}

void net_task_packet_respond(struct net_task_s *task,
                             struct net_layer_s *next_hop,
                             dev_timer_value_t timestamp,
                             const struct net_addr_s dst[static 1])
{
  struct net_layer_s *old_source = task->source;
  struct net_layer_s *old_target = task->target;

  assert(task->type == NET_TASK_INBOUND);

  task->packet.timestamp = timestamp;
  task->packet.src_addr = task->packet.dst_addr;
  task->packet.dst_addr = *dst;

  net_task_push(task, next_hop, old_target, NET_TASK_OUTBOUND);

  net_layer_refdec(old_source);
  net_layer_refdec(old_target);
}

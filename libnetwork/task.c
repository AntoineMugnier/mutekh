#include <mutek/printk.h>

#include <net/task.h>
#include <net/layer.h>
#include <net/scheduler.h>

void net_task_cleanup(struct net_task_s *task)
{
  struct net_layer_s *source = task->header.source;
  struct net_layer_s *target = task->header.target;

  switch (task->header.type) {
  case NET_TASK_INBOUND:
    buffer_refdec(task->inbound.buffer);
    break;

  default:
    break;
  }

  if (task->header.destroy_func)
    task->header.destroy_func(&task->header);
  net_layer_refdec(source);
  net_layer_refdec(target);
}

void net_task_push(struct net_task_header_s *header,
                   struct net_layer_s *target,
                   struct net_layer_s *source,
                   enum net_task_type_e type)
{
  header->target = net_layer_refinc(target);
  header->source = net_layer_refinc(source);
  header->type = type;

  //printk("Task %p push %S\n", header, &header->target->type, 4);

  net_scheduler_task_push(target->scheduler, header);
}

void net_task_inbound_push(struct net_task_s *task,
                           struct net_layer_s *target,
                           struct net_layer_s *source,
                           dev_timer_value_t timestamp,
                           const struct net_addr_s *src_addr,
                           const struct net_addr_s *dst_addr,
                           struct buffer_s *buffer)
{
  task->inbound.timestamp = timestamp;
  task->inbound.buffer = buffer_refinc(buffer);
  if (src_addr)
    task->inbound.src_addr = *src_addr;
  if (dst_addr)
    task->inbound.dst_addr = *dst_addr;

  //printk("Task %p forward <- %S\n", task, &source->type, 4);

  net_task_push(&task->header, target, source, NET_TASK_INBOUND);
}

void net_task_inbound_forward(struct net_task_s *task,
                              struct net_layer_s *target)
{
  struct net_layer_s *old_source = task->header.source;
  struct net_layer_s *old_target = task->header.target;

  //printk("Task %p forward <- %S\n", task, &task->inbound.source->type, 4);

  net_task_push(&task->header, target, old_target, NET_TASK_INBOUND);

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

  net_task_push(&task->header, target, target, NET_TASK_TIMEOUT);
}

void net_task_notification_push(struct net_task_s *task,
                                struct net_layer_s *target,
                                struct net_layer_s *source,
                                uint32_t opcode)
{
  task->notification.opcode = opcode;

  net_task_push(&task->header, target, source, NET_TASK_NOTIFICATION);
}

void net_task_query_push(struct net_task_s *task,
                         struct net_layer_s *target,
                         struct net_layer_s *source,
                         uint32_t opcode)
{
  task->query.opcode = opcode;

  net_task_push(&task->header, target, source, NET_TASK_QUERY);
}

void net_task_query_respond_push(struct net_task_s *task,
                                 error_t err)
{
  struct net_layer_s *old_source = task->header.source;
  struct net_layer_s *old_target = task->header.target;

  assert(task->header.type == NET_TASK_QUERY);

  task->response.err = err;

  net_task_push(&task->header, old_source, old_target, NET_TASK_RESPONSE);

  net_layer_refdec(old_source);
  net_layer_refdec(old_target);
}

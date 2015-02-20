#include <mutek/printk.h>
#include <mutek/buffer_pool.h>

#include <net/layer.h>
#include <net/task.h>
#include <net/scheduler.h>

#include <net/console_sink.h>

static
void net_console_sink_destroyed(struct net_layer_s *layer)
{
}

static
void net_console_sink_task_handle(struct net_layer_s *layer,
                                  struct net_task_header_s *header)
{
  struct net_console_sink_s *sink = net_console_sink_s_from_layer(layer);
  struct net_task_s *task = net_task_s_from_header(header);

  printk("Sink event %d", header->type);

  switch (header->type) {
  case NET_TASK_INBOUND:
    printk(" pkt @%lld %P", task->inbound.timestamp,
           task->inbound.buffer->data,
           task->inbound.buffer->end - task->inbound.buffer->begin);
    break;

  default:
    break;
  }

  printk("\n");

  net_task_cleanup(task);
}

static const struct net_layer_handler_s sink_handler = {
  .destroyed = net_console_sink_destroyed,
  .task_handle = net_console_sink_task_handle,
};

error_t net_console_sink_init(
  struct net_console_sink_s *sink,
  struct net_scheduler_s *scheduler)
{
  return net_layer_init(&sink->layer, &sink_handler, scheduler, NET_CONSOLE_SINK, 0);
}

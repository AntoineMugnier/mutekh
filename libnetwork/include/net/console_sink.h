#ifndef NET_CONSOLE_SINK_H_
#define NET_CONSOLE_SINK_H_

#include <hexo/types.h>
#include <hexo/decls.h>
#include <net/scheduler.h>

#define NET_CONSOLE_SINK NET_LAYER_TYPE('S', 'i', 'n', 'k')

struct net_console_sink_s
{
  struct net_layer_s layer;
};

STRUCT_COMPOSE(net_console_sink_s, layer);

error_t net_console_sink_init(
  struct net_console_sink_s *sink,
  struct net_scheduler_s *sched);

void net_console_sink_cleanup(
  struct net_console_sink_s *sink);

#endif

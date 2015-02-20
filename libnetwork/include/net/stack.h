#ifndef NET_STACK_H
#define NET_STACK_H

#include <hexo/types.h>

struct net_scheduler_s;

struct net_stack_config_s
{
  const char *timer_dev_path;
  size_t stack_size;
};

struct net_stack_s
{
  const struct net_stack_config_s *config;
  struct net_scheduler_s *scheduler;
};

error_t net_stack_create(
  const struct net_stack_config_s *config,
  struct net_stack_s **sched);

#endif

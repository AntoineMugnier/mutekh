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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2016
*/

#include <mutek/printk.h>
#include <mutek/buffer_pool.h>

#include <net/layer.h>
#include <net/task.h>
#include <net/scheduler.h>

#include <inet/layer/icmp.h>
#include <inet/protocol/icmp.h>
#include <inet/protocol/checksum.h>

#include <string.h>

struct inet_icmp_s
{
  struct net_layer_s layer;
};

STRUCT_COMPOSE(inet_icmp_s, layer);

static
void inet_icmp_destroyed(struct net_layer_s *layer)
{
  struct inet_icmp_s *icmp = inet_icmp_s_from_layer(layer);

  mem_free(icmp);
}

static
void inet_icmp_inbound_handle(struct inet_icmp_s *icmp, struct net_task_s *task)
{
  struct buffer_s *buf = task->packet.buffer;
  struct inet_icmp_hdr_s *p = (void *)(buf->data + buf->begin);

  // Incomplete packet
  if (buf->end - buf->begin < sizeof(*p)) {
    printk("Malformed ICMP packet: too short\n");
    goto drop;
  }

  printk("ICMP type %d\n", p->type);

  switch (p->type) {
  case INET_ICMP_ECHO: {
    struct net_addr_s src = task->packet.src_addr;
    p->type = INET_ICMP_ECHO_REPLY;
    p->checksum = 0;
    p->checksum = ~inet_checksum(p, buf->end - buf->begin);
    net_task_packet_respond(task, icmp->layer.parent, 0, &src);
    return;
  }

  default:
    break;
  };

 drop:  
  net_task_destroy(task);
}

static
void inet_icmp_task_handle(struct net_layer_s *layer,
                            struct net_task_s *task)
{
  struct inet_icmp_s *icmp = inet_icmp_s_from_layer(layer);

  switch (task->type) {
  default:
    break;

  case NET_TASK_INBOUND:
    inet_icmp_inbound_handle(icmp, task);
    return;
  }

  net_task_destroy(task);
}

static
void inet_icmp_dangling(struct net_layer_s *layer)
{
  struct inet_icmp_s *icmp = inet_icmp_s_from_layer(layer);

  (void)icmp;
}

static const struct net_layer_handler_s icmp_handler = {
  .destroyed = inet_icmp_destroyed,
  .task_handle = inet_icmp_task_handle,
  .dangling = inet_icmp_dangling,
};

error_t inet_icmp_create(struct net_scheduler_s *scheduler,
                          void *delegate,
                          const struct net_layer_delegate_vtable_s *delegate_vtable,
                          struct net_layer_s **layer)
{
 struct inet_icmp_s *icmp = mem_alloc(sizeof(*icmp), mem_scope_sys);

  if (!icmp)
    return -ENOMEM;

  memset(icmp, 0, sizeof(*icmp));

  error_t err = net_layer_init(&icmp->layer, &icmp_handler, scheduler,
                               delegate, delegate_vtable);
  if (err)
    mem_free(icmp);
  else
    *layer = &icmp->layer;

  return err;
}


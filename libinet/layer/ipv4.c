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

#include <inet/layer/ipv4.h>
#include <inet/protocol/ip.h>
#include <inet/protocol/ipv4.h>

#include <string.h>

//#define dprintk printk
#ifndef dprintk
# define dprintk(...) do{}while(0)
#endif

struct inet_ipv4_s
{
  struct net_layer_s layer;
  struct net_layer_s *icmp;
};

STRUCT_COMPOSE(inet_ipv4_s, layer);

static
void inet_ipv4_destroyed(struct net_layer_s *layer)
{
  struct inet_ipv4_s *ipv4 = inet_ipv4_s_from_layer(layer);

  mem_free(ipv4);
}

static
void inet_ipv4_outbound_handle(struct inet_ipv4_s *ipv4,
                               struct net_task_s *task,
                               uint8_t proto)
{
  struct buffer_s *buf = task->packet.buffer;
  struct inet_ipv4_hdr_s *p;

  if (!ipv4->layer.parent)
    goto drop;

  assert(buf->begin >= sizeof(*p));
  buf->begin -= sizeof(*p);
  p = (void *)(buf->data + buf->begin);

  inet_ipv4_hdr_init(p, 0);
  memcpy(p->src_addr, task->packet.src_addr.ipv4, 4);
  memcpy(p->dst_addr, task->packet.dst_addr.ipv4, 4);
  p->protocol = proto;
  p->ttl = 64;
  endian_be16_na_store(&p->total_length, buf->end - buf->begin);
  endian_16_na_store(&p->checksum, ~inet_ipv4_hdr_checksum(p));

  task->packet.dst_addr.ethertype = 0x0800;

  net_task_packet_forward(task, ipv4->layer.parent);
  return;

 drop:  
  net_task_destroy(task);
}

static
void inet_ipv4_inbound_handle(struct inet_ipv4_s *ipv4, struct net_task_s *task)
{
  struct buffer_s *buf = task->packet.buffer;
  const struct inet_ipv4_hdr_s *p = (const void *)(buf->data + buf->begin);
  size_t header_size;
  uint8_t proto;

  header_size = inet_ipv4_hdr_size_get(p);

  // Malformed packet
  if (inet_ipv4_hdr_version_get(p) != 4) {
    dprintk("Malformed packet: bad IP version\n");
    goto drop;
  }

  // Malformed packet
  if (header_size < INET_IPV4_HDR_SIZE_MIN) {
    dprintk("Malformed packet: header too short\n");
    goto drop;
  }

  // Bad header checksum
  if (inet_ipv4_hdr_checksum(p) != 0xffff) {
    dprintk("Malformed packet: bad header checksum\n");
    goto drop;
  }

  // Incomplete packet (fragmented or bad)
  if (buf->end - buf->begin != endian_be16_na_load(&p->total_length)) {
    dprintk("Malformed packet: bad total length\n");
    goto drop;
  }

  memcpy(task->packet.src_addr.ipv4, p->src_addr, 4);
  memcpy(task->packet.dst_addr.ipv4, p->dst_addr, 4);
  proto = p->protocol;
  buf->begin += inet_ipv4_hdr_size_get(p);

  dprintk("Packet from "INET_IPV4_FMT" to "INET_IPV4_FMT" proto %d: %P\n",
         INET_IPV4_ARG(task->packet.src_addr.ipv4),
         INET_IPV4_ARG(task->packet.dst_addr.ipv4),
         proto, buf->data + buf->begin, buf->end - buf->begin);

  switch (proto) {
  case INET_IP_PROTO_ICMP:
    if (!ipv4->icmp)
      break;
    net_task_packet_forward(task, ipv4->icmp);
    return;

  default:
    break;
  }

 drop:  
  net_task_destroy(task);
}

static
void inet_ipv4_task_handle(struct net_layer_s *layer,
                            struct net_task_s *task)
{
  struct inet_ipv4_s *ipv4 = inet_ipv4_s_from_layer(layer);

  switch (task->type) {
  default:
    break;

  case NET_TASK_INBOUND:
    inet_ipv4_inbound_handle(ipv4, task);
    return;

  case NET_TASK_OUTBOUND:
    if (task->source == ipv4->icmp)
      inet_ipv4_outbound_handle(ipv4, task, INET_IP_PROTO_ICMP);
    else
      break;
    return;
  }

  net_task_destroy(task);
}

static
void inet_ipv4_dangling(struct net_layer_s *layer)
{
  struct inet_ipv4_s *ipv4 = inet_ipv4_s_from_layer(layer);

  (void)ipv4;
}

static
error_t inet_ipv4_bound(struct net_layer_s *layer,
                        void *addr,
                        struct net_layer_s *child)
{
  struct inet_ipv4_s *ipv4 = inet_ipv4_s_from_layer(layer);

  switch (*(const uint8_t *)addr) {
  case INET_IP_PROTO_ICMP:
    if (ipv4->icmp)
      return -EBUSY;
    ipv4->icmp = child;
    return 0;

  default:
    return -ENOTSUP;
  }
}

static
void inet_ipv4_unbound(struct net_layer_s *layer,
                             struct net_layer_s *child)
{
  struct inet_ipv4_s *ipv4 = inet_ipv4_s_from_layer(layer);

  if (child == ipv4->icmp)
    ipv4->icmp = NULL;
}

static const struct net_layer_handler_s ipv4_handler = {
  .destroyed = inet_ipv4_destroyed,
  .task_handle = inet_ipv4_task_handle,
  .dangling = inet_ipv4_dangling,
  .bound = inet_ipv4_bound,
  .unbound = inet_ipv4_unbound,
};

error_t inet_ipv4_create(struct net_scheduler_s *scheduler,
                          void *delegate,
                          const struct net_layer_delegate_vtable_s *delegate_vtable,
                          struct net_layer_s **layer)
{
 struct inet_ipv4_s *ipv4 = mem_alloc(sizeof(*ipv4), mem_scope_sys);

  if (!ipv4)
    return -ENOMEM;

  memset(ipv4, 0, sizeof(*ipv4));

  error_t err = net_layer_init(&ipv4->layer, &ipv4_handler, scheduler,
                               delegate, delegate_vtable);
  if (err)
    mem_free(ipv4);
  else
    *layer = &ipv4->layer;

  return err;
}


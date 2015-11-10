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

#include <hexo/types.h>
#include <hexo/endian.h>
#include <string.h>

#include <mutek/mem_alloc.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/crypto.h>
#include <device/class/ble_radio.h>
#include <device/class/net.h>

#include <mutek/printk.h>

#include <net/layer.h>

static const struct net_layer_handler_s pair_handler;

struct pair_layer_s
{
  struct net_layer_s layer;
  struct net_layer_s *sibling;
  struct net_layer_s *child;
  struct device_s *dev;
};

STRUCT_COMPOSE(pair_layer_s, layer);

static
void pair_task_handle(struct net_layer_s *layer,
                      struct net_task_s *task)
{
  struct pair_layer_s *pair = pair_layer_s_from_layer(layer);
  struct net_task_s *rtask;

  switch (task->type) {
  case NET_TASK_OUTBOUND:
    if (!pair->sibling || !pair->sibling->child)
      break;

    rtask = net_scheduler_task_alloc(pair->layer.scheduler);
    if (rtask)
      net_task_inbound_push(rtask, pair->layer.sibling->child, &pair->layer,
                            net_scheduler_time_get(pair->layer.scheduler),
                            &task->packet.dst_addr, &task->packet.src_addr,
                            task->packet.buffer);
    break;

  default:
    break;
  }

  net_task_destroy(task);
}

static
error_t pair_bound(struct net_layer_s *layer,
                       void *addr,
                       struct net_layer_s *child)
{
  struct pair_layer_s *pair = pair_layer_s_from_layer(layer);

  if (!pair->sibling)
    return -EIO;

  if (pair->child)
    return -EBUSY;

  pair->child = child;

  return 0;
}

static
void pair_unbound(struct net_layer_s *layer,
                  struct net_layer_s *child)
{
  assert(layer->child == child);
  layer->child = NULL;
}

static
void pair_destroyed(struct net_layer_s *layer)
{
  struct pair_layer_s *pair = pair_layer_s_from_layer(layer);

  // .1 takes a reference on .0, so the first to be destroyed is .1.
  if (pair->sibling) {
    assert(pair->sibling->sibling == &pair->layer);
    assert(pair->sibling == &(pair - 1)->layer);
    pair->sibling->sibling = NULL;
    net_layer_refdec(&pair->sibling.layer);
    pair->sibling = NULL;
  } else {
    if (pair->dev) {
      // Pair got destroyed before user retrieved .1
      assert(pair->dev->drv_pv == pair);
      pair->dev->drv_pv = NULL;
    }
    
    mem_free(pair);
  }
}

static const struct net_layer_handler_s pair_handler = {
  .destroyed = pair_destroyed,
  .task_handle = pair_task_handle,
  .bound = pair_bound,
  .unbound = pair_unbound,
  .type = NET_PAIR,
};

static DEV_NET_LAYER_CREATE(pair_layer_create)
{
  struct device_s *dev = accessor->dev;
  struct pair_layer_s *l;
  error_t err;

  switch (type) {
  default:
    return -ENOTSUP;

  case NET_PAIR_0:
    if (dev->drv_pv)
      return -EBUSY;

    l = mem_alloc(sizeof(*l) * 2, mem_scope_sys);
    if (!l)
      return -ENOMEM;

    memset(l, 0, sizeof(*l) * 2);

    err = net_layer_init(l, &pair_handler, scheduler, delegate, delegate_vtable);
    if (err) {
      mem_free(l);
      return err;
    }

    *layer = l;
    dev->drv_pv = l;
    l->dev = dev;

    return 0;

  case NET_PAIR_1:
    if (!dev->drv_pv)
      return -EAGAIN;

    l = dev->drv_pv;
    dev->drv_pv = NULL;
    assert(!l->sibling);

    l->dev = NULL;

    err = net_layer_init(l, &pair_handler, scheduler, delegate, delegate_vtable);
    if (err) {
      mem_free(l);
      return err;
    }

    l[0].sibling = &l[1].layer;
    l[1].sibling = net_layer_refinc(&l[0].layer);

    *layer = l;

    return 0;

  default:
    return -ENOTSUP;
  }
}

static DEV_NET_GET_INFO(pair_get_info)
{
  struct device_s *dev = accessor->dev;

  memset(info, 0, sizeof(*info));
  info->implemented_layers = 0
    | (1 << NET_PAIR)
    ;
  info->prefix_size = 0;
  info->mtu = __MAXOF_TYPE(uint16_t);

  return 0;
}

static DEV_CLEANUP(pair_cleanup);
static DEV_INIT(pair_init);
#define pair_use dev_use_generic

DRIVER_DECLARE(pair_drv, 0, "Network device pair", pair,
               DRIVER_NET_METHODS(pair));

DRIVER_REGISTER(pair_drv);

static DEV_CLEANUP(pair_cleanup)
{
  return 0;
}

static DEV_INIT(pair_init)
{
  dev->drv_pv = NULL;
  dev->drv = &pair_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;
}

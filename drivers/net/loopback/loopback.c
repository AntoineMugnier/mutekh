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

static const struct net_layer_handler_s loopback_handler;

static
void loopback_task_handle(struct net_layer_s *layer,
                          struct net_task_s *task)
{
  struct net_task_s *rtask;
  struct net_layer_s *target;

  switch (task->type) {
  case NET_TASK_OUTBOUND:
    target = net_layer_list_head(&layer->children);
 
    if (!target)
      break;

    rtask = net_scheduler_task_alloc(layer->scheduler);

    if (rtask)
      net_task_inbound_push(rtask, target, layer,
                            net_scheduler_time_get(layer->scheduler),
                            &task->packet.dst_addr, &task->packet.src_addr,
                            task->packet.buffer);

    net_layer_refdec(target);
    break;

  default:
    break;
  }

  net_task_destroy(task);
}

static
error_t loopback_bound(struct net_layer_s *layer,
                       void *addr,
                       struct net_layer_s *child)
{
  if (net_layer_list_count(&layer->children))
    return -EBUSY;

  return 0;
}

static
void loopback_destroyed(struct net_layer_s *layer)
{
  mem_free(layer);
}

static const struct net_layer_handler_s loopback_handler = {
  .destroyed = loopback_destroyed,
  .task_handle = loopback_task_handle,
  .bound = loopback_bound,
  .type = NET_LOOPBACK,
};

static DEV_NET_LAYER_CREATE(loopback_layer_create)
{
  struct device_s *dev = accessor->dev;
  struct net_layer_s *l;
  error_t err;

  if (type != NET_LOOPBACK)
    return -ENOTSUP;

  l = mem_alloc(sizeof(*l), mem_scope_sys);
  if (!l)
    return -ENOMEM;

  memset(l, 0, sizeof(*l));

  err = net_layer_init(l, &loopback_handler, scheduler, delegate, delegate_vtable);
  if (err)
    mem_free(l);
  else
    *layer = l;

  return err;
}

static DEV_NET_GET_INFO(loopback_get_info)
{
  struct device_s *dev = accessor->dev;

  memset(info, 0, sizeof(*info));
  info->implemented_layers = 0
    | (1 << NET_LOOPBACK)
    ;
  info->prefix_size = 0;
  info->mtu = __MAXOF_TYPE(uint16_t);

  return 0;
}

static DEV_CLEANUP(loopback_cleanup);
static DEV_INIT(loopback_init);
#define loopback_use dev_use_generic

DRIVER_DECLARE(loopback_drv, 0, "Network loopback", loopback,
               DRIVER_NET_METHODS(loopback));

DRIVER_REGISTER(loopback_drv);

static DEV_CLEANUP(loopback_cleanup)
{
  return 0;
}

static DEV_INIT(loopback_init)
{
  dev->drv_pv = NULL;
  dev->drv = &loopback_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;
}

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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2016
*/

#include <string.h>
#include <stdio.h>

#include <hexo/types.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>
#include <hexo/bit.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>

#include <device/class/net.h>
#include <device/class/mem.h>
#include <device/class/iomux.h>
#include <device/class/gpio.h>
#include <device/class/timer.h>
#include <device/irq.h>
#include <device/resources.h>

#include <mutek/buffer_pool.h>
#include <net/scheduler.h>
#include <net/layer.h>
#include <net/layer/ethernet.h>
#include <net/task.h>

#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>
#include <mutek/printk.h>

#include <drivers/phy/smi.h>
#include "dwc10100_mac.h"

//#define dprintk printk
#ifndef dprintk
# define dprintk(...) do{}while(0)
#endif

enum dwc_irq_id_e
{
  DWC_IRQ_MAC,
  DWC_IRQ_PHY,
  DWC_IRQ_COUNT,
};

struct dwc_dma_desc_s
{
  uint32_t desc;
  uint32_t size;
  uint32_t buf0;
  uint32_t buf1;
};

struct dwc_private_s
{
  struct net_layer_s layer;
  struct net_layer_s *target;
  struct device_gpio_s gpio;
  uintptr_t mac;
  uintptr_t mmc;
  uintptr_t dma;

  bool_t full_duplex;

  uint16_t status_old;
  uint16_t status_cur;

  struct kroutine_s phy_updater;
  struct kroutine_s mac_configure;

  struct dwc_dma_desc_s tx_desc;
  struct dwc_dma_desc_s rx_desc;

  struct dev_irq_src_s irq_ep[DWC_IRQ_COUNT];
};

STRUCT_COMPOSE(dwc_private_s, layer);

DRIVER_PV(struct dwc_private_s);

static
void dwc_outbound_enqueue(struct dwc_private_s *pv,
                              struct net_task_s *task)
{
}

static
void dwc_layer_destroyed(struct net_layer_s *layer)
{
  struct dwc_private_s *pv = dwc_private_s_from_layer(layer);

  memset(&pv->layer, 0, sizeof(pv->layer));
}

static
void dwc_layer_task_handle(struct net_layer_s *layer,
                      struct net_task_s *task)
{
  struct dwc_private_s *pv = dwc_private_s_from_layer(layer);

  switch (task->type) {
  case NET_TASK_OUTBOUND:
    dwc_outbound_enqueue(pv, task);
    return;

  default:
    break;
  }

  net_task_destroy(task);
}

static
void dwc_layer_child_context_adjust(const struct net_layer_s *layer,
                                    struct net_layer_context_s *ctx)
{
  const struct dwc_private_s *pv = const_dwc_private_s_from_layer(layer);

  (void)pv;
}

static
void dwc_layer_context_changed(struct net_layer_s *layer)
{
  struct dwc_private_s *pv = dwc_private_s_from_layer(layer);

  (void)pv;
}

static
void dwc_layer_dandling(struct net_layer_s *layer)
{
  struct dwc_private_s *pv = dwc_private_s_from_layer(layer);

  (void)pv;
  /* if (pv->rx_packet) { */
  /*   buffer_refdec(pv->rx_packet); */
  /*   pv->rx_packet = NULL; */
  /* } */
}

static
error_t dwc_layer_bound(struct net_layer_s *layer,
                        void *addr,
                        struct net_layer_s *child)
{
  struct dwc_private_s *pv = dwc_private_s_from_layer(layer);

  if (pv->target)
    return -EBUSY;

  pv->target = child;

  return 0;
}

static
void dwc_layer_unbound(struct net_layer_s *layer,
                       struct net_layer_s *child)
{
  struct dwc_private_s *pv = dwc_private_s_from_layer(layer);

  assert(child == pv->target);

  pv->target = NULL;
}

static const struct net_layer_handler_s dwc_layer_handler =
{
  .destroyed = dwc_layer_destroyed,
  .task_handle = dwc_layer_task_handle,
  .child_context_adjust = dwc_layer_child_context_adjust,
  .context_changed = dwc_layer_context_changed,
  .dandling = dwc_layer_dandling,
  .bound = dwc_layer_bound,
  .unbound = dwc_layer_unbound,
};

#define dwc_use dev_use_generic

static DEV_NET_LAYER_CREATE(dwc_net_layer_create)
{
  struct device_s *dev = accessor->dev;
  struct dwc_private_s *pv = dev->drv_pv;
  error_t err;

  if (type != NET_LAYER_ETHERNET)
    return -ENOTSUP;

  if (pv->layer.scheduler)
    return -EBUSY;

  err = net_layer_init(&pv->layer,
                       &dwc_layer_handler, scheduler,
                       delegate, delegate_vtable);

  if (!err)
    *layer = &pv->layer;

  return err;
}

static DEV_NET_GET_INFO(dwc_net_get_info)
{
  info->implemented_layers = bit(NET_LAYER_ETHERNET);
  memset(info->addr.mac, 0, 6);
  info->prefix_size = 0;
  info->mtu = 0;

  return 0;
}

static uint16_t dwc_smi_read(struct device_s *dev,
                              uint_fast8_t phy,
                              uint_fast8_t reg)
{
  struct dwc_private_s *pv = dev->drv_pv;
  uint16_t val;

  while (cpu_mem_read_32(pv->mac + DWC_MAC_MIIAR_ADDR) & DWC_MAC_MIIAR_MB)
    ;

  cpu_mem_write_32(pv->mac + DWC_MAC_MIIAR_ADDR, 0
                   | DWC_MAC_MIIAR_PA(phy)
                   | DWC_MAC_MIIAR_MR(reg)
                   | DWC_MAC_MIIAR_MB);

  while (cpu_mem_read_32(pv->mac + DWC_MAC_MIIAR_ADDR) & DWC_MAC_MIIAR_MB)
    ;

  val = cpu_mem_read_32(pv->mac + DWC_MAC_MIIDR_ADDR);

  return val;
}

static void dwc_smi_write(struct device_s *dev,
                           uint_fast8_t phy,
                           uint_fast8_t reg,
                           uint16_t value)
{
  struct dwc_private_s *pv = dev->drv_pv;

  while (cpu_mem_read_32(pv->mac + DWC_MAC_MIIAR_ADDR) & DWC_MAC_MIIAR_MB)
    ;

  cpu_mem_write_32(pv->mac + DWC_MAC_MIIDR_ADDR, value);
  cpu_mem_write_32(pv->mac + DWC_MAC_MIIAR_ADDR, 0
                   | DWC_MAC_MIIAR_PA(phy)
                   | DWC_MAC_MIIAR_MR(reg)
                   | DWC_MAC_MIIAR_MW
                   | DWC_MAC_MIIAR_MB);
}

static DEV_IRQ_SRC_PROCESS(dwc_irq)
{
  struct device_s *dev = ep->base.dev;
  struct dwc_private_s *pv = dev->drv_pv;
  enum dwc_irq_id_e no = ep - pv->irq_ep;

  printk("%s IRQ %d\n", __FUNCTION__, no);

  if (no == DWC_IRQ_PHY) {
    LOCK_SPIN_IRQ(&dev->lock);
    pv->status_cur = dwc_smi_read(dev, 0, SMI_STATUS_ADDR);
    dwc_smi_read(dev, 0, 29);
    dwc_smi_write(dev, 0, 29, -1);
    LOCK_RELEASE_IRQ(&dev->lock);
    kroutine_exec(&pv->phy_updater);
  }
}

static KROUTINE_EXEC(dwc_phy_updater)
{
  struct dwc_private_s *pv = KROUTINE_CONTAINER(kr, *pv, phy_updater);
  struct device_s *dev = pv->irq_ep[0].base.dev;

  uint_fast16_t cleared = ~pv->status_cur & pv->status_old;
  uint_fast16_t set = pv->status_cur & ~pv->status_old;
  pv->status_old = pv->status_cur;

  if (cleared & SMI_STATUS_LINK_UP)
    printk("%s: Link down\n", __FUNCTION__);

  if (set & SMI_STATUS_LINK_UP)
    printk("%s: Link up\n", __FUNCTION__);

  if (set & SMI_STATUS_AUTONEG_DONE)
    printk("%s: Autoneg complete\n", __FUNCTION__);

  if (set & SMI_STATUS_JABBER_DETECT)
    printk("%s: Jabber detected\n", __FUNCTION__);

  uint_fast16_t local = dwc_smi_read(dev, 0, SMI_AUTONEG_ADV_ADDR);
  uint_fast16_t remote = dwc_smi_read(dev, 0, SMI_AUTONEG_PARTNER_ADDR);
  bool_t fd = !!(local & remote & (SMI_AUTONEG_ADV_CAN_10BASETFD
                                   | SMI_AUTONEG_ADV_CAN_100BASETXFD));
  if (fd != !pv->full_duplex) {
    pv->full_duplex = fd;
    kroutine_exec(&pv->mac_configure);
  }
}

static KROUTINE_EXEC(dwc_mac_configure)
{
  struct dwc_private_s *pv = KROUTINE_CONTAINER(kr, *pv, mac_configure);
  struct device_s *dev = pv->irq_ep[0].base.dev;

  cpu_mem_write_32(pv->mac + DWC_MAC_CR_ADDR, 0
                   | DWC_MAC_CR_WD
                   | DWC_MAC_CR_FES
                   | (pv->full_duplex ? DWC_MAC_CR_DM : 0)
                   | DWC_MAC_CR_TE
                   | DWC_MAC_CR_RE
                   );

  cpu_mem_write_32(pv->mac + DWC_MAC_FFR_ADDR, 0
                   | DWC_MAC_FFR_PAM
                   );
}

#if defined(CONFIG_DRIVER_NET_DWC10100_SMI_MEM)
static DEV_MEM_INFO(dwc_mem_info)
{
  if (band_index > 0)
    return -ENOENT;

  if (accessor->number > 31)
    return -ENOENT;

  memset(info, 0, sizeof(*info));

  info->type = DEV_MEM_REG;
  info->size = 64;
  info->flags = 0
    | DEV_MEM_WRITABLE
    | DEV_MEM_VOLATILE;

  return 0;
}

static DEV_MEM_REQUEST(dwc_mem_request)
{
  struct device_s *dev = accessor->dev;
  error_t err = 0;

  if (accessor->number > 31) {
    err = -ENOENT;
    goto out;
  }

  if (rq->type & (DEV_MEM_OP_CACHE_INVALIDATE
                  | DEV_MEM_OP_PAGE_ERASE
                  | DEV_MEM_OP_PAGE_READ
                  | DEV_MEM_OP_PAGE_WRITE
                  | DEV_MEM_OP_CACHE_FLUSH
                  | DEV_MEM_OP_CONFIG)) {
    err = -ENOTSUP;
    goto out;
  }

  if (rq->size & 1) {
    err = -ENOTSUP;
    goto out;
  }

  LOCK_SPIN_IRQ(&dev->lock);
  if (rq->type & DEV_MEM_OP_PARTIAL_WRITE) {
    const uint16_t *data = (const void*)rq->data;

    for (size_t i = 0; i < rq->size; i += 2)
      dwc_smi_write(dev, accessor->number, (rq->addr + i) / 2, data[i/2]);
  } else {
    uint16_t *data = (void*)rq->data;

    for (size_t i = 0; i < rq->size; i += 2)
      data[i/2] = dwc_smi_read(dev, accessor->number, (rq->addr + i) / 2);

  }
  LOCK_RELEASE_IRQ(&dev->lock);

 out:
  rq->err = err;
  kroutine_exec(&rq->base.kr);
}
#endif

static DEV_INIT(dwc_init)
{
  error_t err = 0;
  struct dwc_private_s *pv;
  bool_t rmii = 0;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  err = device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->mac, NULL);
  if (err)
    goto err_mem;

  err = device_res_get_uint(dev, DEV_RES_MEM, 1, &pv->mmc, NULL);
  if (err)
    goto err_mem;

  err = device_res_get_uint(dev, DEV_RES_MEM, 2, &pv->dma, NULL);
  if (err)
    goto err_mem;

  err = device_iomux_setup(dev, ">mdc >mdio", NULL, NULL, NULL);
  if (err)
    goto err_mem;

  // First, try MII mode
  err = device_iomux_setup(dev,
                           "<tx_clk >tx_d0 >tx_d1 >tx_d2 >tx_d3 >tx_en"
                           "<rx_clk <rx_d0 <rx_d1 <rx_d2 <rx_d3 <rx_err <rx_dv"
                           "<crs <col",
                           NULL, NULL, NULL);
  if (err) {
    // Try RMII mode
    err = device_iomux_setup(dev,
                             ">tx_d0 >tx_d1 >tx_en"
                             "<rx_d0 <rx_d1 <crs_dv"
                             "<ref_clk",
                             NULL, NULL, NULL);
    if (err)
      goto err_mem;

    rmii = 1;
  }

  err = device_get_param_dev_accessor(dev, "gpio", &pv->gpio.base, DRIVER_CLASS_GPIO);
  if (err)
    goto err_mem;

  gpio_id_t map[1];
  gpio_width_t w[1] = {1};
  err = device_res_gpio_map(dev, "resetn:1", map, w);
  if (err)
    goto err_timer;

  err = device_gpio_map_set_mode(&pv->gpio, map, w, 1, DEV_PIN_PUSHPULL);
  if (err)
    goto err_timer;

  err = dev_gpio_out(&pv->gpio, map[0], 1);
  if (err)
    goto err_timer;

  device_irq_source_init(dev, pv->irq_ep, DWC_IRQ_COUNT, &dwc_irq);
  if (device_irq_source_link(dev, pv->irq_ep, DWC_IRQ_COUNT, -1))
    goto err_timer;

  struct dev_resource_s *r = device_res_get(dev, DEV_RES_IRQ, 1);
  assert(r);

  dev_gpio_mode(&pv->gpio, r->u.irq.sink_id, DEV_PIN_INPUT_PULLUP);

  const uint8_t *mac;
  err = device_get_param_blob(dev, "hwaddr", 0, (const void**)&mac);
  if (err)
    mac = (const void*)"\x02\x00\x00\x12\x34\x56";

  cpu_mem_write_32(pv->mac + DWC_MAC_A0HR_ADDR,
                   endian_be16_na_load(mac));
  cpu_mem_write_32(pv->mac + DWC_MAC_A0LR_ADDR,
                   endian_be32_na_load(mac+2));

  dwc_smi_write(dev, 0, SMI_CONTROL_ADDR, 0
                | SMI_CONTROL_DUPLEX_MODE(FULL)
                | SMI_CONTROL_AUTONEG_RESTART
                | SMI_CONTROL_AUTONEG
                | SMI_CONTROL_SPEED_SELECT_2(10_100)
                | SMI_CONTROL_SPEED_SELECT_1(100_RES)
                );

  dwc_smi_write(dev, 0, 30, 0x50);
  dwc_smi_read(dev, 0, 29);
  dwc_smi_write(dev, 0, 29, -1);

  kroutine_init_deferred(&pv->phy_updater, dwc_phy_updater);
  kroutine_init_deferred(&pv->mac_configure, dwc_mac_configure);

  pv->tx_desc.desc = DWC_TX_DESC_TDES0_IC;
  pv->rx_desc.desc = 0;

  return 0;

 err_gpio:
  device_put_accessor(&pv->gpio.base);
 err_timer:
  //  device_put_accessor(&pv->timer.base);
 err_mem:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(dwc_cleanup)
{
  struct dwc_private_s *pv = dev->drv_pv;

  device_put_accessor(&pv->gpio.base);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(dwc10100_drv, 0, "DesignWare 10/100 Eth MAC", dwc,
               DRIVER_NET_METHODS(dwc_net),
#if defined(CONFIG_DRIVER_NET_DWC10100_SMI_MEM)
               DRIVER_MEM_METHODS(dwc_mem)
#endif
               );

DRIVER_REGISTER(dwc10100_drv);

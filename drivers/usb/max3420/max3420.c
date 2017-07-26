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
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright (c) 2016 Sebastien Cerdan <sebcerdan@gmail.com>

*/

#include "max3420_spi.h"

/*

  This driver support max packet size up to 64 bytes.

  Here is a sample max3420 device declaration:

  DEV_DECLARE_STATIC(max3420_dev, "max3420", 0, max3420_drv,
    DEV_STATIC_RES_DEV_PARAM("spi", "/spi*"),
    DEV_STATIC_RES_DEV_PARAM("gpio", "/gpio"),
    DEV_STATIC_RES_DEV_PARAM("icu", "/gpio"),
    DEV_STATIC_RES_IRQ(0, PIN1, DEV_IRQ_SENSE_FALLING_EDGE, 0, 1),

    DEV_STATIC_RES_GPIO("rst",  PIN0,  1),
    DEV_STATIC_RES_GPIO("nirq", PIN1,  1),
    DEV_STATIC_RES_GPIO("gpx",  PIN2,  1),

    DEV_STATIC_RES_UINT_PARAM("gpio-cs-id", PIN3),
  );

*/

static uint8_t max3420_usbdev_set_event(struct max3420_usbdev_private_s *pv)
{
  uint8_t ret = 0;

  if (pv->done & MAX3420_DIS_MASK)
    ret |=  USBDEV_EVENT_STOP;
  if (pv->event & MAX3420_IRQ_VBUS)
    ret |=  USBDEV_EVENT_CONNECT;
  if (pv->event & MAX3420_IRQ_NOVBUS)
    ret |=  USBDEV_EVENT_DISCONNECT;
  if (pv->event & MAX3420_IRQ_RSTDONE)
    ret |=  USBDEV_EVENT_RESET;

  pv->done &= ~MAX3420_DIS_MASK;

  pv->event = 0;
  return ret;
}

static void max3420_usbdev_process_ep0(struct max3420_usbdev_private_s *pv)
{
  struct dev_usbdev_request_s * tr = pv->tr[0];
  struct dev_spi_ctrl_bytecode_rq_s *srq = &pv->spi_rq;

  assert(tr);

  pv->pending |= MAX3420_EP0_MASK;

  void * entry;
  uint32_t arg = 0;

  switch (tr->type)
    {
    case DEV_USBDEV_DATA_IN:
    case DEV_USBDEV_PARTIAL_DATA_IN:
      entry = &max3420_entry_data_in_0;
      break;
    case DEV_USBDEV_DATA_OUT:
    case DEV_USBDEV_PARTIAL_DATA_OUT:
      assert((tr->size % pv->mps[0]) == 0);
      entry = &max3420_entry_data_out_0;
      break;
    case DEV_USBDEV_CTRL_SETUP:
      entry = &max3420_entry_setup;
      break;
    case DEV_USBDEV_CTRL_STATUS_OUT_STALL:
    case DEV_USBDEV_CTRL_STATUS_IN_STALL:
      entry = &max3420_entry_status;
      arg = MAX3420_STLSTAT;
      break;
    case DEV_USBDEV_DATA_IN_STALL:
      entry = &max3420_entry_status;
      arg = MAX3420_STLEP0IN;
      break;
    case DEV_USBDEV_DATA_OUT_STALL:
      entry = &max3420_entry_status;
      arg = MAX3420_STLEP0OUT;
      break;
    case DEV_USBDEV_CTRL_STATUS_OUT:
    case DEV_USBDEV_CTRL_STATUS_IN:
      entry = &max3420_entry_status;
      arg = MAX3420_ACKSTAT;
      break;
    case DEV_USBDEV_EVENT:
      pv->pending &= ~MAX3420_EP0_MASK;
      return;
    default:
      abort();
    }

  error_t ret;

  if (arg)
    ret = dev_spi_bytecode_start(&pv->spi, srq, entry, (1 << R_ARG0), arg);
  else
    ret = dev_spi_bytecode_start(&pv->spi, srq, entry, 0);

  if (ret == 0)
    pv->pending &= ~MAX3420_EP0_MASK;
}

static void max3420_usbdev_process_epn(struct max3420_usbdev_private_s *pv, uint8_t index)
{
  struct dev_usbdev_request_s * tr = pv->tr[index];

  assert(tr);

  switch (tr->type)
    {
    case DEV_USBDEV_DATA_OUT:
    case DEV_USBDEV_PARTIAL_DATA_OUT:
      assert((tr->size % pv->mps[index]) == 0);
    default:
      break;
    }

  pv->pending |= (1 << index);

  dev_spi_bytecode_start(&pv->spi, &pv->spi_rq, &max3420_entry_epn, 0);
}

static void max3420_process_config(struct max3420_usbdev_private_s *pv)
{
  ensure(pv->cfg);

  error_t err;

  switch (pv->cfg->type)
    {
    case DEV_USBDEV_SET_ADDRESS:
      err = dev_spi_bytecode_start(&pv->spi, &pv->spi_rq, &max3420_entry_set_address,
                                   (1 << R_ARG0), pv->cfg->addr);
      break;
    case DEV_USBDEV_UNCONFIGURE:
      err = dev_spi_bytecode_start(&pv->spi, &pv->spi_rq,
                                   &max3420_entry_unconfigure, 0);
      break;
    default:
      abort();
    }

  if (err == 0)
    pv->pending &= ~MAX3420_CFG_MASK;
}

static void max3420_usbdev_start_stop(struct device_s *dev, uint8_t msk)
{
  struct max3420_usbdev_private_s *pv = dev->drv_pv;
  void * entry;

  pv->pending |= msk;

  entry = (msk == MAX3420_CNT_MASK) ? &max3420_entry_connect : &max3420_entry_disconnect;

  if (dev_spi_bytecode_start(&pv->spi, &pv->spi_rq, entry, 0) == 0)
    pv->pending &= ~msk;
}

static void max3420_usbdev_process_pending(struct device_s *dev)
{
  struct max3420_usbdev_private_s *pv = dev->drv_pv;

  if (pv->pending & MAX3420_CNT_MASK)
    return max3420_usbdev_start_stop(dev, MAX3420_CNT_MASK);
  if (pv->pending & MAX3420_DIS_MASK)
    return max3420_usbdev_start_stop(dev, MAX3420_DIS_MASK);
  if (pv->pending & MAX3420_CFG_MASK)
    return max3420_process_config(pv);
  if (pv->pending & MAX3420_EP0_MASK)
    return max3420_usbdev_process_ep0(pv);
  for (uint8_t i = 1; i < MAX3420_EP_COUNT + 1; i++)
    {
      if (pv->pending & (1 << i))
       return max3420_usbdev_process_epn(pv, i);
    }
}

static DEV_USBDEV_REQUEST(max3420_usbdev_transfer)
{
  struct device_s *dev = ctx->dev;
  struct max3420_usbdev_private_s *pv = dev->drv_pv;

  error_t err = -EAGAIN;

  /* Some event are pending */

  bool_t event = pv->event || (pv->done & MAX3420_DIS_MASK);

  if (tr->ep == 0 && event)
    {
      err = -EIO;
      tr->event = max3420_usbdev_set_event(pv);
      goto end;
    }

  assert(pv->tr[tr->ep] == NULL);
  pv->tr[tr->ep] = tr;

  switch (tr->ep)
    {
    case 0:
      max3420_usbdev_process_ep0(pv);
      break;
    case 1 ... 3:
      max3420_usbdev_process_epn(pv, tr->ep);
      break;
    default:
      abort();
    }

end:
  return err;
}

static DEV_USBDEV_ENDPOINT(max3420_usbdev_endpoint)
{
  struct device_s *dev = ctx->dev;
  struct max3420_usbdev_private_s *pv = dev->drv_pv;
  struct usbdev_endpoint_s * ep = NULL;

  switch (address)
    {
    case 1:
      assert(dir == USB_EP_OUT);
      ep = &pv->ep[0];
      break;
    case 2 ... 3:
      assert(dir == USB_EP_IN);
      ep = &pv->ep[address - 1];
      break;
    default:
      break;
    }
  return ep;
}

static error_t max3420_usbdev_check_config(struct max3420_usbdev_private_s *pv,
                                           struct dev_usbdev_config_s *cfg)
{
  if (cfg->intf == NULL)
    {
      pv->mps[0] = usbdev_stack_get_ep0_mps(&pv->usbdev_ctx);

      if (pv->mps[0]> 64)
         return -ENOTSUP;

      return 0;
    }

  for (uint8_t i = 0; i< CONFIG_USBDEV_MAX_INTERFACE_COUNT; i++)
    {
      struct dev_usbdev_interface_cfg_s *icfg = cfg->intf + i;

      if (icfg->i == NULL)
        break;

      USBDEV_FOREACH_ENDPOINT(icfg->i, icfg->epi, icfg->epo,
       {
         if (usb_ep_type_get(epdesc) == USB_EP_CONTROL ||
             usb_ep_type_get(epdesc) == USB_EP_ISOCHRONOUS)
          return -ENOTSUP;

         uint16_t mps = usb_ep_mps_get(epdesc);

         if (mps > 64 || !is_pow2(mps))
            return -ENOTSUP;

         if (usb_ep_dir_get(epdesc) == USB_EP_IN)
         /* IN endpoint */
           {
            if ((epaddr != 2) && (epaddr != 3))
              return -ENOTSUP;
           }
         else if (epaddr != 1)
           return -ENOTSUP;

         pv->mps[epaddr] = mps;

       });
    }
  return 0;
}

#if (CONFIG_USBDEV_MAX_ALTERNATE_COUNT > 0)
static error_t max3420_usbdev_change_interface(struct max3420_usbdev_private_s *pv,
                                               struct dev_usbdev_config_s *cfg)
{
  struct dev_usbdev_interface_cfg_s *icfg = cfg->intf + DEV_USBDEV_INTF_ENABLE;

  USBDEV_FOREACH_ENDPOINT(icfg->i, icfg->epi, icfg->epo, {
      if (usb_ep_type_get(epdesc) == USB_EP_CONTROL ||
          usb_ep_type_get(epdesc) == USB_EP_ISOCHRONOUS)
        return -ENOTSUP;

      uint16_t mps = usb_ep_mps_get(epdesc);

      if (mps > 64 || !is_pow2(mps))
        return -ENOTSUP;

      if (usb_ep_dir_get(epdesc) == USB_EP_IN) {
        /* IN endpoint */
        if ((epaddr != 2) && (epaddr != 3))
          return -ENOTSUP;
      } else if (epaddr != 1)
        return -ENOTSUP;

      pv->mps[epaddr] = mps;
    });

  return 0;
}
#endif

static DEV_USBDEV_CONFIG(max3420_usbdev_config)
{
  struct device_s *dev = ctx->dev;
  struct max3420_usbdev_private_s *pv = dev->drv_pv;

  error_t err = -EAGAIN;
  assert(pv->cfg == NULL);

  switch (cfg->type)
    {
    case DEV_USBDEV_UNCONFIGURE:
    case DEV_USBDEV_SET_ADDRESS:
      pv->cfg = cfg;
      pv->pending |= MAX3420_CFG_MASK;
      max3420_process_config(pv);
      break;
    case DEV_USBDEV_CONFIGURE:
      err = max3420_usbdev_check_config(pv, cfg);
      break;
#if (CONFIG_USBDEV_MAX_ALTERNATE_COUNT > 0)
    case DEV_USBDEV_CHANGE_INTERFACE:
      err = max3420_usbdev_change_interface(pv, cfg);
      break;
#endif
   default:
      err = -EINVAL;
      break;
    }

  return err;
}

static DEV_USE(max3420_usbdev_use)
{
  struct device_accessor_s *accessor = param;

  struct device_s *dev = accessor->dev;

  switch (op)
    {
    case DEV_USE_START:
      if (dev->start_count == 0)
        max3420_usbdev_start_stop(dev, MAX3420_CNT_MASK);
      break;
    case DEV_USE_STOP:
      if (dev->start_count == 0)
        {
          /* Hide device to host */
          max3420_usbdev_start_stop(dev, MAX3420_DIS_MASK);
        }
      break;
    default:
      return dev_use_generic(param, op);
    }

  return 0;
}

static DEV_USBDEV_ALLOC(max3420_usbdev_alloc)
{
  return mem_alloc(size, mem_scope_sys);
}

static DEV_USBDEV_FREE(max3420_usbdev_free)
{
  mem_free(ptr);
}

static DEV_IRQ_SRC_PROCESS(max3420_irq_source_process)
{
  struct device_s *dev = ep->base.dev;
  struct max3420_usbdev_private_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_bytecode_rq_s *srq = &pv->spi_rq;

  lock_spin(&dev->lock);

  pv->icount++;

  dev_spi_bytecode_start(&pv->spi, srq, &max3420_entry_irq, 0);

  lock_release(&dev->lock);
}

static void max3420_usbdev_tr_done(struct device_s *dev)
{
  struct max3420_usbdev_private_s *pv = dev->drv_pv;
  struct dev_usbdev_request_s * tr;

  uint8_t msk = pv->done & MAX3420_EP_MASK;

  while(msk)
    {
      uint8_t idx = __builtin_ctz(msk);
      tr = pv->tr[idx];

      msk ^= 1 << idx;

      if (tr == NULL)
        continue;

      usbdev_stack_request_done(&pv->usbdev_ctx, tr);
      pv->tr[idx] = NULL;
    }

  pv->done &= ~MAX3420_EP_MASK;
}

static void max3420_usbdev_config_done(struct device_s *dev)
{
  struct max3420_usbdev_private_s *pv = dev->drv_pv;
  struct dev_usbdev_request_s * tr;

  switch (pv->cfg->type)
    {
    case DEV_USBDEV_SET_ADDRESS:
      break;
    case DEV_USBDEV_UNCONFIGURE:
      /* End all transfer */
      for (uint8_t i = 0; i < MAX3420_EP_COUNT; i++)
        {
          tr = pv->tr[i];
          if (tr == NULL)
            continue;

          tr->error = -EPIPE;
          pv->tr[i] = NULL;

          usbdev_stack_request_done(&pv->usbdev_ctx, tr);
        }
      break;
    default:
      abort();
    }

  usbdev_stack_config_done(&pv->usbdev_ctx);
  pv->done &= ~MAX3420_CFG_MASK;
  pv->cfg = NULL;
}

static error_t max3420_usbdev_clean(struct device_s *dev)
{
  struct max3420_usbdev_private_s *pv = dev->drv_pv;

  device_irq_source_unlink(dev, &pv->src_ep, 1);
  dev_drv_spi_bytecode_cleanup(&pv->spi, &pv->spi_rq);

  mem_free(pv);

  return 0;
}

static void max3420_usbdev_raise_event(struct max3420_usbdev_private_s *pv)
{
  struct dev_usbdev_request_s * tr = pv->tr[0];

  if (tr)
  /* Terminate on-going transfer */
    {
      tr->error = -EIO;
      tr->event = max3420_usbdev_set_event(pv);

      pv->done &= ~MAX3420_EP0_MASK;
      pv->pending &= ~MAX3420_EP0_MASK;

      usbdev_stack_request_done(&pv->usbdev_ctx, tr);
      pv->tr[0] = NULL;
    }
}

static const struct dev_usbdev_driver_ops_s max3420_usbdev_ops_s = {
  .f_transfer = max3420_usbdev_transfer,
  .f_config = max3420_usbdev_config,
  .f_alloc = max3420_usbdev_alloc,
  .f_free = max3420_usbdev_free,
  .f_endpoint = max3420_usbdev_endpoint
};

static KROUTINE_EXEC(max3420_spi_rq_done)
{
  struct dev_spi_ctrl_bytecode_rq_s *srq = KROUTINE_CONTAINER(kr, *srq, base.base.kr);
  struct device_s *dev = srq->base.base.pvdata;
  struct max3420_usbdev_private_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->pending & MAX3420_INIT_MASK)
    {
      pv->pending &= ~MAX3420_INIT_MASK;

      if (srq->base.err)
        {
          max3420_usbdev_clean(dev);
          device_async_init_done(dev, -EIO);
          goto end;
        }

      pv->done &= ~MAX3420_INIT_MASK;

      /* Initialise USB stack here */
      usbdev_stack_init(dev, &pv->usbdev_ctx, MAX3420_EPIN_MSK, MAX3420_EPOUT_MSK,
                        &max3420_usbdev_ops_s);

      device_async_init_done(dev, 0);
    }

  assert(!srq->base.err);

  if (pv->event || (pv->done & MAX3420_DIS_MASK))
    max3420_usbdev_raise_event(pv);

  if (pv->done & MAX3420_CFG_MASK)
    max3420_usbdev_config_done(dev);

  if (pv->done & MAX3420_EP_MASK)
    max3420_usbdev_tr_done(dev);

  if (pv->icount != bc_get_reg(&srq->vm, R_ICOUNT))
    {
      if (dev_spi_bytecode_start(&pv->spi, srq, &max3420_entry_irq, 0) == 0)
        goto end;
    }

  /* Start pending transfer if necessary */
  max3420_usbdev_process_pending(dev);

end:
  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_INIT(max3420_usbdev_init);
static DEV_CLEANUP(max3420_usbdev_cleanup);
static DEV_USE(max3420_usbdev_use);

DRIVER_DECLARE(max3420_drv, 0, "MAX3420 USB controller", max3420_usbdev,
               DRIVER_USBDEV_METHODS(max3420_usbdev));

DRIVER_REGISTER(max3420_drv);

static DEV_INIT(max3420_usbdev_init)
{
  struct max3420_usbdev_private_s *pv;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  struct dev_spi_ctrl_bytecode_rq_s *srq = &pv->spi_rq;

  struct device_gpio_s *gpio;
  struct device_timer_s *timer;

  static const struct dev_spi_ctrl_config_s spi_config = {
    .ck_mode = DEV_SPI_CK_MODE_0,
    .bit_order = DEV_SPI_MSB_FIRST,
    .miso_pol = DEV_SPI_ACTIVE_HIGH,
    .mosi_pol = DEV_SPI_ACTIVE_HIGH,
    .cs_pol   = DEV_SPI_ACTIVE_LOW,
    .bit_rate1k = CONFIG_DRIVER_USBDEV_MAX3420_SPI_BITRATE >> 10,
    .word_width = 8,
  };

  if (dev_drv_spi_bytecode_init(dev, srq, &max3420_bytecode,
                                &spi_config, &pv->spi, &gpio, &timer))
    goto err_mem;

  /* Base 500 us time */
  dev_timer_init_sec(timer, &pv->bt, 0, 500, 1000000);

  srq->base.base.pvdata = dev;

  /* init GPIO stuff */

  static const gpio_width_t pin_wmap[3] = {1, 1, 1};

  if (device_res_gpio_map(dev, "rst:1 nirq:1 gpx:1", pv->pin_map, NULL))
    goto err_srq;

  srq->gpio_map = pv->pin_map;
  srq->gpio_wmap = pin_wmap;

  if (device_gpio_map_set_mode(gpio, pv->pin_map, pin_wmap, 3,
                               DEV_PIN_PUSHPULL, DEV_PIN_INPUT,
                               DEV_PIN_INPUT))
    goto err_srq;

  /* kroutine */
  kroutine_init_deferred(&srq->base.base.kr, &max3420_spi_rq_done);

  /* Disable bytecode trace */
  bc_set_trace(&srq->vm, 0, 0);

  /* irq io pin */
  device_irq_source_init(dev, &pv->src_ep, 1, &max3420_irq_source_process);

  if (device_irq_source_link(dev, &pv->src_ep, 1, 1))
    goto err_srq;

  /* Start initialisation */

  bc_set_reg(&srq->vm, R_CTX_PV, (uintptr_t)pv);

  pv->pending = MAX3420_INIT_MASK;

  ensure(!dev_spi_bytecode_start(&pv->spi, srq, &max3420_entry_reset, 0));

  return -EAGAIN;

 err_srq:
  dev_drv_spi_bytecode_cleanup(&pv->spi, &pv->spi_rq);
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(max3420_usbdev_cleanup)
{
  struct max3420_usbdev_private_s *pv = dev->drv_pv;

  if (usbdev_stack_cleanup(&pv->usbdev_ctx))
    return -EBUSY;

  max3420_usbdev_clean(dev);

  return 0;
}

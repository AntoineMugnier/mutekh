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

  Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2015
*/

#define LOGK_MODULE_ID "spim"

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/spi.h>
#include <device/class/iomux.h>

#include <arch/nrf5x/spim.h>
#if defined(CONFIG_DRIVER_NRF52_SPIM_PAN58)
# include <arch/nrf5x/ppi.h>
# include <arch/nrf5x/gpiote.h>
#endif

#define GPIOTE_ADDR NRF_PERIPHERAL_ADDR(NRF5X_GPIOTE)
#define GPIOTE_CHANNEL CONFIG_DRIVER_NRF52_SPIM_PAN58_GPIOTE_FIRST
#define PPI_CHANNEL CONFIG_DRIVER_NRF52_SPIM_PAN58_PPI_FIRST

DRIVER_PV(struct nrf5x_spim_context_s
{
  uintptr_t addr;

  size_t transferred;

  union {
    uint8_t byte[16];
    uint32_t word[4];
  } buffer_in, buffer_out;

  struct dev_irq_src_s irq_ep[1];
  struct dev_spi_ctrl_transfer_s *current_transfer;
  bool_t buffered_in : 1;

  struct dev_spi_ctrl_context_s spi_ctrl_ctx;
});

static DEV_SPI_CTRL_CONFIG(nrf5x_spim_config)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_spim_context_s *pv = dev->drv_pv;
  uint32_t config = 0;
  uint32_t rate;

  if (cfg->word_width != 8
      || cfg->miso_pol != DEV_SPI_ACTIVE_HIGH
      || cfg->mosi_pol != DEV_SPI_ACTIVE_HIGH) {
    return -ENOTSUP;
  }

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (pv->current_transfer != NULL)
    return -EBUSY;

  switch (cfg->ck_mode) {
  case DEV_SPI_CK_MODE_0:
    config |= NRF_SPIM_CONFIG_CPOL_ACTIVEHIGH | NRF_SPIM_CONFIG_CPHA_LEADING;
    break;
  case DEV_SPI_CK_MODE_1:
    config |= NRF_SPIM_CONFIG_CPOL_ACTIVELOW | NRF_SPIM_CONFIG_CPHA_LEADING;
    break;
  case DEV_SPI_CK_MODE_2:
    config |= NRF_SPIM_CONFIG_CPOL_ACTIVEHIGH | NRF_SPIM_CONFIG_CPHA_TRAILING;
    break;
  case DEV_SPI_CK_MODE_3:
    config |= NRF_SPIM_CONFIG_CPOL_ACTIVELOW | NRF_SPIM_CONFIG_CPHA_TRAILING;
    break;
  }

  config |= (cfg->bit_order == DEV_SPI_MSB_FIRST)
    ? NRF_SPIM_CONFIG_ORDER_MSBFIRST
    : NRF_SPIM_CONFIG_ORDER_LSBFIRST;

  nrf_reg_set(pv->addr, NRF_SPIM_CONFIG, config);

  rate = cfg->bit_rate1k * 1024;
  if (rate > 8000000) {
    printk("nRF5x SPI Warning: bit rate capped to 8MHz (was %d)\n", rate);
    rate = 8000000;
  }

  nrf_reg_set(pv->addr, NRF_SPIM_FREQUENCY,
              NRF_SPIM_FREQUENCY_(rate));

  return 0;
}

#define nrf5x_spim_select (dev_spi_ctrl_select_t*)dev_driver_notsup_fcn

static void nrf5x_spim_transfer_ended(struct nrf5x_spim_context_s *pv)
{
  struct dev_spi_ctrl_transfer_s *tr = pv->current_transfer;

  logk_trace("%s, count %d transferred %d", __FUNCTION__, tr->data.count, pv->transferred);

  if (tr->data.in && tr->data.in_width)
    logk_trace("in: %P", nrf_reg_get(pv->addr, NRF_SPIM_RXD_PTR), pv->transferred);

  assert(tr);

  if (pv->buffered_in) {
    switch (tr->data.in_width) {
    case 2: {
      uint16_t *dst = (uint16_t *)tr->data.in;

      for (size_t i = 0; i < pv->transferred; ++i)
        dst[i] = pv->buffer_in.byte[i];

      break;
    }

    case 4: {
      uint32_t *dst = (uint32_t *)tr->data.in;

      for (size_t i = 0; i < pv->transferred; ++i)
        dst[i] = pv->buffer_in.byte[i];

      break;
    }
    }
  }

  tr->data.in = (void *)((uintptr_t)tr->data.in + pv->transferred * tr->data.in_width);
  tr->data.out = (const void *)((uintptr_t)tr->data.out + pv->transferred * tr->data.out_width);
  tr->data.count -= pv->transferred;
}

static void nrf5x_spim_next_start(struct nrf5x_spim_context_s *pv)
{
  struct dev_spi_ctrl_transfer_s *tr = pv->current_transfer;
  size_t count;

  assert(tr);

  count = __MIN(255, tr->data.count);

  nrf_reg_set(pv->addr, NRF_SPIM_RXD_LIST, NRF_SPIM_RXD_LIST_DISABLED);

  switch (tr->data.out_width) {
  case 0:
    nrf_reg_set(pv->addr, NRF_SPIM_ORC, endian_le32_na_load(tr->data.out));
    nrf_reg_set(pv->addr, NRF_SPIM_TXD_PTR, 0);
    break;

  case 1:
    if ((uintptr_t)tr->data.out < 0x20000000) {
      uintptr_t offset = (uintptr_t)tr->data.out % sizeof(pv->buffer_out);
      uint32_t *src = (uint32_t *)((uintptr_t)tr->data.out - offset);

      for (size_t i = offset / 4; i < sizeof(pv->buffer_out) / 4; ++i)
        pv->buffer_out.word[i] = src[i];

      nrf_reg_set(pv->addr, NRF_SPIM_TXD_PTR, (uintptr_t)pv->buffer_out.byte + offset);
      count = __MIN(count, sizeof(pv->buffer_out) - offset);
    } else {
      nrf_reg_set(pv->addr, NRF_SPIM_TXD_PTR, (uintptr_t)tr->data.out);
    }

    break;

  case 2: {
    const uint16_t *src = (const uint16_t *)tr->data.out;

    count = __MIN(count, sizeof(pv->buffer_out));
    for (size_t i = 0; i < sizeof(pv->buffer_out) / 4; ++i)
      pv->buffer_out.byte[i] = src[i];

    nrf_reg_set(pv->addr, NRF_SPIM_TXD_PTR, (uintptr_t)pv->buffer_out.byte);
    break;
  }

  case 4: {
    const uint32_t *src = (const uint32_t *)tr->data.out;

    count = __MIN(count, sizeof(pv->buffer_out));
    for (size_t i = 0; i < sizeof(pv->buffer_out) / 4; ++i)
      pv->buffer_out.byte[i] = src[i];

    nrf_reg_set(pv->addr, NRF_SPIM_TXD_PTR, (uintptr_t)pv->buffer_out.byte);
    break;
  }
  }

  pv->buffered_in = 0;

  if (tr->data.in) {
    switch (tr->data.in_width) {
    case 0:
      nrf_reg_set(pv->addr, NRF_SPIM_RXD_PTR, 0);
      break;

    case 1:
      nrf_reg_set(pv->addr, NRF_SPIM_RXD_PTR, (uintptr_t)tr->data.in);
      break;

    case 2:
    case 4:
      nrf_reg_set(pv->addr, NRF_SPIM_RXD_PTR, (uintptr_t)pv->buffer_in.byte);
      count = __MIN(count, sizeof(pv->buffer_in));
      pv->buffered_in = 1;
      break;
    }
  } else {
    nrf_reg_set(pv->addr, NRF_SPIM_RXD_PTR, 0);
  }

  logk_trace("%s count %d out w %d in w %d%s", __FUNCTION__,
          count,
          tr->data.out_width,
          tr->data.in_width, pv->buffered_in ? " buffered" : "");

  if (tr->data.out_width)
    logk_trace("out: %P", nrf_reg_get(pv->addr, NRF_SPIM_TXD_PTR), count);

  nrf_reg_set(pv->addr, NRF_SPIM_TXD_MAXCNT, tr->data.out_width ? count : 0);
  nrf_reg_set(pv->addr, NRF_SPIM_RXD_MAXCNT, count);

#if defined(CONFIG_DRIVER_NRF52_SPIM_PAN58)
  if (nrf_reg_get(pv->addr, NRF_SPIM_RXD_MAXCNT) <= 1
      && nrf_reg_get(pv->addr, NRF_SPIM_TXD_MAXCNT) == 1)
    nrf_ppi_enable(PPI_CHANNEL);
  else
    nrf_ppi_disable(PPI_CHANNEL);
#endif

  pv->transferred = count;

  nrf_it_enable(pv->addr, NRF_SPIM_END);
  nrf_task_trigger(pv->addr, NRF_SPIM_START);
}

static DEV_IRQ_SRC_PROCESS(nrf5x_spim_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_spim_context_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_transfer_s *tr = pv->current_transfer;

  logk_trace("%s", __FUNCTION__);

  LOCK_SPIN_SCOPED(&dev->lock);

  if (nrf_event_check(pv->addr, NRF_SPIM_END)) {
    nrf_event_clear(pv->addr, NRF_SPIM_END);

    if (tr) {
      nrf5x_spim_transfer_ended(pv);

      if (tr->data.count) {
        nrf5x_spim_next_start(pv);
      } else {
        pv->current_transfer = NULL;
        kroutine_exec(&tr->kr);
      }
    }
  }
}

static DEV_SPI_CTRL_TRANSFER(nrf5x_spim_transfer)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_spim_context_s *pv = dev->drv_pv;
  bool_t done = 1;

  logk_trace("%s", __FUNCTION__);

  LOCK_SPIN_IRQ(&dev->lock);

  tr->err = 0;

  if (pv->current_transfer) {
    tr->err = -EBUSY;
  } else if (tr->cs_op != DEV_SPI_CS_NOP_NOP) {
    tr->err = -ENOTSUP;
  } else if (tr->data.count > 0) {
    done = 0;

    pv->current_transfer = tr;

    nrf5x_spim_next_start(pv);
  }

 out:
  LOCK_RELEASE_IRQ(&dev->lock);

  if (done)
    kroutine_exec(&tr->kr);
}

#define nrf5x_spim_cscfg (dev_spi_ctrl_cscfg_t*)dev_driver_notsup_fcn
#define nrf5x_spim_use dev_use_generic

static DEV_INIT(nrf5x_spim_init)
{
  struct nrf5x_spim_context_s *pv;
  iomux_io_id_t id[3];

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto free_pv;

  pv->current_transfer = NULL;

#ifdef CONFIG_DEVICE_SPI_REQUEST
  if (dev_spi_context_init(dev, &pv->spi_ctrl_ctx))
    goto free_pv;
#endif

  if (device_iomux_setup(dev, ">clk <miso? >mosi?", NULL, id, NULL))
    goto free_queue;

  nrf_reg_set(
              pv->addr, NRF_SPIM_PSEL_SCK,
              id[0] != IOMUX_INVALID_ID ? id[0] : (uint32_t)-1);
  nrf_reg_set(
              pv->addr, NRF_SPIM_PSEL_MISO,
              id[1] != IOMUX_INVALID_ID ? id[1] : (uint32_t)-1);
  nrf_reg_set(
              pv->addr, NRF_SPIM_PSEL_MOSI,
              id[2] != IOMUX_INVALID_ID ? id[2] : (uint32_t)-1);

  nrf_reg_set(pv->addr, NRF_SPIM_ENABLE, NRF_SPIM_ENABLE_ENABLED);

  nrf_it_disable(pv->addr, NRF_SPIM_END);

#if defined(CONFIG_DRIVER_NRF52_SPIM_PAN58)
  nrf_reg_set(GPIOTE_ADDR, NRF_GPIOTE_CONFIG(GPIOTE_CHANNEL), 0
              | NRF_GPIOTE_CONFIG_MODE_EVENT
              | NRF_GPIOTE_CONFIG_PSEL(id[0])
              | NRF_GPIOTE_CONFIG_POLARITY_TOGGLE);

  nrf_ppi_setup(PPI_CHANNEL,
                GPIOTE_ADDR, NRF_GPIOTE_IN(GPIOTE_CHANNEL),
                pv->addr, NRF_SPIM_STOP);
#endif


  device_irq_source_init(dev, pv->irq_ep, 1, &nrf5x_spim_irq);
  if (device_irq_source_link(dev, pv->irq_ep, 1, -1))
    goto free_queue;

  return 0;

 free_queue:
#ifdef CONFIG_DEVICE_SPI_REQUEST
  dev_spi_context_cleanup(&pv->spi_ctrl_ctx);
#endif
 free_pv:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(nrf5x_spim_cleanup)
{
  struct nrf5x_spim_context_s *pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_SPI_REQUEST
  if (!dev_rq_queue_isempty(&pv->spi_ctrl_ctx.queue))
    return -EBUSY;

  dev_spi_context_cleanup(&pv->spi_ctrl_ctx);
#endif

  nrf_it_disable(pv->addr, NRF_SPIM_END);

  device_irq_source_unlink(dev, pv->irq_ep, 1);

  nrf_reg_set(pv->addr, NRF_SPIM_PSEL_SCK, (uint32_t)-1);
  nrf_reg_set(pv->addr, NRF_SPIM_PSEL_MISO, (uint32_t)-1);
  nrf_reg_set(pv->addr, NRF_SPIM_PSEL_MOSI, (uint32_t)-1);

  device_iomux_cleanup(dev);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(nrf5x_spim_drv, 0, "nRF52 SPIM", nrf5x_spim,
               DRIVER_SPI_CTRL_METHODS(nrf5x_spim));

DRIVER_REGISTER(nrf5x_spim_drv);


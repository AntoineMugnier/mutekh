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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2014
*/

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

#include <arch/nrf5x/spi.h>

#define dprintk(...) do{}while(0)

DRIVER_PV(struct nrf5x_spi_context_s
{
  uintptr_t addr;

  struct dev_irq_src_s irq_ep[1];
  uint8_t words_in_transit;

  struct dev_spi_ctrl_transfer_s *current_transfer;
  uint32_t ctrl;
  uint32_t route;

#ifdef CONFIG_DEVICE_SPI_REQUEST
  struct dev_spi_ctrl_context_s spi_ctrl_ctx;
#endif
});

static DEV_SPI_CTRL_CONFIG(nrf5x_spi_config)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_spi_context_s *pv = dev->drv_pv;
  error_t err = 0;
  uint32_t config = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->current_transfer != NULL) {
    err = -EBUSY;
    goto out;
  }

  if (cfg->word_width != 8) {
    err = -ENOTSUP;
    goto out;
  }

  if (cfg->miso_pol != DEV_SPI_ACTIVE_HIGH
      || cfg->mosi_pol != DEV_SPI_ACTIVE_HIGH) {
    err = -ENOTSUP;
    goto out;
  }

  switch (cfg->ck_mode) {
  case DEV_SPI_CK_MODE_1:
    config |= NRF_SPI_CONFIG_CPOL_ACTIVELOW
      | NRF_SPI_CONFIG_CPHA_LEADING;
    break;
  case DEV_SPI_CK_MODE_3:
    config |= NRF_SPI_CONFIG_CPOL_ACTIVELOW
      | NRF_SPI_CONFIG_CPHA_TRAILING;
    break;
  case DEV_SPI_CK_MODE_0:
    config |= NRF_SPI_CONFIG_CPOL_ACTIVEHIGH
      | NRF_SPI_CONFIG_CPHA_LEADING;
    break;
  case DEV_SPI_CK_MODE_2:
    config |= NRF_SPI_CONFIG_CPOL_ACTIVEHIGH
      | NRF_SPI_CONFIG_CPHA_TRAILING;
    break;
  }

  config |= (cfg->bit_order == DEV_SPI_MSB_FIRST)
    ? NRF_SPI_CONFIG_ORDER_MSBFIRST
    : NRF_SPI_CONFIG_ORDER_LSBFIRST;

  nrf_reg_set(pv->addr, NRF_SPI_CONFIG, config);
  nrf_reg_set(pv->addr, NRF_SPI_FREQUENCY,
              NRF_SPI_FREQUENCY_(cfg->bit_rate));

 out:
  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

#define nrf5x_spi_select (dev_spi_ctrl_select_t*)dev_driver_notsup_fcn

static void nrf5x_spi_tr_put_one(struct nrf5x_spi_context_s *pv,
                                 struct dev_spi_ctrl_transfer_s *tr)
{
  uint8_t word = 0;

  switch (tr->data.out_width) {
  case 1:
    word = *(const uint8_t*)tr->data.out;
    break;
  case 2:
    word = *(const uint16_t*)tr->data.out;
    break;
  case 0:
  case 4:
    word = *(const uint32_t*)tr->data.out;
    break;
  }

  dprintk("SPI tx: %02x\n", word);

  nrf_reg_set(pv->addr, NRF_SPI_TXD, word);

  tr->data.out = (const void*)((uintptr_t)tr->data.out + tr->data.out_width);
}

static void nrf5x_spi_tr_get_one(struct nrf5x_spi_context_s *pv,
                                 struct dev_spi_ctrl_transfer_s *tr)
{
  uint8_t word = nrf_reg_get(pv->addr, NRF_SPI_RXD);

  dprintk("SPI rx: %02x\n", word);

  if (tr->data.in) {
    switch (tr->data.in_width) {
    case 1:
      *(uint8_t*)tr->data.in = word;
      break;
    case 2:
      *(uint16_t*)tr->data.in = word;
      break;
    case 4:
      *(uint32_t*)tr->data.in = word;
      break;
    }

    tr->data.in = (void*)((uint8_t*)tr->data.in + tr->data.in_width);
  }
}

static
void nrf5x_spi_transfer_start(
                                struct nrf5x_spi_context_s *pv,
                                struct dev_spi_ctrl_transfer_s *tr)
{
  pv->words_in_transit = 0;

  dprintk("SPI rq %d %d %P...", tr->data.count, tr->data.out_width, tr->data.out, tr->data.count);

  while (tr->data.count && pv->words_in_transit < 2) {
    nrf5x_spi_tr_put_one(pv, tr);
    tr->data.count--;
    pv->words_in_transit++;
  }
}

static DEV_IRQ_SRC_PROCESS(nrf5x_spi_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_spi_context_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_transfer_s *tr;

  lock_spin(&dev->lock);

  tr = pv->current_transfer;
  assert(tr);

  if (!nrf_event_check(pv->addr, NRF_SPI_READY)) {
    tr = NULL;
    goto out;
  }
  nrf_event_clear(pv->addr, NRF_SPI_READY);

  nrf5x_spi_tr_get_one(pv, tr);

  if (tr->data.count) {
    nrf5x_spi_tr_put_one(pv, tr);
    tr->data.count--;
  } else {
    // Only decrement words in transit when nothing is fed back in
    // mosi.
    pv->words_in_transit--;
  }

  if (pv->words_in_transit) {
    tr = NULL;
  } else {
    pv->current_transfer = NULL;
    nrf_it_disable(pv->addr, NRF_SPI_READY);
  }

 out:
  lock_release(&dev->lock);

  // tr is not NULL only if it is finished.
  if (tr) {
    dprintk(" done\n");
    kroutine_exec(&tr->kr);
  }
}

static DEV_SPI_CTRL_TRANSFER(nrf5x_spi_transfer)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_spi_context_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->current_transfer != NULL) {
    tr->err = -EBUSY;
  } else if (!((0x17 >> tr->data.out_width) & 1) || (tr->data.in && !((0x16 >> tr->data.in_width) & 1))) {
    printk("Error: out_width: %d, in_width: %d\n", tr->data.out_width, tr->data.in_width);
    printk("Error: out: %p, in: %p\n", tr->data.out, tr->data.in);
    tr->err = -EINVAL;
  } else {
    assert(tr->data.count > 0);

    pv->current_transfer = tr;
    tr->err = 0;

    nrf_event_clear(pv->addr, NRF_SPI_READY);
    nrf_it_enable(pv->addr, NRF_SPI_READY);
    nrf5x_spi_transfer_start(pv, tr);

    tr = NULL;
  }

  LOCK_RELEASE_IRQ(&dev->lock);

  if (tr)
    kroutine_exec(&tr->kr);
}


#define nrf5x_spi_use dev_use_generic

static DEV_INIT(nrf5x_spi_init)
{
  struct nrf5x_spi_context_s *pv;
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
              pv->addr, NRF_SPI_PSELSCK,
              id[0] != IOMUX_INVALID_ID ? id[0] : (uint32_t)-1);
  nrf_reg_set(
              pv->addr, NRF_SPI_PSELMISO,
              id[1] != IOMUX_INVALID_ID ? id[1] : (uint32_t)-1);
  nrf_reg_set(
              pv->addr, NRF_SPI_PSELMOSI,
              id[2] != IOMUX_INVALID_ID ? id[2] : (uint32_t)-1);

  nrf_it_disable(pv->addr, NRF_SPI_READY);

  device_irq_source_init(dev, pv->irq_ep, 1, &nrf5x_spi_irq);

  if (device_irq_source_link(dev, pv->irq_ep, 1, -1))
    goto free_queue;

  nrf_reg_set(pv->addr, NRF_SPI_ENABLE, NRF_SPI_ENABLE_ENABLED);

  return 0;

 free_queue:
#ifdef CONFIG_DEVICE_SPI_REQUEST
  dev_spi_context_cleanup(&pv->spi_ctrl_ctx);
#endif
 free_pv:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(nrf5x_spi_cleanup)
{
  struct nrf5x_spi_context_s *pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_SPI_REQUEST
  if (!dev_request_queue_isempty(&pv->queue.queue))
    return -EBUSY;

  dev_spi_context_cleanup(&pv->spi_ctrl_ctx);
#endif

  nrf_it_disable(pv->addr, NRF_SPI_READY);

  device_irq_source_unlink(dev, pv->irq_ep, 1);

  nrf_reg_set(pv->addr, NRF_SPI_PSELSCK, (uint32_t)-1);
  nrf_reg_set(pv->addr, NRF_SPI_PSELMISO, (uint32_t)-1);
  nrf_reg_set(pv->addr, NRF_SPI_PSELMOSI, (uint32_t)-1);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(nrf5x_spi_drv, 0, "nRF5x SPI", nrf5x_spi,
               DRIVER_SPI_CTRL_METHODS(nrf5x_spi));

DRIVER_REGISTER(nrf5x_spi_drv);


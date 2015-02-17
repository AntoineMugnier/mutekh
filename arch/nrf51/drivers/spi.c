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

#include <arch/nrf51/spi.h>

struct nrf51_spi_context_s
{
    uintptr_t addr;

    struct dev_irq_ep_s irq_ep[1];
    uint8_t words_in_transit;

    struct dev_spi_ctrl_transfer_s *current_transfer;
    uint32_t ctrl;
    uint32_t route;

#ifdef CONFIG_DEVICE_SPI_REQUEST
    struct dev_spi_ctrl_queue_s queue;
#endif
};

static DEV_SPI_CTRL_CONFIG(nrf51_spi_config)
{
    struct device_s *dev = accessor->dev;
    struct nrf51_spi_context_s *pv = dev->drv_pv;
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

    if (cfg->miso_pol != DEV_SPI_CS_ACTIVE_HIGH
        || cfg->mosi_pol != DEV_SPI_CS_ACTIVE_HIGH) {
        err = -ENOTSUP;
        goto out;
    }

    switch (cfg->ck_mode) {
    case DEV_SPI_CK_LOW_LEADING:
        config |= NRF51_SPI_CONFIG_CPOL_ACTIVELOW
            | NRF51_SPI_CONFIG_CPHA_LEADING;
        break;
    case DEV_SPI_CK_LOW_TRAILING:
        config |= NRF51_SPI_CONFIG_CPOL_ACTIVELOW
            | NRF51_SPI_CONFIG_CPHA_TRAILING;
        break;
    case DEV_SPI_CK_HIGH_LEADING:
        config |= NRF51_SPI_CONFIG_CPOL_ACTIVEHIGH
            | NRF51_SPI_CONFIG_CPHA_LEADING;
        break;
    case DEV_SPI_CK_HIGH_TRAILING:
        config |= NRF51_SPI_CONFIG_CPOL_ACTIVEHIGH
            | NRF51_SPI_CONFIG_CPHA_TRAILING;
        break;
    }

    config |= (cfg->bit_order == DEV_SPI_MSB_FIRST)
        ? NRF51_SPI_CONFIG_ORDER_MSBFIRST
        : NRF51_SPI_CONFIG_ORDER_LSBFIRST;

    nrf_reg_set(pv->addr, NRF51_SPI_CONFIG, config);
    nrf_reg_set(pv->addr, NRF51_SPI_FREQUENCY,
                NRF51_SPI_FREQUENCY_(cfg->bit_rate));

  out:
    LOCK_RELEASE_IRQ(&dev->lock);

    return err;
}

static DEV_SPI_CTRL_SELECT(nrf51_spi_select)
{
    return -ENOTSUP;
}

static void nrf51_spi_tr_put_one(
    struct nrf51_spi_context_s *pv,
    struct dev_spi_ctrl_transfer_s *tr)
{
    nrf_reg_set(pv->addr, NRF51_SPI_TXD, *(uint8_t*)tr->out);
    tr->out = (void*)((uintptr_t)tr->out + tr->out_width);
}

static void nrf51_spi_tr_get_one(
    struct nrf51_spi_context_s *pv,
    struct dev_spi_ctrl_transfer_s *tr)
{
    if (tr->in) {
        *(uint8_t*)tr->in = nrf_reg_get(pv->addr, NRF51_SPI_RXD);
        tr->in = (void*)((uintptr_t)tr->in + tr->in_width);
    }
}

static
bool_t nrf51_spi_transfer_start(
    struct nrf51_spi_context_s *pv,
    struct dev_spi_ctrl_transfer_s *tr)
{
    pv->words_in_transit = 0;

    while (tr->count && pv->words_in_transit < 2) {
        nrf51_spi_tr_put_one(pv, tr);
        tr->count--;
        pv->words_in_transit++;
    }

    return 0;
}

static DEV_IRQ_EP_PROCESS(nrf51_spi_irq)
{
    struct device_s *dev = ep->dev;
    struct nrf51_spi_context_s *pv = dev->drv_pv;
    struct dev_spi_ctrl_transfer_s *tr = pv->current_transfer;

    lock_spin(&dev->lock);

    if (!nrf_event_check(pv->addr, NRF51_SPI_READY))
        goto out;
    nrf_event_clear(pv->addr, NRF51_SPI_READY);

    nrf51_spi_tr_get_one(pv, tr);

    if (tr->count) {
        nrf51_spi_tr_put_one(pv, tr);
        tr->count--;
    } else {
        // Only decrement words in transit when nothing is fed back in
        // mosi.
        pv->words_in_transit--;
    }

    if (pv->words_in_transit)
        tr = NULL;
    else
        pv->current_transfer = NULL;

  out:
    lock_release(&dev->lock);

    // tr is not NULL only if it is finished.
    if (tr) {
        nrf_it_disable(pv->addr, NRF51_SPI_READY);
        kroutine_exec(&tr->kr, 0);
    }
}

static DEV_SPI_CTRL_TRANSFER(nrf51_spi_transfer)
{
    struct device_s *dev = accessor->dev;
    struct nrf51_spi_context_s *pv = dev->drv_pv;
    bool_t done = 1;

    LOCK_SPIN_IRQ(&dev->lock);

    if (pv->current_transfer != NULL)
    {
        tr->err = -EBUSY;
    }
    else
    {
        assert(tr->count > 0);

        pv->current_transfer = tr;

        tr->accessor = accessor;
        tr->err = 0;

        nrf_event_clear(pv->addr, NRF51_SPI_READY);
        nrf_it_enable(pv->addr, NRF51_SPI_READY);

        done = nrf51_spi_transfer_start(pv, tr);
    }

    LOCK_RELEASE_IRQ(&dev->lock);

    if (done) {
        pv->current_transfer = NULL;
        kroutine_exec(&tr->kr, cpu_is_interruptible());
    }
}

#ifdef CONFIG_DEVICE_SPI_REQUEST

static DEV_SPI_CTRL_QUEUE(nrf51_spi_queue)
{
    struct device_s *dev = accessor->dev;
    struct nrf51_spi_context_s *pv = dev->drv_pv;
    return &pv->queue;
}

#endif

static DEV_INIT(nrf51_spi_init);
static DEV_CLEANUP(nrf51_spi_cleanup);

#define nrf51_spi_use dev_use_generic

DRIVER_DECLARE(nrf51_spi_drv, "nRF51 SPI", nrf51_spi,
               DRIVER_SPI_CTRL_METHODS(nrf51_spi));

DRIVER_REGISTER(nrf51_spi_drv);

static DEV_INIT(nrf51_spi_init)
{
    struct nrf51_spi_context_s *pv;
    iomux_io_id_t id[3];

    dev->status = DEVICE_DRIVER_INIT_FAILED;

    pv = mem_alloc(sizeof(*pv), mem_scope_sys);
    dev->drv_pv = pv;

    if (!pv)
        return -ENOMEM;

    if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
        goto free_pv;

    pv->current_transfer = NULL;

#ifdef CONFIG_DEVICE_SPI_REQUEST
    if (dev_spi_queue_init(dev, &pv->queue))
        goto free_pv;
#endif

    if (device_iomux_setup(dev, ">clk <miso? >mosi?", NULL, id, NULL))
        goto free_queue;

    nrf_reg_set(
        pv->addr, NRF51_SPI_PSELSCK,
        id[0] != IOMUX_INVALID_ID ? id[0] : (uint32_t)-1);
    nrf_reg_set(
        pv->addr, NRF51_SPI_PSELMISO,
        id[1] != IOMUX_INVALID_ID ? id[1] : (uint32_t)-1);
    nrf_reg_set(
        pv->addr, NRF51_SPI_PSELMOSI,
        id[2] != IOMUX_INVALID_ID ? id[2] : (uint32_t)-1);

    nrf_it_disable(pv->addr, NRF51_SPI_READY);

    device_irq_source_init(dev, pv->irq_ep, 1,
                           &nrf51_spi_irq, DEV_IRQ_SENSE_HIGH_LEVEL);

    if (device_irq_source_link(dev, pv->irq_ep, 1, -1))
        goto free_queue;

    dev->drv = &nrf51_spi_drv;
    dev->status = DEVICE_DRIVER_INIT_DONE;

    return 0;

  free_queue:
#ifdef CONFIG_DEVICE_SPI_REQUEST
    dev_spi_queue_cleanup(&pv->queue);
#endif
  free_pv:
    mem_free(pv);
    return -1;
}

DEV_CLEANUP(nrf51_spi_cleanup)
{
    struct nrf51_spi_context_s *pv = dev->drv_pv;

    nrf_it_disable(pv->addr, NRF51_SPI_READY);

    device_irq_source_unlink(dev, pv->irq_ep, 1);

    nrf_reg_set(pv->addr, NRF51_SPI_PSELSCK, (uint32_t)-1);
    nrf_reg_set(pv->addr, NRF51_SPI_PSELMISO, (uint32_t)-1);
    nrf_reg_set(pv->addr, NRF51_SPI_PSELMOSI, (uint32_t)-1);

#ifdef CONFIG_DEVICE_SPI_REQUEST
    dev_spi_queue_cleanup(&pv->queue);
#endif

    mem_free(pv);
}

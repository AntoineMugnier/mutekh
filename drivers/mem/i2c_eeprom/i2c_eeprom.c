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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2014
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/class/mem.h>
#include <device/class/i2c.h>

#define dprintk(k...) do {} while (0)
//#define dprintk printk

struct i2c_eeprom_priv_s
{
    struct device_i2c_ctrl_s bus;
    uint8_t saddr;
    uint8_t addr[2];
    uint8_t addr_size;
    uint32_t size;
    uint32_t page_size_l2;
    struct dev_i2c_ctrl_transaction_rq_s i2c_req;
    struct dev_i2c_ctrl_transaction_data_s i2c_transfer[2];
    dev_request_queue_root_t queue;

    size_t sc;
    size_t page;

    bool_t last_was_write;
    bool_t busy;
};

DRIVER_PV(struct i2c_eeprom_priv_s);

static DEV_MEM_INFO(i2c_eeprom_info)
{
    struct device_s *dev = accessor->dev;
    struct i2c_eeprom_priv_s *pv = dev->drv_pv;

    if (band_index > 0)
        return -ENOENT;

    memset(info, 0, sizeof(*info));
  
    info->type = DEV_MEM_FLASH;
    info->flags = DEV_MEM_PARTIAL_WRITE
        | DEV_MEM_PARTIAL_READ
        | DEV_MEM_CROSS_READ;

    info->size = pv->size >> pv->page_size_l2;
    info->page_log2 = pv->page_size_l2;

    return 0;
}

static void i2c_rq_run(
    struct device_s *dev,
    struct dev_mem_rq_s *rq);

static void i2c_eeprom_request_run(
    struct device_s *dev)
{
    struct i2c_eeprom_priv_s *pv = dev->drv_pv;
    struct dev_mem_rq_s *rq = dev_mem_rq_head(&pv->queue);

    pv->busy = 1;
    pv->page = 0;
    pv->sc = 0;

    i2c_rq_run(dev, rq);
}

static KROUTINE_EXEC(i2c_eeprom_done)
{
    struct i2c_eeprom_priv_s *pv;
    struct dev_mem_rq_s *rq, *done = NULL;
    struct device_s *dev;

    pv = KROUTINE_CONTAINER(kr, *pv, i2c_req.base.base.kr);
    dev = pv->i2c_req.pvdata;

    LOCK_SPIN_IRQ(&dev->lock);

    rq = dev_mem_rq_head(&pv->queue);

    if (pv->i2c_req.error) {
        // Write ACK polling
        if (pv->last_was_write && pv->i2c_req.error == -EHOSTUNREACH)
          goto out;

        rq->error = -EIO;
        done = rq;
        goto out;
    }

    pv->last_was_write = 0;

    switch ((uint16_t)rq->type) {
    case DEV_MEM_OP_PARTIAL_WRITE:
        pv->last_was_write = 1;
    case DEV_MEM_OP_PARTIAL_READ:
        done = rq;
        break;

    case DEV_MEM_OP_PAGE_WRITE:
    case DEV_MEM_OP_PAGE_READ:
        rq->error = -ENOTSUP;
        done = rq;
        goto out;
    }

  out:

    if (!done) {
        i2c_rq_run(dev, rq);
    } else {
        dev_mem_rq_pop(&pv->queue);
        if (dev_rq_queue_isempty(&pv->queue))
            pv->busy = 0;
        else
            i2c_eeprom_request_run(dev);
    }
    
    LOCK_RELEASE_IRQ(&dev->lock);

    if (!done)
        return;

    dev_mem_rq_done(done);
}

static void i2c_rq_run(
    struct device_s *dev,
    struct dev_mem_rq_s *rq)
{
    struct i2c_eeprom_priv_s *pv = dev->drv_pv;

    dev_i2c_ctrl_rq_init(&pv->i2c_req.base, i2c_eeprom_done);

    pv->i2c_req.base.saddr = pv->saddr + (rq->partial.addr >> (pv->addr_size * 8));
    pv->i2c_req.pvdata = dev;
    pv->i2c_req.transfer = pv->i2c_transfer;

    pv->i2c_req.transfer_count = 2;
    pv->i2c_transfer[0].data = pv->addr;
    pv->i2c_transfer[0].size = pv->addr_size;
    pv->i2c_transfer[0].type = DEV_I2C_CTRL_TRANSACTION_WRITE;

    if (pv->addr_size == 1) {
        pv->addr[0] = rq->partial.addr;
    } else {
        endian_be16_na_store(pv->addr, rq->partial.addr);
    }

    switch ((uint16_t)rq->type) {
    case DEV_MEM_OP_PARTIAL_READ:
        pv->i2c_transfer[1].data = rq->partial.data;
        pv->i2c_transfer[1].size = rq->partial.size;
        pv->i2c_transfer[1].type = DEV_I2C_CTRL_TRANSACTION_READ;

        dprintk("%s partial read @%x, size: %d\n", __FUNCTION__,
                (uint32_t)rq->partial.addr, rq->partial.data);
        break;

    case DEV_MEM_OP_PARTIAL_WRITE:
        pv->i2c_transfer[1].data = rq->partial.data;
        pv->i2c_transfer[1].size = rq->partial.size;
        pv->i2c_transfer[1].type = DEV_I2C_CTRL_TRANSACTION_WRITE;

        dprintk("%s partial write @%x, size: %d\n", __FUNCTION__,
                (uint32_t)rq->partial.addr, rq->partial.data);
        break;

    case DEV_MEM_OP_PAGE_READ:
    case DEV_MEM_OP_PAGE_WRITE:
      UNREACHABLE();
    }
    
    dev_i2c_transaction_start(&pv->bus, &pv->i2c_req);
}

static DEV_MEM_REQUEST(i2c_eeprom_request)
{
    struct device_s *dev = accessor->dev;
    struct i2c_eeprom_priv_s *pv = dev->drv_pv;

    LOCK_SPIN_IRQ(&dev->lock);

    logk("rq %x", rq->type);

    // Drop side requests
    rq->type &= DEV_MEM_OP_PARTIAL_READ
        | DEV_MEM_OP_PARTIAL_WRITE;

    switch (rq->type) {
    case DEV_MEM_OP_PARTIAL_READ:
    case DEV_MEM_OP_PARTIAL_WRITE:
      break;

    default:
        rq->error = -ENOTSUP;
        goto out;
    }

    dev_mem_rq_pushback(&pv->queue, rq);
    if (!pv->busy)
      i2c_eeprom_request_run(dev);
    rq = NULL;

out:
    LOCK_RELEASE_IRQ(&dev->lock);

    if (!rq)
        return;

    dev_mem_rq_done(rq);
}


#define i2c_eeprom_use dev_use_generic

static DEV_INIT(i2c_eeprom_init)
{
    struct i2c_eeprom_priv_s *pv;
    error_t err;
    uintptr_t page_size, size, addr_size;


    pv = mem_alloc(sizeof(*pv), mem_scope_sys);
    if (!pv)
        return -ENOMEM;

    err = dev_drv_i2c_transaction_init(dev, &pv->i2c_req, &pv->bus);
    if (err)
        goto fail;

    pv->saddr = pv->i2c_req.base.saddr;

    err = device_get_param_uint(dev, "size", &size);
    if (err)
        goto fail;

    err = device_get_param_uint(dev, "page_size", &page_size);
    if (err)
        goto fail;

    err = device_get_param_uint(dev, "addr_size", &addr_size);
    if (err)
        goto fail;

    pv->busy = 0;
    pv->page_size_l2 = bit_ctz(page_size);
    pv->size = size;
    pv->addr_size = addr_size;
    pv->last_was_write = 0;

    dev_rq_queue_init(&pv->queue);

    dev->drv_pv = pv;

    return 0;

  fail:
    free(pv);
    return err;
}

static DEV_CLEANUP(i2c_eeprom_cleanup)
{
    struct i2c_eeprom_priv_s *pv = dev->drv_pv;

    if (pv->busy)
        return -EBUSY;

    device_put_accessor(&pv->bus.base);
    dev_rq_queue_destroy(&pv->queue);
    mem_free(pv);

    return 0;
}

DRIVER_DECLARE(i2c_eeprom_drv, 0, "I2C eeprom", i2c_eeprom,
               DRIVER_MEM_METHODS(i2c_eeprom));

DRIVER_REGISTER(i2c_eeprom_drv);


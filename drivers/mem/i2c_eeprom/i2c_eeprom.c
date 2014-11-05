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
    struct device_i2c_s bus;
    uint8_t saddr;
    uint8_t addr[2];
    uint8_t addr_size;
    uint32_t size;
    uint32_t page_size_l2;
    struct dev_i2c_rq_s i2c_req;
    struct dev_i2c_transfer_s i2c_transfer[2];
    dev_request_queue_root_t queue;

    size_t sc;
    size_t page;

    bool_t last_was_write;
};

static DEV_MEM_INFO(i2c_eeprom_info)
{
    struct device_s *dev = accessor->dev;
    struct i2c_eeprom_priv_s *pv = dev->drv_pv;

    if (accessor->number > 0)
        return -ENOENT;

    if (band_index > 0)
        return -ENOENT;

    memset(info, 0, sizeof(*info));
  
    info->type = DEV_MEM_FLASH;
    info->flags = DEV_MEM_WRITABLE
        | DEV_MEM_PARTIAL_WRITE
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
    struct dev_request_s *drq;
    struct dev_mem_rq_s *rq;

    drq = dev_request_queue_head(&pv->queue);

    if (!drq)
        return;

    rq = dev_mem_rq_s_from_base(drq);

    pv->page = 0;
    pv->sc = 0;

    i2c_rq_run(dev, rq);
}

static KROUTINE_EXEC(i2c_eeprom_done)
{
    struct i2c_eeprom_priv_s *pv;
    struct dev_request_s *drq;
    struct dev_mem_rq_s *rq, *done = NULL;
    struct device_s *dev;

    pv = KROUTINE_CONTAINER(kr, *pv, i2c_req.base.kr);
    dev = pv->i2c_req.base.pvdata;

    LOCK_SPIN_IRQ(&dev->lock);

    drq = dev_request_queue_head(&pv->queue);
    rq = dev_mem_rq_s_from_base(drq);

    if (pv->i2c_req.error) {
        // Write ACK polling
        if (pv->last_was_write
            && pv->i2c_req.error_transfer == 0
            && pv->i2c_req.error_offset == 0
            && pv->i2c_req.error == -EHOSTUNREACH)
            goto out;

        rq->err = -EIO;
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
        pv->last_was_write = 1;
    case DEV_MEM_OP_PAGE_READ:
        rq->addr += 1 << pv->page_size_l2;
        rq->size -= 1 << pv->page_size_l2;

        if (!rq->size) {
            done = rq;
            break;
        }

        pv->page++;
        if (pv->page >> rq->sc_log2) {
            pv->sc++;
            pv->page = 0;
        }
    }

  out:

    if (!done) {
        i2c_rq_run(dev, rq);
    } else {
        dev_request_queue_pop(&pv->queue);
        i2c_eeprom_request_run(dev);
    }
    
    LOCK_RELEASE_IRQ(&dev->lock);

    if (!done)
        return;

    kroutine_exec(&done->base.kr, cpu_is_interruptible());
}

static void i2c_rq_run(
    struct device_s *dev,
    struct dev_mem_rq_s *rq)
{
    struct i2c_eeprom_priv_s *pv = dev->drv_pv;

    kroutine_init(&pv->i2c_req.base.kr, i2c_eeprom_done, KROUTINE_IMMEDIATE);

    pv->i2c_req.saddr = pv->saddr + (rq->addr >> (pv->addr_size * 8));
    pv->i2c_req.base.pvdata = dev;
    pv->i2c_req.transfer = pv->i2c_transfer;

    pv->i2c_req.transfer_count = 2;
    pv->i2c_transfer[0].data = pv->addr;
    pv->i2c_transfer[0].size = pv->addr_size;
    pv->i2c_transfer[0].type = DEV_I2C_WRITE;

    if (pv->addr_size == 1) {
        pv->addr[0] = rq->addr;
    } else {
        endian_be16_na_store(pv->addr, rq->addr);
    }

    switch ((uint16_t)rq->type) {
    case DEV_MEM_OP_PARTIAL_READ:
        pv->i2c_transfer[1].data = rq->data;
        pv->i2c_transfer[1].size = rq->size;
        pv->i2c_transfer[1].type = DEV_I2C_READ;

        dprintk("%s partial read @%x, size: %d\n", __FUNCTION__,
                (uint32_t)rq->addr, rq->data);
        break;

    case DEV_MEM_OP_PARTIAL_WRITE:
        pv->i2c_transfer[1].data = rq->data;
        pv->i2c_transfer[1].size = rq->size;
        pv->i2c_transfer[1].type = DEV_I2C_WRITE;

        dprintk("%s partial write @%x, size: %d\n", __FUNCTION__,
                (uint32_t)rq->addr, rq->data);
        break;

    case DEV_MEM_OP_PAGE_READ:
        pv->i2c_transfer[1].type = DEV_I2C_READ;

        dprintk("%s page read @%x\n", __FUNCTION__, (uint32_t)rq->addr);
        goto page;

    case DEV_MEM_OP_PAGE_WRITE:
        pv->i2c_transfer[1].type = DEV_I2C_WRITE;

        dprintk("%s page write @%x\n", __FUNCTION__, (uint32_t)rq->addr);

    page:
        pv->i2c_transfer[1].data
            = rq->sc_data[pv->sc]
            + (pv->page << pv->page_size_l2);
        pv->i2c_transfer[1].size = 1 << pv->page_size_l2;
        break;
    }
    
    DEVICE_OP(&pv->bus, request, &pv->i2c_req);
}

static DEV_MEM_REQUEST(i2c_eeprom_request)
{
    struct device_s *dev = accessor->dev;
    struct i2c_eeprom_priv_s *pv = dev->drv_pv;

    LOCK_SPIN_IRQ(&dev->lock);

    if (rq->type & DEV_MEM_OP_PAGE_ERASE) {
        rq->err = -ENOTSUP;
        goto out;
    }

    // Drop side requests
    rq->type &= DEV_MEM_OP_PARTIAL_READ
        | DEV_MEM_OP_PARTIAL_READ
        | DEV_MEM_OP_PAGE_READ
        | DEV_MEM_OP_PAGE_WRITE;
    
    if (!rq->type) {
        rq->err = -ENOTSUP;
        goto out;
    }

    // More than one request bit at a time
    if (rq->type & (rq->type - 1)) {
        rq->err = -EINVAL;
        goto out;
    }

    dev_request_queue_pushback(&pv->queue, &rq->base);
    i2c_eeprom_request_run(dev);
    rq = NULL;

out:
    LOCK_RELEASE_IRQ(&dev->lock);

    if (!rq)
        return;

    kroutine_exec(&rq->base.kr, cpu_is_interruptible());
}

static const struct driver_mem_s    i2c_eeprom_mem_drv =
{
    .class_     = DRIVER_CLASS_MEM,
    .f_info     = i2c_eeprom_info,
    .f_request  = i2c_eeprom_request,
};

static DEV_INIT(i2c_eeprom_init);
static DEV_CLEANUP(i2c_eeprom_cleanup);

const struct driver_s i2c_eeprom_drv =
{
    .desc        = "I2C eeprom",
    .f_init      = i2c_eeprom_init,
    .f_cleanup   = i2c_eeprom_cleanup,
    .classes     = {
        &i2c_eeprom_mem_drv,
        0,
    }
};

REGISTER_DRIVER(i2c_eeprom_drv);

static DEV_INIT(i2c_eeprom_init)
{
    struct i2c_eeprom_priv_s *pv;
    error_t err;
    uintptr_t page_size, size, addr_size;

    dev->status = DEVICE_DRIVER_INIT_FAILED;

    pv = mem_alloc(sizeof(*pv), mem_scope_sys);
    if (!pv)
        return -ENOMEM;

    err = device_get_param_dev_accessor(dev, "bus", &pv->bus, DRIVER_CLASS_I2C);
    if (err)
        goto fail;

    err = dev_i2c_res_get_addr(dev, &pv->saddr, 0);
    if (err)
        goto fail;

    err = device_get_param_uint(dev, "size", &size);
    if (err)
        goto fail;

    err = device_get_param_uint(dev, "page_size", &page_size);
    if (err)
        goto fail;

    err = device_get_param_uint(dev, "addr_size", &addr_size);
    if (err)
        goto fail;

    pv->page_size_l2 = __builtin_ctz(page_size);
    pv->size = size;
    pv->addr_size = addr_size;

    dev_request_queue_init(&pv->queue);

    dev->drv_pv = pv;
    dev->drv = &i2c_eeprom_drv;
    dev->status = DEVICE_DRIVER_INIT_DONE;

    return 0;

  fail:
    free(pv);
    return err;
}

static DEV_CLEANUP(i2c_eeprom_cleanup)
{
    struct i2c_eeprom_priv_s *pv = dev->drv_pv;

    device_put_accessor(&pv->bus);
    dev_request_queue_destroy(&pv->queue);
    mem_free(pv);
}

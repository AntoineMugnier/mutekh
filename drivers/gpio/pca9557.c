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

    Copyright (c) 2014 Nicolas Pouillon <nipo@ssji.net>
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>

#include <mutek/mem_alloc.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/request.h>
#include <device/class/gpio.h>
#include <device/class/i2c.h>

#include <mutek/printk.h>
#include <string.h>

enum pca9557_reg
{
    REG_INPUT,
    REG_OUTPUT,
    REG_POLARITY,
    REG_CONFIG,
};

DRIVER_PV(struct pca9557_private_s
{
    // Path to slave device
    struct device_i2c_s i2c_dev;
    uint8_t saddr;

    // Cache
    uint8_t output_value;
    uint8_t input_value;
    uint8_t input_mode;

    uint8_t command[2];

    // Queue
    dev_request_queue_root_t pending;

    // Outgoing request
    struct dev_i2c_rq_s i2c_req;
    struct dev_i2c_transfer_s i2c_transfer[2];
});

STRUCT_COMPOSE(pca9557_private_s, i2c_req);

static void pca9557_req_next(struct device_s *dev);

static void pca9557_req_done(
    struct device_s *dev,
    struct dev_gpio_rq_s *rq)
{
    struct pca9557_private_s *pv = dev->drv_pv;

    dev_gpio_rq_pop(&pv->pending);
    rq->base.drvdata = NULL;
    dev_gpio_rq_done(rq);
    pca9557_req_next(dev);
}

static KROUTINE_EXEC(pca9557_i2c_write_done)
{
    struct pca9557_private_s *pv
        = KROUTINE_CONTAINER(kr, *pv, i2c_req.base.kr);
    struct dev_gpio_rq_s *rq
        = dev_gpio_rq_head(&pv->pending);
    struct device_s *dev = pv->i2c_req.pvdata;

    LOCK_SPIN_IRQ_SCOPED(&dev->lock);

    rq->error = pv->i2c_req.error;
    pca9557_req_done(dev, rq);
}

static KROUTINE_EXEC(pca9557_i2c_read_done)
{
    struct pca9557_private_s *pv
        = KROUTINE_CONTAINER(kr, *pv, i2c_req.base.kr);
    struct dev_gpio_rq_s *rq
        = dev_gpio_rq_head(&pv->pending);
    struct device_s *dev = pv->i2c_req.pvdata;

    LOCK_SPIN_IRQ_SCOPED(&dev->lock);

    rq->error = pv->i2c_req.error;
    *rq->input.data = pv->input_value >> rq->io_first;

    pca9557_req_done(dev, rq);
}

static void pca9557_mode_update(
    struct pca9557_private_s *pv)
{
    dev_i2c_ctrl_rq_init(&pv->i2c_req.base, pca9557_i2c_write_done);

    pv->i2c_req.transfer = pv->i2c_transfer;
    pv->i2c_transfer[0].data = pv->command;
    pv->i2c_transfer[0].size = 2;
    pv->i2c_transfer[0].type = DEV_I2C_WRITE;
    pv->i2c_req.transfer_count = 1;
    pv->command[0] = REG_CONFIG;
    pv->command[1] = pv->input_mode;

    DEVICE_OP(&pv->i2c_dev, request, &pv->i2c_req);
}

static void pca9557_output_update(
    struct pca9557_private_s *pv)
{
    dev_i2c_ctrl_rq_init(&pv->i2c_req.base, pca9557_i2c_write_done);

    pv->i2c_req.transfer = pv->i2c_transfer;
    pv->i2c_transfer[0].data = pv->command;
    pv->i2c_transfer[0].size = 2;
    pv->i2c_transfer[0].type = DEV_I2C_WRITE;
    pv->i2c_req.transfer_count = 1;
    pv->command[0] = REG_OUTPUT;
    pv->command[1] = pv->output_value;

    DEVICE_OP(&pv->i2c_dev, request, &pv->i2c_req);
}

static void pca9557_input_get(
    struct pca9557_private_s *pv)
{
    dev_i2c_ctrl_rq_init(&pv->i2c_req.base, pca9557_i2c_read_done);

    pv->i2c_req.transfer = pv->i2c_transfer;
    pv->i2c_transfer[0].data = pv->command;
    pv->i2c_transfer[0].size = 1;
    pv->i2c_transfer[0].type = DEV_I2C_WRITE;
    pv->i2c_transfer[1].data = &pv->input_value;
    pv->i2c_transfer[1].size = 1;
    pv->i2c_transfer[1].type = DEV_I2C_READ;
    pv->i2c_req.transfer_count = 2;

    pv->command[0] = REG_INPUT;

    DEVICE_OP(&pv->i2c_dev, request, &pv->i2c_req);
}

static void pca9557_req_serve(
    struct device_s *dev,
    struct dev_gpio_rq_s *rq)
{
    struct pca9557_private_s *pv = dev->drv_pv;

    switch (rq->type) {
    case DEV_GPIO_MODE: {
        uint8_t mask = (1 << rq->io_last) - (rq->io_first ? (1 << rq->io_first) : 0);

        switch (rq->mode.mode) {
        case DEV_PIN_PUSHPULL:
            pv->input_mode &= ~mask;
            break;

        case DEV_PIN_DISABLED:
            pv->input_mode |= mask;
            break;

        default:
            rq->error = -ENOTSUP;
            pca9557_req_done(dev, rq);
            return;
        }

        pca9557_mode_update(pv);
        break;
    }

    case DEV_GPIO_SET_OUTPUT: {
        uint8_t mask = (1 << (rq->io_last - rq->io_first + 1)) - 1;
        uint8_t setm = (*rq->output.set_mask & mask) << rq->io_first;
        uint8_t clearm = (*rq->output.clear_mask & mask) << rq->io_first;
        uint8_t set = (setm & clearm) << rq->io_first;
        uint8_t toggle = (setm & ~clearm) << rq->io_first;
        uint8_t clear = (~clearm & ~setm & mask) << rq->io_first;

        pv->output_value ^= toggle;
        pv->output_value &= ~clear;
        pv->output_value |= set;

        pca9557_output_update(pv);
        break;
    }

    case DEV_GPIO_GET_INPUT: {
        pca9557_input_get(pv);
        break;
    }

    case DEV_GPIO_INPUT_IRQ_RANGE:
      rq->error = -ENOTSUP;
      break;
    }
}

static void pca9557_req_next(struct device_s *dev)
{
    struct pca9557_private_s *pv = dev->drv_pv;
    struct dev_gpio_rq_s *rq = dev_gpio_rq_head(&pv->pending);

    if (rq)
        pca9557_req_serve(dev, rq);
}

static DEV_GPIO_REQUEST(pca9557_request)
{
    struct device_s *dev = gpio->dev;
    struct pca9557_private_s *pv = dev->drv_pv;

    if (rq->io_last >= 8) {
        rq->error = -ERANGE;
        return;
    }

    LOCK_SPIN_IRQ_SCOPED(&dev->lock);

    bool_t empty = dev_rq_queue_isempty(&pv->pending);

    dev_gpio_rq_pushback(&pv->pending, rq);
    if (empty)
        pca9557_req_serve(dev, rq);
}

#define pca9557_cancel (dev_gpio_cancel_t*)&dev_driver_notsup_fcn
#define pca9557_set_mode   (dev_gpio_set_mode_t*)dev_driver_notsup_fcn
#define pca9557_set_output (dev_gpio_set_output_t*)dev_driver_notsup_fcn
#define pca9557_get_input  (dev_gpio_get_input_t*)dev_driver_notsup_fcn
#define pca9557_input_irq_range  (dev_gpio_input_irq_range_t*)dev_driver_notsup_fcn
#define pca9557_use dev_use_generic

static DEV_INIT(pca9557_init)
{
    struct pca9557_private_s *pv;


    pv = mem_alloc(sizeof(*pv), mem_scope_sys);
    if (!pv)
        return -ENOMEM;

    memset(pv, 0, sizeof(*pv));
    dev->drv_pv = pv;

    if (device_get_param_dev_accessor(dev, "bus", &pv->i2c_dev, DRIVER_CLASS_I2C)) {
        printk("Bus not found\n");
        goto err_mem;
    }

    if (dev_i2c_res_get_addr(dev, &pv->saddr, 0)) {
        printk("Address not found\n");
        goto err_mem;
    }


    pv->input_mode = 0xff;
    pv->input_value = 0;
    pv->output_value = 0;

    dev_rq_queue_init(&pv->pending);

    pv->i2c_req.pvdata = dev;
    pv->i2c_req.saddr = pv->saddr;

    return 0;

  err_mem:
    mem_free(pv);
    return -1;
}

static DEV_CLEANUP(pca9557_cleanup)
{
    struct pca9557_private_s  *pv = dev->drv_pv;

    device_put_accessor(&pv->i2c_dev.base);

    mem_free(pv);
}

DRIVER_DECLARE(pca9557_drv, 0, "pca9557", pca9557,
               DRIVER_GPIO_METHODS(pca9557));

DRIVER_REGISTER(pca9557_drv);


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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2016
*/

#define LOGK_MODULE_ID "nci_"

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>
#include <hexo/bit.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <mutek/bytecode.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/irq.h>

#include <device/class/timer.h>
#include <device/class/icu.h>
#include <device/class/nfc.h>
#include <device/class/gpio.h>
#if defined(CONFIG_DRIVER_NFC_NCI_SPI)
# include <device/class/spi.h>
#elif defined(CONFIG_DRIVER_NFC_NCI_I2C)
# include <device/class/i2c.h>
#endif

#include "nci_regs.h"
#if defined(CONFIG_DRIVER_NFC_NCI_SPI)
# include "nci_spi.o.h"
#elif defined(CONFIG_DRIVER_NFC_NCI_I2C)
# include "nci_i2c.o.h"
#endif
#include "nci_app.o.h"

enum nci_state_e
{
  NCI_IDLE,
  NCI_RUNNING,
  NCI_WAIT_DELAY,
  NCI_WAIT_IRQ_DELAY,
  NCI_WAIT_BUS,
  NCI_WAIT_GPIO,
  NCI_WAIT_RQ,
};

struct nci_info_s
{
  uint8_t nfcid[10];
  uint8_t version;
};

struct nci_context_s
{
  struct device_gpio_s *gpio;
#if defined(CONFIG_DRIVER_NFC_NCI_SPI)
  struct device_spi_ctrl_s spi;
  struct dev_spi_ctrl_bytecode_rq_s spi_rq;
#elif defined(CONFIG_DRIVER_NFC_NCI_I2C)
  struct device_i2c_ctrl_s i2c;
  struct dev_i2c_ctrl_bytecode_rq_s i2c_rq;
#endif
  struct dev_irq_src_s irq_ep;
  struct device_timer_s *timer;
  struct dev_timer_rq_s timer_rq;
  struct bc_context_s vm;
  struct dev_gpio_rq_s gpio_rq;
  struct kroutine_s runner;

  dev_timer_delay_t five_ms;
  
  enum nci_state_e state;
  int8_t readback0;
  bool_t irq_pending;

  dev_request_queue_root_t queue;

  struct nci_info_s info;

  gpio_id_t gpio_id[1];
};

DRIVER_PV(struct nci_context_s);

static
struct device_s *nci_context_device_get(const struct nci_context_s *pv)
{
#if defined(CONFIG_DRIVER_NFC_NCI_SPI)
  return pv->spi_rq.base.base.pvdata;
#elif defined(CONFIG_DRIVER_NFC_NCI_I2C)
  return pv->i2c_rq.base.base.pvdata;
#endif
}

static const void *const nci_reg_op[4] = {
  &nci_reg_read,
  &nci_reg_write,
  &nci_reg_or,
  &nci_reg_andn,
};

static const void *const nci_fifo_op[2] = {
  &nci_fifo_read,
  &nci_fifo_write,
};

__unused__
static const char *const nci_reg_op_str[4] = {
  "get",
  "set",
  "or",
  "andn",
};

__unused__
static const char *const nci_fifo_op_str[2] = {
  "read",
  "write",
};

static DEV_IRQ_SRC_PROCESS(nci_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nci_context_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);
  if (pv->state == NCI_WAIT_IRQ_DELAY) {
    DEVICE_OP(pv->timer, cancel, &pv->timer_rq);
    dprintk(" -> irq while sleeping\n");
    bc_skip(&pv->vm);
    pv->state = NCI_IDLE;
    kroutine_exec(&pv->runner);
  } else {
    pv->irq_pending = 1;
  }
  lock_release(&dev->lock);
}

static KROUTINE_EXEC(nci_runner)
{
  struct nci_context_s *pv = KROUTINE_CONTAINER(kr, *pv, runner);
  struct device_s *dev = nci_context_device_get(pv);
  uint16_t op;
  bool_t can_run = 0;

  LOCK_SPIN_IRQ(&dev->lock);
  if (pv->state == NCI_IDLE) {
    can_run = 1;
    pv->state = NCI_RUNNING;
  }
  LOCK_RELEASE_IRQ(&dev->lock);

  if (!can_run)
    return;

  for (;;) {
    //dprintk("%s run\n", __FUNCTION__);
    op = bc_run(&pv->vm, -1);
    //dprintk("%s op = %04x\n", __FUNCTION__, op);

    if (!bit_get(op, 15)) {
      assert(!op);
      LOCK_SPIN_IRQ_SCOPED(&dev->lock);
      pv->state = NCI_IDLE;
      return;
    }

    switch (bit_get_mask(op, 12, 3)) {
    case 0: { // wait_ms (5ms steps)
      pv->timer_rq.delay = pv->five_ms * bit_get_mask(op, 0, 12);
      dprintk("%s delay %dms: %d\n", __FUNCTION__,
              bit_get_mask(op, 0, 12) * 5, pv->timer_rq.delay);
      pv->timer_rq.deadline = 0;

      LOCK_SPIN_IRQ_SCOPED(&dev->lock);
      DEVICE_OP(pv->timer, cancel, &pv->timer_rq);
      if (DEVICE_OP(pv->timer, request, &pv->timer_rq) == 0)
        pv->state = NCI_WAIT_DELAY;

      if (pv->state == NCI_RUNNING)
        continue;
      return;
    }

    case 1: { // on_irq_timeout (5ms steps)
      pv->timer_rq.deadline = 0;
      pv->timer_rq.delay = pv->five_ms * bit_get_mask(op, 0, 12);
      dprintk("%s delay irq %dms: %d\n", __FUNCTION__,
              bit_get_mask(op, 0, 12) * 5, pv->timer_rq.delay);

      LOCK_SPIN_IRQ_SCOPED(&dev->lock);
      if (bit_get_mask(op, 0, 12) == 0) {
        dprintk(" -> %s\n", pv->irq_pending ? "irq pending cleared" : "no irq pending");
        pv->irq_pending = 0;
      } else if (pv->irq_pending) {
        pv->irq_pending = 0;
        bc_skip(&pv->vm);
        dprintk(" -> pending irq\n");
      } else {
        DEVICE_OP(pv->timer, cancel, &pv->timer_rq);
        if (DEVICE_OP(pv->timer, request, &pv->timer_rq) == 0)
          pv->state = NCI_WAIT_IRQ_DELAY;
      }

      if (pv->state == NCI_RUNNING)
        continue;
      return;
    }

    case 2: { // Reg access
      dprintk("%s reg %s 0x%02x 0x%02x\n", __FUNCTION__,
              nci_reg_op_str[bit_get_mask(op, 10, 2)],
              bit_get_mask(op, 4, 6),
              (uint8_t)bc_get_reg(&pv->vm, bit_get_mask(op, 0, 4)));

      LOCK_SPIN_IRQ_SCOPED(&dev->lock);
      pv->state = NCI_WAIT_BUS;
      pv->readback0 = bit_get_mask(op, 10, 2) ? -1 : bit_get_mask(op, 0, 4);
#if defined(CONFIG_DRIVER_NFC_NCI_SPI)
      dev_spi_bytecode_start(&pv->spi, &pv->spi_rq,
                             nci_reg_op[bit_get_mask(op, 10, 2)],
                             3, bit_get_mask(op, 4, 6),
                             bc_get_reg(&pv->vm, bit_get_mask(op, 0, 4)));
#elif defined(CONFIG_DRIVER_NFC_NCI_I2C)
      dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_rq,
                             nci_reg_op[bit_get_mask(op, 10, 2)],
                             3, bit_get_mask(op, 4, 6),
                             bc_get_reg(&pv->vm, bit_get_mask(op, 0, 4)));
#endif
      return;
    }

    case 3: { // Fifo access
      if (bit_get(op, 9)) {
        if (bit_get(op, 8))
          dprintk("%s fifo write pack [%P]\n", __FUNCTION__,
                  bc_get_bytepack(&pv->vm, bit_get_mask(op, 4, 4)),
                  bit_get_mask(op, 0, 4) + 1);
        else
          dprintk("%s fifo read pack %d\n", __FUNCTION__,
                  bit_get_mask(op, 0, 4) + 1);

        LOCK_SPIN_IRQ_SCOPED(&dev->lock);
        pv->state = NCI_WAIT_BUS;
#if defined(CONFIG_DRIVER_NFC_NCI_SPI)
        dev_spi_bytecode_start(&pv->spi, &pv->spi_rq,
                               nci_fifo_op[bit_get(op, 8)],
                               3,
                               bc_get_bytepack(&pv->vm, bit_get_mask(op, 4, 4)),
                               bit_get_mask(op, 0, 4) + 1);
#elif defined(CONFIG_DRIVER_NFC_NCI_I2C)
        dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_rq,
                               nci_fifo_op[bit_get(op, 8)],
                               3,
                               bc_get_bytepack(&pv->vm, bit_get_mask(op, 4, 4)),
                               bit_get_mask(op, 0, 4) + 1);
#endif
      } else {
        if (bit_get(op, 8))
          dprintk("%s fifo write %p %d [%P]\n", __FUNCTION__,
                  bc_get_reg(&pv->vm, bit_get_mask(op, 4, 4)),
                  bc_get_reg(&pv->vm, bit_get_mask(op, 0, 4)),
                  bc_get_reg(&pv->vm, bit_get_mask(op, 4, 4)),
                  bc_get_reg(&pv->vm, bit_get_mask(op, 0, 4)));
        else
          dprintk("%s fifo read %d\n", __FUNCTION__,
                  bc_get_reg(&pv->vm, bit_get_mask(op, 0, 4)));

        if (bc_get_reg(&pv->vm, bit_get_mask(op, 0, 4)) == 0)
          continue;

        LOCK_SPIN_IRQ_SCOPED(&dev->lock);
        pv->state = NCI_WAIT_BUS;
#if defined(CONFIG_DRIVER_NFC_NCI_SPI)
        dev_spi_bytecode_start(&pv->spi, &pv->spi_rq,
                               nci_fifo_op[bit_get(op, 8)],
                               3,
                               bc_get_reg(&pv->vm, bit_get_mask(op, 4, 4)),
                               bc_get_reg(&pv->vm, bit_get_mask(op, 0, 4)));
#elif defined(CONFIG_DRIVER_NFC_NCI_I2C)
        dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_rq,
                               nci_fifo_op[bit_get(op, 8)],
                               3,
                               bc_get_reg(&pv->vm, bit_get_mask(op, 4, 4)),
                               bc_get_reg(&pv->vm, bit_get_mask(op, 0, 4)));
#endif
      }
      return;
    }

    case 4: // wait_rq
      switch (bit_get_mask(op, 8, 4)) {
      case 0: {
        dprintk("%s wait rq\n", __FUNCTION__);
        LOCK_SPIN_IRQ_SCOPED(&dev->lock);
        if (dev_request_queue_isempty(&pv->queue)) {
          device_sleep_schedule(dev);
          pv->state = NCI_WAIT_RQ;
        }

        if (pv->state == NCI_RUNNING)
          break;
        return;
      }

      case 1: {
        error_t err = -bc_get_reg(&pv->vm, bit_get_mask(op, 0, 4));
        dprintk("%s rq done %d\n", __FUNCTION__, err);
        LOCK_SPIN_IRQ_SCOPED(&dev->lock);
        assert(!dev_request_queue_isempty(&pv->queue));

        struct dev_nfc_rq_s *rq = dev_nfc_rq_s_cast(dev_request_queue_pop(&pv->queue));
        rq->error = -err;
        kroutine_exec(&rq->base.kr);

        break;
      }
      }
      continue;

    case 5: { // if_no_rq
      dprintk("%s rq next then\n", __FUNCTION__);
      LOCK_SPIN_IRQ_SCOPED(&dev->lock);
      if (!dev_request_queue_isempty(&pv->queue)) {
        bc_set_reg(&pv->vm, bit_get_mask(op, 0, 4),
                   (uintptr_t)dev_nfc_rq_s_cast(dev_request_queue_head(&pv->queue)));
        dprintk(" -> ok r%d = %p\n",
                bit_get_mask(op, 0, 4),
                bc_get_reg(&pv->vm, bit_get_mask(op, 0, 4)));
      } else {
        bc_skip(&pv->vm);
      }
      continue;
    }

    case 6: { // Reset
      pv->gpio_rq.io_first
        = pv->gpio_rq.io_last
        = pv->gpio_id[0];
      pv->readback0 = op & 1;

      LOCK_SPIN_IRQ_SCOPED(&dev->lock);
      pv->state = NCI_WAIT_GPIO;
      DEVICE_OP(pv->gpio, request, &pv->gpio_rq);
      return;
    }

    case 7: // print
      dprintk("  print %08x\n", bc_get_reg(&pv->vm, bit_get_mask(op, 0, 4)));
      continue;
    }
  }
}

static KROUTINE_EXEC(nci_gpio_done)
{
  struct nci_context_s *pv = KROUTINE_CONTAINER(kr, *pv, gpio_rq.base.kr);
  struct device_s *dev = nci_context_device_get(pv);

  //dprintk("%s\n", __FUNCTION__);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);
  assert(pv->state == NCI_WAIT_GPIO);
  pv->state = NCI_IDLE;
  kroutine_exec(&pv->runner);
}

#if defined(CONFIG_DRIVER_NFC_NCI_SPI)
static KROUTINE_EXEC(nci_spi_done)
{
  struct nci_context_s *pv = KROUTINE_CONTAINER(kr, *pv, spi_rq.base.base.kr);
  struct device_s *dev = nci_context_device_get(pv);

  //dprintk("%s\n", __FUNCTION__);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);
  assert(pv->state == NCI_WAIT_BUS);

  if (pv->readback0 >= 0) {
    dprintk("  -> r%d = 0x%02x\n", pv->readback0, bc_get_reg(&pv->spi_rq.vm, 0));
    bc_set_reg(&pv->vm, pv->readback0, bc_get_reg(&pv->spi_rq.vm, 0));
    pv->readback0 = -1;
  }

  pv->state = NCI_IDLE;
  kroutine_exec(&pv->runner);
}
#elif defined(CONFIG_DRIVER_NFC_NCI_I2C)
static KROUTINE_EXEC(nci_i2c_done)
{
  struct nci_context_s *pv = KROUTINE_CONTAINER(kr, *pv, i2c_rq.base.base.kr);
  struct device_s *dev = nci_context_device_get(pv);

  //dprintk("%s\n", __FUNCTION__);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);
  assert(pv->state == NCI_WAIT_BUS);

  if (pv->readback0 >= 0) {
    dprintk("  -> r%d = 0x%02x\n", pv->readback0, bc_get_reg(&pv->i2c_rq.vm, 0));
    bc_set_reg(&pv->vm, pv->readback0, bc_get_reg(&pv->i2c_rq.vm, 0));
    pv->readback0 = -1;
  }

  pv->state = NCI_IDLE;
  kroutine_exec(&pv->runner);
}
#endif

static KROUTINE_EXEC(nci_timer_done)
{
  struct nci_context_s *pv = KROUTINE_CONTAINER(kr, *pv, timer_rq.rq.kr);
  struct device_s *dev = nci_context_device_get(pv);

  dprintk("%s\n", __FUNCTION__);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);
  assert(pv->state == NCI_WAIT_IRQ_DELAY || pv->state == NCI_WAIT_DELAY);
  pv->state = NCI_IDLE;
  kroutine_exec(&pv->runner);
}

static DEV_NFC_REQUEST(nci_request)
{
  struct device_s *dev = accessor->dev;
  struct nci_context_s *pv = dev->drv_pv;

  dprintk("%s\n", __FUNCTION__);

  switch (rq1->type) {
  notsup:
    rq1->error = -ENOTSUP;
    kroutine_exec(&rq1->base.kr);

    if (rq2) {
      rq2->error = -ENOTSUP;
      kroutine_exec(&rq2->base.kr);
    }
    return;

  case DEV_NFC_TRANSMIT:
    if (rq2) {
      if (rq2->type != DEV_NFC_RECEIVE)
        goto notsup;

      if (rq2->data.framing != rq1->data.framing)
        goto notsup;

      rq1->error = 0;
      rq2->error = 0;

      rq1->base.drvdata = (void*)(intptr_t)-1;

      LOCK_SPIN_IRQ_SCOPED(&dev->lock);
      dev_request_queue_pushback(&pv->queue, &rq1->base);
      dev_request_queue_pushback(&pv->queue, &rq2->base);

      if (pv->state == NCI_WAIT_RQ) {
        pv->state = NCI_IDLE;
        dprintk("%s wait done\n", __FUNCTION__);
        kroutine_exec(&pv->runner);
      }
      return;
    }
    // Fallthrough

  case DEV_NFC_POWEROFF:
  case DEV_NFC_POWERON:
  case DEV_NFC_HALT:
  case DEV_NFC_REQUEST_FIND:
  case DEV_NFC_WAKEUP_FIND:
  case DEV_NFC_ACTIVATE:
  case DEV_NFC_PCD_CONFIG:
  case DEV_NFC_RECEIVE:
    if (rq2)
      goto notsup;

    rq1->error = 0;
    rq1->base.drvdata = 0;
    LOCK_SPIN_IRQ_SCOPED(&dev->lock);
    dev_request_queue_pushback(&pv->queue, &rq1->base);

    if (pv->state == NCI_WAIT_RQ) {
      pv->state = NCI_IDLE;
      dprintk("%s wait done\n", __FUNCTION__);
      kroutine_exec(&pv->runner);
    }
    return;
  }
}

static DEV_NFC_CANCEL(nci_cancel)
{
  struct device_s *dev = accessor->dev;
  struct nci_context_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  GCT_FOREACH(dev_request_queue, &pv->queue, item,
              if (item == &rq->base) {
                dev_request_queue_remove(&pv->queue, &rq->base);
#ifdef CONFIG_DEVICE_SLEEP
                if (dev_request_queue_isempty(&pv->queue))
                  device_sleep_schedule(dev);
#endif
                return 0;
              });

  return -ENOENT;
}

static DEV_NFC_INFO_GET(nci_info_get)
{
  struct device_s *dev = accessor->dev;
  struct nci_context_s *pv = dev->drv_pv;

  (void)pv;
}

static DEV_USE(nci_use)
{
  switch (op) {
#ifdef CONFIG_DEVICE_SLEEP
  case DEV_USE_SLEEP: {
    struct device_s *dev = param;
    struct nci_context_s *pv = dev->drv_pv;

    // TODO Do something useful
    (void)pv;

    return 0;
  }
#endif

  default:
    return dev_use_generic(param, op);
  }
}

#if defined(CONFIG_DRIVER_NFC_NCI_SPI)
static const struct dev_spi_ctrl_config_s nci_spi_config =
{
  .dirty = 0,
  .ck_mode = DEV_SPI_CK_MODE_0,
  .bit_order = DEV_SPI_MSB_FIRST,
  .miso_pol = DEV_SPI_ACTIVE_HIGH,
  .mosi_pol = DEV_SPI_ACTIVE_HIGH,
  .cs_pol = DEV_SPI_ACTIVE_LOW,
  .bit_rate = 1000000,
  .word_width = 8,
};
#endif

static DEV_INIT(nci_init)
{
  struct nci_context_s *pv;
  error_t err;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  dev_request_queue_init(&pv->queue);

  device_irq_source_init(dev, &pv->irq_ep, 1, &nci_irq);
  err = device_irq_source_link(dev, &pv->irq_ep, 1, -1);
  if (err)
    goto free_pv;

  pv->state = NCI_IDLE;
#if defined(CONFIG_DRIVER_NFC_NCI_SPI)
  err = dev_drv_spi_bytecode_init(dev, &pv->spi_rq, &nci_spi_bytecode,
                                  &nci_spi_config, &pv->spi, &pv->gpio, &pv->timer);
#elif defined(CONFIG_DRIVER_NFC_NCI_I2C)
  err = dev_drv_i2c_bytecode_init(dev, &pv->i2c_rq, &nci_i2c_bytecode,
                                  &pv->i2c, &pv->gpio, &pv->timer);
#endif
  if (err)
    goto free_pv;

  gpio_width_t pin_wmap[1];

  err = device_res_gpio_map(dev, "resetn:1", pv->gpio_id, pin_wmap);
  if (err)
    goto free_bus;

  err = device_gpio_map_set_mode(pv->gpio, pv->gpio_id, pin_wmap, 1, DEV_PIN_PUSHPULL);
  if (err)
    goto free_bus;

  dev_timer_init_sec_ceil(pv->timer, &pv->five_ms, 0, 1, 200);

  pv->gpio_rq.base.pvdata = dev;
  pv->gpio_rq.output.set_mask = (void*)&pv->readback0;
  pv->gpio_rq.output.clear_mask = (void*)&pv->readback0;
  pv->gpio_rq.type = DEV_GPIO_SET_OUTPUT;

  kroutine_init_deferred(&pv->gpio_rq.base.kr, nci_gpio_done);

#if defined(CONFIG_DRIVER_NFC_NCI_SPI)
  pv->spi_rq.base.base.pvdata = dev;
  kroutine_init_deferred(&pv->spi_rq.base.base.kr, &nci_spi_done);
#elif defined(CONFIG_DRIVER_NFC_NCI_I2C)
  pv->i2c_rq.base.base.pvdata = dev;
  kroutine_init_deferred(&pv->i2c_rq.base.base.kr, &nci_i2c_done);
#endif
  kroutine_init_deferred(&pv->timer_rq.rq.kr, nci_timer_done);
  kroutine_init_deferred(&pv->runner, &nci_runner);

  pv->readback0 = -1;

  bc_init(&pv->vm, &nci_app_bytecode);
  bc_set_pc(&pv->vm, &nci_start);
  bc_set_reg(&pv->vm, 8, (uintptr_t)&pv->info);
  kroutine_exec(&pv->runner);

  return 0;

 free_bus:
#if defined(CONFIG_DRIVER_NFC_NCI_SPI)
  dev_drv_spi_bytecode_cleanup(&pv->spi, &pv->spi_rq);
#elif defined(CONFIG_DRIVER_NFC_NCI_I2C)
  dev_drv_i2c_bytecode_cleanup(&pv->i2c, &pv->i2c_rq);
#endif
 free_pv:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(nci_cleanup)
{
  struct nci_context_s *pv = dev->drv_pv;
  bool_t still_busy;

  LOCK_SPIN_IRQ(&dev->lock);
  still_busy = !dev_request_queue_isempty(&pv->queue) || pv->state != NCI_IDLE;
  LOCK_RELEASE_IRQ(&dev->lock);

  if (still_busy)
    return -EBUSY;

#if defined(CONFIG_DRIVER_NFC_NCI_SPI)
  dev_drv_spi_bytecode_cleanup(&pv->spi, &pv->spi_rq);
#elif defined(CONFIG_DRIVER_NFC_NCI_I2C)
  dev_drv_i2c_bytecode_cleanup(&pv->i2c, &pv->i2c_rq);
#endif
  device_irq_source_unlink(dev, &pv->irq_ep, 1);
  dev_request_queue_destroy(&pv->queue);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(nci_drv, 0, "Nci", nci,
               DRIVER_NFC_METHODS(nci));

DRIVER_REGISTER(nci_drv);

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

    Copyright (c) 2017, Nicolas Pouillon <nipo@ssji.net>
*/

#define LOGK_MODULE_ID "ntag"

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/bit.h>
#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>

#include <device/class/mem.h>
#ifdef CONFIG_DRIVER_NTAG_I2C_PASSTHROUGH
# include <device/class/char.h>
#endif
#include <device/class/i2c.h>

#include "ntag_i2c.o.h"

#ifdef CONFIG_DRIVER_NTAG_I2C_PASSTHROUGH
#include "ntag.h"
#endif

struct ntag_private_s
{
  struct device_i2c_ctrl_s i2c;
  struct dev_i2c_ctrl_bytecode_rq_s i2c_rq;
  dev_request_queue_root_t mem_queue;
  struct dev_request_s *rq;
#ifdef CONFIG_DRIVER_NTAG_I2C_PASSTHROUGH
  struct device_timer_s *timer;
  struct dev_timer_rq_s timer_rq;
  struct device_gpio_s *gpio;
  dev_request_queue_root_t char_queue;
  struct dev_irq_src_s irq_ep;
  gpio_id_t gpio_map[1];
  gpio_width_t gpio_wmap[1];
  bool_t must_refresh;
#endif
};

DRIVER_PV(struct ntag_private_s);

#ifdef CONFIG_DRIVER_NTAG_I2C_PASSTHROUGH
# define NTAG_RQ_MEM 0
# define NTAG_RQ_CHAR 1
#endif

static DEV_MEM_INFO(ntag_mem_info)
{
  struct device_s *dev = accessor->dev;
  struct ntag_private_s *pv = dev->drv_pv;

  if (band_index > 0)
    return -ENOENT;

  if (accessor->number > 0)
    return -ENOENT;

  memset(info, 0, sizeof(*info));
  
  info->type = DEV_MEM_FLASH;
  info->flags = 0
    | DEV_MEM_WRITABLE
    ;
  info->map_base = 0;
  info->size = bc_get_reg(&pv->i2c_rq.vm, NTAG_I2C_BCGLOBAL_SIZE) / 16;
  info->page_log2 = 4;

  return 0;
}

static
void ntag_timeout_reset(struct device_s *dev)
{
  struct ntag_private_s *pv = dev->drv_pv;

  if (pv->timer_rq.base.pvdata) {
    logk_debug("Seems timer is active, trying to cancel");
    if (DEVICE_OP(pv->timer, cancel, &pv->timer_rq)) {
      logk_debug("Cancel failed");
      return;
    }
  }
  
  pv->timer_rq.base.pvdata = dev;
  DEVICE_OP(pv->timer, request, &pv->timer_rq);
}
  
static
void ntag_next(struct device_s *dev)
{
  struct ntag_private_s *pv = dev->drv_pv;
  struct dev_request_s *rq;

  logk_trace("%s", __func__);

  if (pv->i2c_rq.base.base.pvdata)
    return;

  while ((rq = dev_request_queue_head(&pv->mem_queue))) {
    struct dev_mem_rq_s *mrq = dev_mem_rq_s_cast(rq);

    uint8_t block = mrq->addr / 16;
    size_t sc_count = 1;
    uint8_t *buffer = mrq->sc_data[0];
    
    while (mrq->sc_data[sc_count] == mrq->sc_data[0] + (sc_count << (mrq->sc_log2 + 4))
           && (sc_count << mrq->sc_log2) < mrq->size)
      sc_count++;

    mrq->addr += sc_count << (mrq->sc_log2 + 4);
    mrq->sc_data += sc_count;
    mrq->size -= sc_count << mrq->sc_log2;

    logk_debug("merged %d contiguous SC buffers, starts at %x", sc_count, block);
    
    if (mrq->type & DEV_MEM_OP_PAGE_WRITE) {
      pv->i2c_rq.base.base.pvdata = dev;
      pv->rq = rq;
      dev_i2c_bytecode_start(
          &pv->i2c, &pv->i2c_rq, &ntag_bc_write,
          NTAG_BC_WRITE_BCARGS(block, buffer, sc_count << mrq->sc_log2));

      return;
    } else if (mrq->type & DEV_MEM_OP_PAGE_READ) {
      pv->i2c_rq.base.base.pvdata = dev;
      pv->rq = rq;
      dev_i2c_bytecode_start(
          &pv->i2c, &pv->i2c_rq, &ntag_bc_read,
          NTAG_BC_READ_BCARGS(block, buffer, sc_count << mrq->sc_log2));

      return;
    } else {
      mrq->error = -EINVAL;
      mrq->size = 0;
      continue;
    }
  }

#ifdef CONFIG_DRIVER_NTAG_I2C_PASSTHROUGH
  if (pv->must_refresh) {
    pv->must_refresh = 0;
    logk_debug("Refreshing state...");
    pv->i2c_rq.base.base.pvdata = dev;
    dev_i2c_bytecode_start(
        &pv->i2c, &pv->i2c_rq, &ntag_bc_passthrough_update,
        NTAG_BC_PASSTHROUGH_UPDATE_BCARGS());

    return;
  }
  
  rq = dev_request_queue_head(&pv->char_queue);
  struct dev_char_rq_s *crq = dev_char_rq_s_cast(rq);
  reg_t nc = bc_get_reg(&pv->i2c_rq.vm, NTAG_I2C_BCGLOBAL_NC);
  reg_t ns = bc_get_reg(&pv->i2c_rq.vm, NTAG_I2C_BCGLOBAL_NS);
  
  if (!(ns & NS_NFC_PRESENT)) {
    if (rq && crq->type & _DEV_CHAR_WRITE) {
      logk_debug("Failing write request...");
      crq->error = -EIO;
      dev_char_rq_remove(&pv->char_queue, crq);
      kroutine_exec(&rq->kr);
    }

    if (nc & (NC_PT_ON | NC_FD_ON_MASK | NC_FD_OFF_MASK)) {
      logk_debug("Disabling passthrough...");
      pv->i2c_rq.base.base.pvdata = dev;
      dev_i2c_bytecode_start(
          &pv->i2c, &pv->i2c_rq, &ntag_bc_passthrough_disable,
          NTAG_BC_PASSTHROUGH_DISABLE_BCARGS());
    }

    return;
  }

  if (!(nc & NC_PT_ON)) {
    logk_debug("Enabling passthrough...");
    pv->i2c_rq.base.base.pvdata = dev;
    dev_i2c_bytecode_start(
      &pv->i2c, &pv->i2c_rq, &ntag_bc_passthrough_enable,
      NTAG_BC_PASSTHROUGH_ENABLE_BCARGS());
    return;
  }

  if (!rq) {
    logk_debug("No request, out");
    ntag_timeout_reset(dev);
    return;
  }
  
  if (ns & NS_NFC_LOCKED && ns & NS_NFC_READY) {
    logk_debug("Locked to NFC with data, out");
    ntag_timeout_reset(dev);
    return;
  }

  if (crq->type & _DEV_CHAR_WRITE) {
    if (nc & NC_PT_TO_I2C) {
      logk_debug("Switching to TX");
      pv->i2c_rq.base.base.pvdata = dev;
      dev_i2c_bytecode_start(
          &pv->i2c, &pv->i2c_rq, &ntag_bc_pt_send_start,
          NTAG_BC_PT_SEND_START_BCARGS());
      return;
    }
  } else {
    if (!(nc & NC_PT_TO_I2C)) {
      logk_debug("Switching to RX");
      pv->i2c_rq.base.base.pvdata = dev;
      dev_i2c_bytecode_start(
          &pv->i2c, &pv->i2c_rq, &ntag_bc_pt_recv_start,
          NTAG_BC_PT_RECV_START_BCARGS());
      return;
    }
  }

  if (crq->type & _DEV_CHAR_WRITE) {
    logk_debug("Switching to TX and sending");
    pv->i2c_rq.base.base.pvdata = dev;
    pv->rq = rq;
    dev_i2c_bytecode_start(
        &pv->i2c, &pv->i2c_rq, &ntag_bc_pt_send,
        NTAG_BC_PT_SEND_BCARGS(crq->data));
    return;
  } else if (ns & NS_I2C_READY) {
    logk_debug("Ready for reading buffer, reading");
    pv->i2c_rq.base.base.pvdata = dev;
    pv->rq = rq;
    dev_i2c_bytecode_start(
        &pv->i2c, &pv->i2c_rq, &ntag_bc_pt_recv,
        NTAG_BC_PT_RECV_BCARGS(crq->data));

    return;
  }
  
  ntag_timeout_reset(dev);

  return;
#endif
}

static
KROUTINE_EXEC(ntag_i2c_done)
{
  struct ntag_private_s *pv  = KROUTINE_CONTAINER(kr, *pv, i2c_rq.base.base.kr);
  struct device_s *dev = pv->i2c_rq.base.base.pvdata;
  
  pv->i2c_rq.base.base.pvdata = NULL;

  logk_debug("%s %d pt nc %02x ns %02x",
             __func__, pv->i2c_rq.error,
             bc_get_reg(&pv->i2c_rq.vm, NTAG_I2C_BCGLOBAL_NC),
             bc_get_reg(&pv->i2c_rq.vm, NTAG_I2C_BCGLOBAL_NS));

  logk_debug("pt %s, %s, %s%s%s%s",
             bc_get_reg(&pv->i2c_rq.vm, NTAG_I2C_BCGLOBAL_NC) & NC_PT_ON ? "on" : "off",
             bc_get_reg(&pv->i2c_rq.vm, NTAG_I2C_BCGLOBAL_NC) & NC_PT_TO_I2C ? "to i2c" : "to NFC",
             bc_get_reg(&pv->i2c_rq.vm, NTAG_I2C_BCGLOBAL_NS) & NS_I2C_LOCKED ? "i2c locked, " : "",
             bc_get_reg(&pv->i2c_rq.vm, NTAG_I2C_BCGLOBAL_NS) & NS_NFC_LOCKED ? "NFC locked, " : "",
             bc_get_reg(&pv->i2c_rq.vm, NTAG_I2C_BCGLOBAL_NS) & NS_I2C_READY ? "i2c ready, " : "",
             bc_get_reg(&pv->i2c_rq.vm, NTAG_I2C_BCGLOBAL_NS) & NS_NFC_READY ? "NFC ready, " : ""
             );
  
  assert(dev);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  struct dev_request_s *rq = pv->rq;

  if (rq) {
#ifdef CONFIG_DRIVER_NTAG_I2C_PASSTHROUGH
    switch ((uintptr_t)(rq->drvdata)) {
    case NTAG_RQ_MEM:
#endif
      dev_mem_rq_s_cast(rq)->err = pv->i2c_rq.error;
      if (pv->i2c_rq.error || dev_mem_rq_s_cast(rq)->size == 0) {
        dev_mem_rq_remove(&pv->mem_queue, rq);
        kroutine_exec(&rq->kr);
        pv->rq = NULL;
      }
#ifdef CONFIG_DRIVER_NTAG_I2C_PASSTHROUGH
      break;

    case NTAG_RQ_CHAR:
      if (pv->i2c_rq.error != -EAGAIN) {
        dev_char_rq_s_cast(rq)->error = pv->i2c_rq.error;
        dev_char_rq_remove(&pv->char_queue, rq);
        kroutine_exec(&rq->kr);
        pv->rq = NULL;
      }
      break;
    }
#endif
  }

  ntag_next(dev);
}

static
DEV_MEM_REQUEST(ntag_mem_request)
{
  struct device_s *dev = accessor->dev;
  struct ntag_private_s *pv = dev->drv_pv;

  if (rq->band_mask != 1
      || rq->type & (DEV_MEM_OP_PARTIAL_READ | DEV_MEM_OP_PARTIAL_WRITE)
      ) {
    rq->error = -ENOTSUP;
    dev_mem_rq_done(rq);
    return;
  }

  if (rq->addr % 16) {
    rq->error = -EINVAL;
    dev_mem_rq_done(rq);
    return;
  }

  if (rq->addr + (rq->size << (4 + rq->sc_log2))
      > bc_get_reg(&pv->i2c_rq.vm, NTAG_I2C_BCGLOBAL_SIZE)) {
    rq->error = -EINVAL;
    dev_mem_rq_done(rq);
    return;
  }
    
  rq->error = 0;
#ifdef CONFIG_DRIVER_NTAG_I2C_PASSTHROUGH
  rq->base.drvdata = (void*)(uintptr_t)NTAG_RQ_MEM;
#endif

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  dev_mem_rq_pushback(&pv->mem_queue, rq);
  ntag_next(dev);
}

#ifdef CONFIG_DRIVER_NTAG_I2C_PASSTHROUGH
static
KROUTINE_EXEC(ntag_timeout)
{
  struct ntag_private_s *pv  = KROUTINE_CONTAINER(kr, *pv, timer_rq.base.kr);
  struct device_s *dev = pv->timer_rq.base.pvdata;

  logk_debug("Timeout");

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  pv->timer_rq.base.pvdata = NULL;
  pv->must_refresh = 1;

  ntag_next(dev);
}

static
DEV_CHAR_REQUEST(ntag_char_request)
{
  struct device_s *dev = accessor->dev;
  struct ntag_private_s *pv = dev->drv_pv;

  if (rq->size != 64
      || rq->type & ~(DEV_CHAR_READ_FRAME | DEV_CHAR_WRITE_FRAME)
      ) {
    rq->error = -ENOTSUP;
    dev_mem_rq_done(rq);
    return;
  }

#ifdef CONFIG_DRIVER_NTAG_I2C_PASSTHROUGH
  rq->base.drvdata = (void*)(uintptr_t)NTAG_RQ_CHAR;
#endif
  
  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  dev_mem_rq_pushback(&pv->char_queue, rq);
  ntag_next(dev);
}

static
DEV_CHAR_CANCEL(ntag_char_cancel)
{
  struct device_s *dev = accessor->dev;
  struct ntag_private_s *pv = dev->drv_pv;
  error_t err = -ENOENT;
  
  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (pv->rq == &rq->base)
    return -EBUSY;
  
  GCT_FOREACH(dev_request_queue, &pv->char_queue, item, {
      if (item != &rq->base)
        GCT_FOREACH_CONTINUE;
      dev_char_rq_remove(&pv->char_queue, item);
      err = 0;
      GCT_FOREACH_BREAK;
    });

  return err;
}

static
DEV_IRQ_SRC_PROCESS(ntag_irq)
{
  struct device_s *dev = ep->base.dev;
  struct ntag_private_s *pv = dev->drv_pv;

  logk_debug("IRQ");

  lock_spin(&dev->lock);

  pv->must_refresh = 1;
  ntag_next(dev);
  
  lock_release(&dev->lock);
}
#endif

#define ntag_use dev_use_generic

static
DEV_INIT(ntag_init)
{
  struct ntag_private_s *pv;
  error_t err;

  logk_trace("%s", __func__);

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

#ifdef CONFIG_DRIVER_NTAG_I2C_PASSTHROUGH
  device_irq_source_init(dev, &pv->irq_ep, 1, &ntag_irq);
  err = device_irq_source_link(dev, &pv->irq_ep, 1, -1);
  if (err)
    goto err_pv;

  dev_rq_queue_init(&pv->char_queue);
#endif
  dev_rq_queue_init(&pv->mem_queue);

  err = dev_drv_i2c_bytecode_init(dev, &pv->i2c_rq, &ntag_i2c_bytecode,
                                  &pv->i2c,
#ifdef CONFIG_DRIVER_NTAG_I2C_PASSTHROUGH
                                    &pv->gpio, &pv->timer
#else
                                    NULL, NULL
#endif
                                    );
  if (err)
    goto err_pv;

#ifdef CONFIG_DRIVER_NTAG_I2C_PASSTHROUGH
  err = device_res_gpio_map(dev, "fd:1", pv->gpio_map, pv->gpio_wmap);
  if (err)
    goto err_pv;

  /* err = device_gpio_map_set_mode(pv->gpio, pv->gpio_map, pv->gpio_wmap, 1, */
  /*                                DEV_PIN_); */
  /* if (err) */
  /*   goto err_pv; */

  pv->i2c_rq.base.base.pvdata = NULL;
  pv->i2c_rq.gpio_map = pv->gpio_map;
  pv->i2c_rq.gpio_wmap = pv->gpio_wmap;

  dev_timer_rq_init(&pv->timer_rq, &ntag_timeout);
  dev_timer_init_sec(pv->timer, &pv->timer_rq.delay, 0, 1, 10);
#endif

  dev_i2c_ctrl_rq_init(&pv->i2c_rq.base, &ntag_i2c_done);
  pv->i2c_rq.base.base.pvdata = dev;
  dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_rq, &ntag_bc_init, NTAG_BC_INIT_BCARGS());

  return 0;

 err_pv:
  mem_free(pv);
  return err;
}

static
DEV_CLEANUP(ntag_cleanup)
{
  logk_trace("%s", __func__);

  struct ntag_private_s *pv = dev->drv_pv;

  if (pv->i2c_rq.base.base.pvdata)
    return -EBUSY;

  dev_drv_i2c_bytecode_cleanup(&pv->i2c, &pv->i2c_rq);
  dev_rq_queue_destroy(&pv->mem_queue);
#ifdef CONFIG_DRIVER_NTAG_I2C_PASSTHROUGH
  dev_rq_queue_destroy(&pv->char_queue);
  device_irq_source_unlink(dev, &pv->irq_ep, 1);
#endif
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(ntag_drv, 0, "NTAG-I2C", ntag,
#ifdef CONFIG_DRIVER_NTAG_I2C_PASSTHROUGH
               DRIVER_CHAR_METHODS(ntag_char),
#endif
                 DRIVER_MEM_METHODS(ntag_mem));

DRIVER_REGISTER(ntag_drv);


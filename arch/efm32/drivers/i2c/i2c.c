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

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2014
    Copyright Vincent DEFILIPPI <vincentdefilippi@gmail.com> (c) 2016
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/i2c.h>
#include <device/class/iomux.h>
#include <device/clock.h>

#include <arch/efm32/i2c.h>

#define EFM32_I2C_IRQ_MASK \
(\
  EFM32_I2C_IEN_ACK     |\
  EFM32_I2C_IEN_RXDATAV |\
  EFM32_I2C_IEN_NACK    |\
  EFM32_I2C_IEN_MSTOP   |\
  EFM32_I2C_IEN_BUSERR  \
)

enum efm32_i2c_state_e
{
  EFM32_I2C_STATE_IDLE,
  EFM32_I2C_STATE_START,
  EFM32_I2C_STATE_WRITE,
  EFM32_I2C_STATE_READ,
  EFM32_I2C_STATE_STOP,
};

DRIVER_PV(struct efm32_i2c_private_s
{
  uintptr_t                       addr;
#ifdef CONFIG_DEVICE_CLOCK
  struct dev_clock_sink_ep_s      clk_ep;
#endif
  struct dev_i2c_ctrl_context_s   i2c_ctrl_ctx;
  struct dev_irq_src_s            irq_ep;
  struct dev_i2c_ctrl_transfer_s  *tr;
  enum efm32_i2c_state_e          state;
  uint32_t                        bitrate;
  uint32_t                        byte_cnt;
});

static void efm32_i2c_start(struct efm32_i2c_private_s *pv);
static void efm32_i2c_write(struct efm32_i2c_private_s *pv);

static void
efm32_i2c_update_rate(struct efm32_i2c_private_s *pv, const struct dev_freq_s *freq)
{
  /* NLOW + NHIGH */
  static const uint32_t clhr = 4 + 4;

  /* FSCL = FHFPERCLK /(((NLOW + NHIGH ) x (DIV + 1)) + 4) */
  uint32_t div = ((freq->num / (pv->bitrate * freq->denom)) - 4) / clhr - 1;
  cpu_mem_write_32(pv->addr + EFM32_I2C_CLKDIV_ADDR, endian_le32(div));
}

static inline bool_t
efm32_i2c_get_txbl(struct efm32_i2c_private_s *pv)
{
  /* Return 1 when the transmit buffer is empty.
     Return 0 when the transmit buffer is full. */
  uint32_t x = cpu_mem_read_32(pv->addr + EFM32_I2C_STATUS_ADDR);
  return !!(endian_le32(x) & EFM32_I2C_STATUS_TXBL);
}

static inline void
efm32_i2c_terminate(struct device_s *dev, struct efm32_i2c_private_s *pv)
{
  kroutine_exec(&pv->tr->kr);
  pv->tr = NULL;
}

static inline void
efm32_i2c_abort(struct device_s *dev, struct efm32_i2c_private_s *pv)
{
  uint32_t cmd = \
                 //EFM32_I2C_CMD_ABORT |
                 EFM32_I2C_CMD_STOP |
                 EFM32_I2C_CMD_CLEARTX |
                 EFM32_I2C_CMD_CLEARPC;

  cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR, endian_le32(cmd));
}

static inline void
efm32_i2c_read_done(struct device_s *dev, struct efm32_i2c_private_s *pv)
{
  uint8_t data = cpu_mem_read_32(pv->addr + EFM32_I2C_RXDATA_ADDR);
  pv->tr->data[pv->byte_cnt] = endian_le32(data);
  pv->byte_cnt++;

  if (pv->byte_cnt < pv->tr->size)
    {
      cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR, endian_le32(EFM32_I2C_CMD_ACK));
      return;
    }

  if ((pv->tr->type & _DEV_I2C_ENDING_MASK) == _DEV_I2C_STOP)
    {
      pv->state = EFM32_I2C_STATE_STOP;
      cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR, endian_le32(EFM32_I2C_CMD_NACK));
      cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR, endian_le32(EFM32_I2C_CMD_STOP));
      return;
    }
  else if ((pv->tr->type & _DEV_I2C_ENDING_MASK) == _DEV_I2C_RESTART)
    {
      pv->state = EFM32_I2C_STATE_IDLE;
      cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR, endian_le32(EFM32_I2C_CMD_NACK));
    }

  efm32_i2c_terminate(dev, pv);
}

static inline void
efm32_i2c_write_done(struct device_s *dev, struct efm32_i2c_private_s *pv)
{
  if (pv->byte_cnt < pv->tr->size)
    {
      efm32_i2c_write(pv);
      return;
    }

  if ((pv->tr->type & _DEV_I2C_ENDING_MASK) == _DEV_I2C_STOP)
    {
      pv->state = EFM32_I2C_STATE_STOP;
      cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR, endian_le32(EFM32_I2C_CMD_STOP));
      return;
    }
  else if ((pv->tr->type & _DEV_I2C_ENDING_MASK) == _DEV_I2C_RESTART)
    {
      pv->state = EFM32_I2C_STATE_IDLE;
    }

  efm32_i2c_terminate(dev, pv);
}

static void
efm32_i2c_start_done(struct device_s *dev, struct efm32_i2c_private_s *pv)
{
  if (!(pv->tr->type & _DEV_I2C_READ_OP))
    {
      pv->state = EFM32_I2C_STATE_WRITE;
      if (pv->tr->size == 0)
        efm32_i2c_write_done(dev, pv);
      else
        efm32_i2c_write(pv);
    }
  else
    pv->state = EFM32_I2C_STATE_READ;
}

static
DEV_IRQ_SRC_PROCESS(efm32_i2c_irq)
{
  struct device_s                  *dev = ep->base.dev;
  struct efm32_i2c_private_s       *pv  = dev->drv_pv;

  lock_spin(&dev->lock);

  while (dev->start_count != 0)
    {
      uint32_t irq = endian_le32(cpu_mem_read_32(pv->addr + EFM32_I2C_IF_ADDR));
      /* Reset interrupts flags */
      cpu_mem_write_32(pv->addr + EFM32_I2C_IFC_ADDR, endian_le32(irq));
      irq &= EFM32_I2C_IRQ_MASK;

      if (!irq || (pv->tr == NULL))
        break;

      uint32_t status = endian_le32(cpu_mem_read_32(pv->addr + EFM32_I2C_STATUS_ADDR));

      if (irq & EFM32_I2C_IF_BUSERR)
        {
          pv->tr->err = -EIO;
          efm32_i2c_abort(dev, pv);
          pv->state = EFM32_I2C_STATE_STOP;
        }
      else if (irq & EFM32_I2C_IF_NACK)
        {
          pv->tr->err = -EAGAIN;
          if (pv->state == EFM32_I2C_STATE_START)
            pv->tr->err = -EHOSTUNREACH;
          efm32_i2c_abort(dev, pv);
          pv->state = EFM32_I2C_STATE_STOP;
        }
      else
        {
          switch (pv->state)
            {
              case EFM32_I2C_STATE_START:
                if (status & EFM32_I2C_IF_ACK)
                  efm32_i2c_start_done(dev, pv);
                break;

              case EFM32_I2C_STATE_WRITE:
                if (status & EFM32_I2C_IF_ACK)
                  efm32_i2c_write_done(dev, pv);
                break;

              case EFM32_I2C_STATE_READ:
                if (status & EFM32_I2C_STATUS_RXDATAV)
                  efm32_i2c_read_done(dev, pv);
                break;

              case EFM32_I2C_STATE_STOP:
                if (irq & EFM32_I2C_IF_MSTOP)
                  {
                    efm32_i2c_terminate(dev, pv);
                    pv->state = EFM32_I2C_STATE_IDLE;
                    dev->start_count &= ~1;
                  }
                break;

              default:
                break;
            }
        }
    }

# ifdef CONFIG_DEVICE_CLOCK_GATING
  if (dev->start_count == 0)
    dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
# endif

  lock_release(&dev->lock);
}

static void
efm32_i2c_write(struct efm32_i2c_private_s *pv)
{
  /* TX buffer must not be full. */
  assert(efm32_i2c_get_txbl(pv));

  while (pv->byte_cnt < pv->tr->size && efm32_i2c_get_txbl(pv))
    {
      uint8_t data = pv->tr->data[pv->byte_cnt];
      cpu_mem_write_32(pv->addr + EFM32_I2C_TXDATA_ADDR, endian_le32(data));
      pv->byte_cnt++;
    }
}

static void
efm32_i2c_start(struct efm32_i2c_private_s *pv)
{
  /* TX buffer must not be full. */
  assert(efm32_i2c_get_txbl(pv));

  cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR, endian_le32(EFM32_I2C_CMD_START));

  uint32_t saddr = (pv->tr->saddr << 1) & 0xFE;
  if (pv->tr->type & _DEV_I2C_READ_OP)
    saddr++;

  cpu_mem_write_32(pv->addr + EFM32_I2C_TXDATA_ADDR, endian_le32(saddr));
}

static
DEV_I2C_CTRL_TRANSFER(efm32_i2c_transfer)
{
  struct device_s               *dev = accessor->dev;
  struct efm32_i2c_private_s    *pv  = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->tr != NULL)
    {
      tr->err = -EBUSY;
      kroutine_exec(&tr->kr);
      goto out;
    }


  dev->start_count |= 1;
# ifdef CONFIG_DEVICE_CLOCK_GATING
  dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER_CLOCK);
# endif

  tr->err = 0;
  if (tr->type == DEV_I2C_RESET)
    {
      if (pv->state != EFM32_I2C_STATE_IDLE)
        efm32_i2c_abort(dev, pv);
      pv->state = EFM32_I2C_STATE_IDLE;
      kroutine_exec(&tr->kr);
      dev->start_count &= ~1;
      goto out;
    }

  pv->tr = tr;
  pv->byte_cnt = 0;

  if (pv->state == EFM32_I2C_STATE_IDLE)
    {
      pv->state = EFM32_I2C_STATE_START;
      efm32_i2c_start(pv);
    }
  else if (!(pv->tr->type & _DEV_I2C_READ_OP) && pv->state == EFM32_I2C_STATE_WRITE)
    {
      pv->state = EFM32_I2C_STATE_WRITE;
      efm32_i2c_write(pv);
    }
  else if (pv->tr->type & _DEV_I2C_READ_OP && pv->state == EFM32_I2C_STATE_READ)
    {
      pv->state = EFM32_I2C_STATE_READ;
      /* Generate the ACK for the previous request */
      cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR, endian_le32(EFM32_I2C_CMD_ACK));
    }
  else
    {
      if (pv->state != EFM32_I2C_STATE_IDLE)
        efm32_i2c_abort(dev, pv);
      pv->state = EFM32_I2C_STATE_IDLE;
      pv->tr->err = -ENOTSUP;
      kroutine_exec(&tr->kr);
      dev->start_count &= ~1;
      goto out;
    }

 out:
  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_USE(efm32_i2c_use)
{
  switch (op)
    {
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
    case DEV_USE_CLOCK_SINK_FREQ_CHANGED: {
      struct dev_clock_notify_s *chg = param;
      struct dev_clock_sink_ep_s *sink = chg->sink;
      struct device_s *dev = sink->dev;
      struct efm32_i2c_private_s *pv = dev->drv_pv;
      efm32_i2c_update_rate(pv, &chg->freq);
      return 0;
    }
#endif

#ifdef CONFIG_DEVICE_CLOCK_GATING
    case DEV_USE_START: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;
      struct efm32_i2c_private_s *pv = dev->drv_pv;
      if (dev->start_count == 0)
        dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER_CLOCK);
      return 0;
    }

    case DEV_USE_STOP: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;
      struct efm32_i2c_private_s *pv = dev->drv_pv;
      if (dev->start_count == 0)
        dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
      return 0;
    }
#endif

    default:
      return dev_use_generic(param, op);
    }
}


static
DEV_INIT(efm32_i2c_init)
{
  struct efm32_i2c_private_s    *pv;
  error_t err;

  /* allocate driver private context. */
  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  struct dev_freq_s freq;
#ifdef CONFIG_DEVICE_CLOCK
  /* enable clock */
  err = dev_drv_clock_init(
    dev, &pv->clk_ep, 0, DEV_CLOCK_EP_FREQ_NOTIFY |
    DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_GATING_SYNC, &freq);
#else
  err = device_get_res_freq(dev, &freq, 0);
#endif
  if (err)
    goto err_mem;

  /* retreive the device base address from device tree. */
  err = device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL);
  if(err)
    goto err_clk;

  err = dev_drv_i2c_ctrl_context_init(dev, &pv->i2c_ctrl_ctx);
  if (err)
    goto err_queue;

  /* Reset Device by disabling controller (bus idle timeout has not effect). */
  cpu_mem_write_32(pv->addr + EFM32_I2C_CTRL_ADDR, 0);

  /* Send ABORT command as specified in reference manual*/
  uint32_t cmd = EFM32_I2C_CMD_ABORT |
                 EFM32_I2C_CMD_CLEARTX |
                 EFM32_I2C_CMD_CLEARPC;

  cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR, endian_le32(cmd));

  /* Disable and clear interrupts */
  cpu_mem_write_32(pv->addr + EFM32_I2C_IEN_ADDR, 0);

  /* setup pinmux */
  iomux_demux_t loc[2];
  err = device_iomux_setup(dev, ",scl ,sda", loc, NULL, NULL);
  if (err)
    goto err_queue;

#if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12)||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG14)
  uint32_t enable = 0;
  uint32_t route = 0;

  if (loc[0] != IOMUX_INVALID_DEMUX)
    {
      enable |= EFM32_I2C_ROUTEPEN_SCLPEN;
      EFM32_I2C_ROUTELOC0_SCLLOC_SET(route, loc[0]);
    }
  if (loc[1] != IOMUX_INVALID_DEMUX)
    {
      enable |= EFM32_I2C_ROUTEPEN_SDAPEN;
      EFM32_I2C_ROUTELOC0_SDALOC_SET(route, loc[1]);
    }

  if (enable == 0) {
    err = -EINVAL;
    goto err_queue;
  }

  cpu_mem_write_32(pv->addr + EFM32_I2C_ROUTELOC0_ADDR, endian_le32(route));
  cpu_mem_write_32(pv->addr + EFM32_I2C_ROUTEPEN_ADDR, endian_le32(enable));
#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFM
  uint32_t route = EFM32_I2C_ROUTE_SCLPEN | EFM32_I2C_ROUTE_SDAPEN;
  EFM32_I2C_ROUTE_LOCATION_SETVAL(route, loc[0]);
  cpu_mem_write_32(pv->addr + EFM32_I2C_ROUTE_ADDR, endian_le32(route));
#else
# error
#endif

  device_irq_source_init(dev, &pv->irq_ep, 1, &efm32_i2c_irq);
  err = device_irq_source_link(dev, &pv->irq_ep, 1, -1);
  if (err)
    goto err_queue;

  err = device_res_get_uint(dev, DEV_RES_I2C_BITRATE, 0, &pv->bitrate, NULL);
  if(err)
    goto err_link;

  efm32_i2c_update_rate(pv, &freq);

  /* Enable controller */
  uint32_t x = EFM32_I2C_CTRL_EN;
  cpu_mem_write_32(pv->addr + EFM32_I2C_CTRL_ADDR, endian_le32(x));

  /* Clear interrupts flags */
  cpu_mem_write_32(pv->addr + EFM32_I2C_IFC_ADDR, endian_le32(EFM32_I2C_IFC_MASK));
  cpu_mem_write_32(pv->addr + EFM32_I2C_IEN_ADDR, endian_le32(EFM32_I2C_IRQ_MASK));

  pv->state = EFM32_I2C_STATE_IDLE;

#ifdef CONFIG_DEVICE_CLOCK_GATING
  dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
#endif

  return 0;

err_link:
  device_irq_source_unlink(dev, &pv->irq_ep, 1);

err_queue:
#ifdef CONFIG_DEVICE_I2C_REQUEST
  dev_drv_i2c_ctrl_context_cleanup(&pv->i2c_ctrl_ctx);
#endif

err_clk:
#ifdef CONFIG_DEVICE_CLOCK
  dev_drv_clock_cleanup(dev, &pv->clk_ep);
#endif

err_mem:
  mem_free(pv);

  return err;
}

static
DEV_CLEANUP(efm32_i2c_cleanup)
{
  struct efm32_i2c_private_s    *pv = dev->drv_pv;

  if (pv->tr)
    return -EBUSY;

#ifdef CONFIG_DEVICE_CLOCK_GATING
  dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER_CLOCK);
#endif

  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  /* disable I2C device and interrupts. */
  cpu_mem_write_32(pv->addr + EFM32_I2C_IEN_ADDR, 0);
  cpu_mem_write_32(pv->addr + EFM32_I2C_CTRL_ADDR, 0);

#ifdef CONFIG_DEVICE_I2C_REQUEST
  dev_drv_i2c_ctrl_context_cleanup(&pv->i2c_ctrl_ctx);
#endif

#ifdef CONFIG_DEVICE_CLOCK
  dev_drv_clock_cleanup(dev, &pv->clk_ep);
#endif

  device_iomux_cleanup(dev);
  mem_free(pv);
  return 0;
}

DRIVER_DECLARE(efm32_i2c_drv, 0, "EFM32 i2c", efm32_i2c,
               DRIVER_I2C_CTRL_METHODS(efm32_i2c));

DRIVER_REGISTER(efm32_i2c_drv);

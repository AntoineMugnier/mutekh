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

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2014
    Copyright Vincent DEFILIPPI <vincentdefilippi@gmail.com> (c) 2016
    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2018
*/

#define LOGK_MODULE_ID "i2cs"

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/error.h>
#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/request.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/i2c_slave.h>
#include <device/class/iomux.h>
#include <device/clock.h>

#include <arch/efm32/i2c.h>

#define IDLE_IRQ_MASK (0                        \
                       | EFM32_I2C_IF_RXDATAV   \
                       | EFM32_I2C_IF_ADDRA     \
                       )

#define TX_IRQ_MASK (0                          \
                     | EFM32_I2C_IF_RSTART      \
                     | EFM32_I2C_IF_SSTOP       \
                     | EFM32_I2C_IF_ACK         \
                     | EFM32_I2C_IF_NACK        \
                     | EFM32_I2C_IF_BUSERR      \
                     | EFM32_I2C_IF_CLTO        \
                     )

#define RX_IRQ_MASK (0                          \
                     | EFM32_I2C_IF_RXDATAV     \
                     | EFM32_I2C_IF_RSTART      \
                     | EFM32_I2C_IF_SSTOP       \
                     | EFM32_I2C_IF_BUSERR      \
                     | EFM32_I2C_IF_CLTO        \
                     )

enum efm32_i2c_slave_state_e
{
  STATE_IDLE,
  STATE_TRANSMITTING,
  STATE_TRANSMITTING_UNDERFLOW,
  STATE_RECEIVING,
  STATE_RECEIVING_WAITING,
  STATE_RECEIVING_BLOCKED,
};

struct efm32_i2c_slave_pv_s
{
  uintptr_t addr;
  enum efm32_i2c_slave_state_e state;
#ifdef CONFIG_DEVICE_CLOCK
  struct dev_clock_sink_ep_s clk_ep;
#endif
  struct dev_irq_src_s irq_ep;
  dev_request_queue_root_t queue;
  struct dev_i2c_slave_rq_s *addr_sel;
  uint8_t timeout;
};

DRIVER_PV(struct efm32_i2c_slave_pv_s);

static
void efm32_i2c_slave_saddr_setup(driver_pv_t *pv)
{
  uint8_t addr = 0x00;
  uint8_t mask = 0x7f;

  if (pv->addr_sel)
    {
      addr = pv->addr_sel->selection.saddr;
      mask = pv->addr_sel->selection.saddr_mask;
    }

  logk_debug("%s addr: %02x mask: %02x", __func__, addr, mask);
  
  cpu_mem_write_32(pv->addr + EFM32_I2C_SADDR_ADDR,
                   endian_le32(EFM32_I2C_SADDR_VAL_SHIFT_VAL(addr)));
  cpu_mem_write_32(pv->addr + EFM32_I2C_SADDRMASK_ADDR,
                   endian_le32(EFM32_I2C_SADDRMASK_VAL_SHIFT_VAL(mask)));
}

static
void efm32_i2c_slave_data_queue_cancel(driver_pv_t *pv)
{
  for (;;)
    {
      struct dev_i2c_slave_rq_s *rq
        = dev_i2c_slave_rq_s_cast(dev_request_queue_pop(&pv->queue));

      logk_debug("%s %p", __func__, rq);

      if (!rq)
        break;

      kroutine_exec(&rq->base.kr);
    }
}

static
void efm32_i2c_slave_rq_end(driver_pv_t *pv,
                            struct dev_i2c_slave_rq_s *rq,
                            error_t err)
{
  rq->error = err;
  dev_request_queue_remove(&pv->queue, &rq->base);
  kroutine_exec(&rq->base.kr);
}

static
void efm32_i2c_slave_idle_setup(driver_pv_t *pv)
{
  logk_debug("%s", __func__);

  pv->state = STATE_IDLE;
  cpu_mem_write_32(pv->addr + EFM32_I2C_IEN_ADDR, endian_le32(IDLE_IRQ_MASK));

  efm32_i2c_slave_saddr_setup(pv);
}

static
void efm32_i2c_slave_tx_setup(driver_pv_t *pv)
{
  logk_debug("%s", __func__);

  pv->state = STATE_TRANSMITTING_UNDERFLOW;
  cpu_mem_write_32(pv->addr + EFM32_I2C_IEN_ADDR, endian_le32(TX_IRQ_MASK));
  pv->timeout = 10;
}

static
void efm32_i2c_slave_rx_setup(driver_pv_t *pv)
{
  logk_debug("%s", __func__);

  pv->state = STATE_RECEIVING_BLOCKED;
  cpu_mem_write_32(pv->addr + EFM32_I2C_IEN_ADDR, endian_le32(RX_IRQ_MASK));
  pv->timeout = 10;
}

  static
void efm32_i2c_slave_rx_byte(driver_pv_t *pv, struct dev_i2c_slave_rq_s *rq)
{
  uint8_t data = endian_le32(cpu_mem_read_32(pv->addr + EFM32_I2C_RXDATA_ADDR));

  pv->timeout = 10;

  *rq->transfer.data = data;
  rq->transfer.data++;
  rq->transfer.size--;

  rq->error = 0;
  
  logk_debug("%s %02x, %d bytes to go, %s at end", __func__,
             data, rq->transfer.size, rq->transfer.end_ack ? "ACK" : "NACK");

  if (rq->transfer.size)
    {
      cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR,
                       endian_le32(EFM32_I2C_CMD_ACK));
    }
  else if (!rq->transfer.end_ack)
    {
      cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR,
                       endian_le32(EFM32_I2C_CMD_NACK));

      efm32_i2c_slave_rq_end(pv, rq, 0);
      efm32_i2c_slave_data_queue_cancel(pv);
      efm32_i2c_slave_idle_setup(pv);
    }
  else
    {
      efm32_i2c_slave_rq_end(pv, rq, 0);

      rq = dev_i2c_slave_rq_s_cast(dev_request_queue_head(&pv->queue));

      if (rq)
          cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR,
                           endian_le32(EFM32_I2C_CMD_ACK));
      else
        pv->state = STATE_RECEIVING_BLOCKED;
    }
}

static
void efm32_i2c_slave_tx_byte(driver_pv_t *pv, struct dev_i2c_slave_rq_s *rq)
{
  uint8_t data = *rq->transfer.data;

  pv->timeout = 10;

  assert(rq->transfer.size);

  rq->error = 0;

  logk_debug("%s %02x", __func__, data);
  
  cpu_mem_write_32(pv->addr + EFM32_I2C_TXDATA_ADDR, endian_le32(data));
  rq->transfer.data++;
  rq->transfer.size--;
}

static
DEV_IRQ_SRC_PROCESS(efm32_i2c_slave_irq)
{
  struct device_s *dev = ep->base.dev;
  DEVICE_PV(pv, dev);

  LOCK_SPIN_SCOPED(&dev->lock);

  while (dev->start_count)
    {
      uint32_t rirq = endian_le32(cpu_mem_read_32(pv->addr + EFM32_I2C_IF_ADDR));
      uint32_t mask = endian_le32(cpu_mem_read_32(pv->addr + EFM32_I2C_IEN_ADDR));
      uint32_t irq = rirq & mask;

      logk_debug("%s state %d irq %04x mask %04x left %04x", __func__, pv->state,
                 rirq, mask, irq);

      /* Reset interrupts flags */
      cpu_mem_write_32(pv->addr + EFM32_I2C_IFC_ADDR, endian_le32(rirq));

      if (!irq)
        return;

      /* End of data phase conditions */
      if (irq & (EFM32_I2C_IF_RSTART | EFM32_I2C_IF_SSTOP))
        {
          logk_debug(" terminated");

          if (pv->state != STATE_IDLE) {
            efm32_i2c_slave_data_queue_cancel(pv);
            efm32_i2c_slave_idle_setup(pv);
          }

          if (!pv->addr_sel)
            dev->start_count &= ~1;

          continue;
        }

      /* Errors */
      if (irq & (EFM32_I2C_IF_BUSERR | EFM32_I2C_IF_ARBLOST))
        {
          logk_debug(" error");

          cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR,
                           endian_le32(EFM32_I2C_CMD_ABORT));

          if (pv->state != STATE_IDLE) {
            efm32_i2c_slave_data_queue_cancel(pv);
            efm32_i2c_slave_idle_setup(pv);
          }

          if (!pv->addr_sel)
            dev->start_count &= ~1;

          continue;
        }

      /* Timeout */
      if (irq & (EFM32_I2C_IF_CLTO))
        {
          logk_debug(" clto");

          if (pv->timeout == 0)
            {
              cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR,
                               endian_le32(EFM32_I2C_CMD_ABORT));
              if (pv->state != STATE_IDLE) {
                efm32_i2c_slave_data_queue_cancel(pv);
                efm32_i2c_slave_idle_setup(pv);
              }

              if (!pv->addr_sel)
                dev->start_count &= ~1;

              continue;
            }
          else
            {
              pv->timeout--;
            }
        }
      
      /* Slave address selection */
      if (irq & EFM32_I2C_IF_ADDRA)
        {
          logk_debug(" addra");

          assert(irq & EFM32_I2C_IF_RXDATAV);
          assert(pv->state == STATE_IDLE);

          struct dev_i2c_slave_rq_s *s = pv->addr_sel;
          uint8_t data = endian_le32(cpu_mem_read_32(pv->addr + EFM32_I2C_RXDATA_ADDR));
          uint8_t addr = data >> 1;
          uint8_t read = data & 1;
          
          if (!s || ((s->selection.saddr ^ addr) & s->selection.saddr_mask))
            {
              cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR,
                               endian_le32(EFM32_I2C_CMD_NACK));
            }
          else
            {
              pv->addr_sel = NULL;
              s->selection.saddr = addr;
              s->selection.read = read;
              s->error = 0;

              kroutine_exec(&s->base.kr);

              if (read)
                {
                  efm32_i2c_slave_tx_setup(pv);

                  cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR,
                                   endian_le32(EFM32_I2C_CMD_ACK));
                }
              else
                {
                  efm32_i2c_slave_rx_setup(pv);
                }
            }

          /* We do not want more data to be popped by rx path */
          irq &= ~EFM32_I2C_IF_RXDATAV;
        }

      /* Data receive path */
      if (irq & EFM32_I2C_IF_RXDATAV)
        {
          logk_debug(" rxdatav");

          assert(pv->state == STATE_RECEIVING);

          struct dev_i2c_slave_rq_s *rq
            = dev_i2c_slave_rq_s_cast(dev_request_queue_head(&pv->queue));

          if (rq)
            efm32_i2c_slave_rx_byte(pv, rq);
          else
            pv->state = STATE_RECEIVING_WAITING;
        }

      /* Data transmit path, acked */
      if (irq & EFM32_I2C_IF_ACK)
        {
          logk_debug(" ack");

          assert(pv->state == STATE_TRANSMITTING);

          struct dev_i2c_slave_rq_s *rq
            = dev_i2c_slave_rq_s_cast(dev_request_queue_head(&pv->queue));

          assert(rq);
          
          if (rq->transfer.size)
            {
              efm32_i2c_slave_tx_byte(pv, rq);
            }
          else
            {
              rq->transfer.end_ack = 1;
              efm32_i2c_slave_rq_end(pv, rq, 0);

              rq = dev_i2c_slave_rq_s_cast(dev_request_queue_head(&pv->queue));
              if (rq)
                efm32_i2c_slave_tx_byte(pv, rq);
              else
                pv->state = STATE_TRANSMITTING_UNDERFLOW;
            }
        }

      /* Data transmit path, nacked */
      if (irq & EFM32_I2C_IF_NACK)
        {
          logk_debug(" nack");

          if (pv->state == STATE_TRANSMITTING)
            {
              struct dev_i2c_slave_rq_s *rq
                = dev_i2c_slave_rq_s_cast(dev_request_queue_head(&pv->queue));

              if (rq)
                {
                  rq->transfer.end_ack = 0;
                  efm32_i2c_slave_rq_end(pv, rq, 0);
                }
              efm32_i2c_slave_data_queue_cancel(pv);
              efm32_i2c_slave_idle_setup(pv);
            }
        }
    }
 
#ifdef CONFIG_DEVICE_CLOCK_GATING
  dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
#endif
}

static
DEV_I2C_SLAVE_REQUEST(efm32_i2c_slave_request)
{
  struct device_s *dev = accessor->dev;
  DEVICE_PV(pv, dev);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  logk_debug("%s state %d rq type %d", __func__, pv->state, rq->type);

  switch (rq->type)
    {
    case DEV_I2C_SLAVE_SELECTION:
      logk_debug(" slave select");

      if (pv->addr_sel)
        {
          logk_debug(" busy");
          rq->error = -ENOTSUP;
          kroutine_exec(&rq->base.kr);
          return;
        }

      pv->addr_sel = rq;

      if (pv->state == STATE_IDLE)
        {
          logk_debug(" idle");
          efm32_i2c_slave_data_queue_cancel(pv);
          efm32_i2c_slave_idle_setup(pv);
        }

      dev->start_count |= 1;

#ifdef CONFIG_DEVICE_CLOCK_GATING
      dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER_CLOCK);
#endif
      break;

    case DEV_I2C_SLAVE_TRANSMIT:
      logk_debug(" transmit");

      switch (pv->state)
        {
        case STATE_TRANSMITTING_UNDERFLOW:
          logk_debug(" unlock underflow");
          pv->state = STATE_TRANSMITTING;
          efm32_i2c_slave_tx_byte(pv, rq);
          /* fallthrough */

        case STATE_TRANSMITTING:
          rq->error = -ECANCELED;
          dev_request_queue_pushback(&pv->queue, &rq->base);
          break;

        default:
          logk_debug(" bad sequencing");
          rq->error = -EINVAL;
          kroutine_exec(&rq->base.kr);
          return;
        }
      break;

    case DEV_I2C_SLAVE_RECEIVE:
      switch (pv->state)
        {
        case STATE_RECEIVING_BLOCKED:
          logk_debug(" unlock blocked");
          pv->state = STATE_RECEIVING;
          rq->error = -ECANCELED;
          dev_request_queue_pushback(&pv->queue, &rq->base);
          cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR,
                           endian_le32(EFM32_I2C_CMD_ACK));
          break;

        case STATE_RECEIVING:
          rq->error = -ECANCELED;
          dev_request_queue_pushback(&pv->queue, &rq->base);
          break;

        case STATE_RECEIVING_WAITING:
          logk_debug(" unlock waiting");
          pv->state = STATE_RECEIVING;
          rq->error = -ECANCELED;
          dev_request_queue_pushback(&pv->queue, &rq->base);
          efm32_i2c_slave_rx_byte(pv, rq);
          break;

        default:
          logk_debug(" bad sequencing");
          rq->error = -EINVAL;
          kroutine_exec(&rq->base.kr);
          return;
        }
      break;

    default:
      logk_debug(" wtf ?");
      rq->error = -ENOTSUP;
      kroutine_exec(&rq->base.kr);
      return;
    }
}

static DEV_USE(efm32_i2c_slave_use)
{
  switch (op)
    {
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
    case DEV_USE_CLOCK_SINK_FREQ_CHANGED: {
      struct dev_clock_notify_s *chg = param;
      struct dev_clock_sink_ep_s *sink = chg->sink;
      struct device_s *dev = sink->dev;
      DEVICE_PV(pv, dev);
# ifdef CONFIG_DEVICE_CLOCK_GATING
      dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER_CLOCK);
# endif
# ifdef CONFIG_DEVICE_CLOCK_GATING
      if (dev->start_count == 0)
        dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
# endif
      return 0;
    }
#endif

#ifdef CONFIG_DEVICE_CLOCK_GATING
    case DEV_USE_START: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;
      DEVICE_PV(pv, dev);
      if (dev->start_count == 0)
        dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER_CLOCK);
      return 0;
    }

    case DEV_USE_STOP: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;
      DEVICE_PV(pv, dev);
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
DEV_INIT(efm32_i2c_slave_init)
{
  struct efm32_i2c_slave_pv_s *pv;

  /* allocate driver private context. */
  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  struct dev_freq_s freq;
#ifdef CONFIG_DEVICE_CLOCK
  /* enable clock */
  if (dev_drv_clock_init(dev, &pv->clk_ep, 0, DEV_CLOCK_EP_FREQ_NOTIFY |
                         DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_GATING_SYNC, &freq))
    goto err_mem;
#else
  if (device_get_res_freq(dev, &freq, 0))
    goto err_mem;
#endif

  /* retreive the device base address from device tree. */
  if(device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_clk;

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
  if (device_iomux_setup(dev, ",scl ,sda", loc, NULL, NULL))
    goto err_clk;

#if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1) ||  \
  (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12)
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

  if (enable == 0)
    goto err_clk;

  cpu_mem_write_32(pv->addr + EFM32_I2C_ROUTELOC0_ADDR, endian_le32(route));
  cpu_mem_write_32(pv->addr + EFM32_I2C_ROUTEPEN_ADDR, endian_le32(enable));
#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFM
  uint32_t route = EFM32_I2C_ROUTE_SCLPEN | EFM32_I2C_ROUTE_SDAPEN;
  EFM32_I2C_ROUTE_LOCATION_SETVAL(route, loc[0]);
  cpu_mem_write_32(pv->addr + EFM32_I2C_ROUTE_ADDR, endian_le32(route));
#else
# error
#endif

  device_irq_source_init(dev, &pv->irq_ep, 1, &efm32_i2c_slave_irq);
  if (device_irq_source_link(dev, &pv->irq_ep, 1, -1))
    goto err_clk;

  /* Enable controller */
  uint32_t x = EFM32_I2C_CTRL_EN
    | EFM32_I2C_CTRL_SLAVE
    | EFM32_I2C_CTRL_CLTO_SHIFT_VAL(1024PCC);
  cpu_mem_write_32(pv->addr + EFM32_I2C_CTRL_ADDR, endian_le32(x));

  /* Clear interrupts flags */
  cpu_mem_write_32(pv->addr + EFM32_I2C_IFC_ADDR, endian_le32(-1));
  cpu_mem_write_32(pv->addr + EFM32_I2C_IEN_ADDR, endian_le32(IDLE_IRQ_MASK));

#ifdef CONFIG_DEVICE_CLOCK_GATING
  dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
#endif

  dev_request_queue_init(&pv->queue);
  pv->addr_sel = NULL;
  
  return 0;

 err_link:
  device_irq_source_unlink(dev, &pv->irq_ep, 1);

 err_clk:
#ifdef CONFIG_DEVICE_CLOCK
  dev_drv_clock_cleanup(dev, &pv->clk_ep);
#endif

 err_mem:
  mem_free(pv);
  return -EINVAL;
}

static
DEV_CLEANUP(efm32_i2c_slave_cleanup)
{
  DEVICE_PV(pv, dev);

  if (pv->addr_sel || !dev_request_queue_isempty(&pv->queue))
    return -EBUSY;

  dev_request_queue_destroy(&pv->queue);

#ifdef CONFIG_DEVICE_CLOCK_GATING
  dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER_CLOCK);
#endif

  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  /* disable I2C device and interrupts. */
  cpu_mem_write_32(pv->addr + EFM32_I2C_IEN_ADDR, 0);
  cpu_mem_write_32(pv->addr + EFM32_I2C_CTRL_ADDR, 0);

#ifdef CONFIG_DEVICE_CLOCK
  dev_drv_clock_cleanup(dev, &pv->clk_ep);
#endif

  device_iomux_cleanup(dev);
  mem_free(pv);
  return 0;
}

DRIVER_DECLARE(efm32_i2c_slave_drv, 0, "EFM32 i2c slave", efm32_i2c_slave,
               DRIVER_I2C_SLAVE_METHODS(efm32_i2c_slave));

DRIVER_REGISTER(efm32_i2c_slave_drv);

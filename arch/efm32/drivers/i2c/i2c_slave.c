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
#define CLTO 100

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

enum i2c_slave_state_e
{
  I2C_SLAVE_IDLE,
  I2C_SLAVE_WAIT_SSEL,
  I2C_SLAVE_WAIT_TX_FIRST,
  I2C_SLAVE_WAIT_TX_OTHER,
  I2C_SLAVE_WAIT_RX_RQ,
  I2C_SLAVE_WAIT_RX_BUS,
};

struct efm32_i2c_slave_pv_s
{
  uintptr_t addr;
#ifdef CONFIG_DEVICE_CLOCK
  struct dev_clock_sink_ep_s clk_ep;
#endif
  struct dev_irq_src_s irq_ep;
  dev_request_queue_root_t queue;
  struct dev_i2c_slave_rq_s *addr_sel;
  uint32_t timeout;
  enum i2c_slave_state_e state;
};

DRIVER_PV(struct efm32_i2c_slave_pv_s);

static
void efm32_i2c_slave_irq_setup(driver_pv_t *pv)
{
  switch (pv->state) {
  case I2C_SLAVE_IDLE:
    cpu_mem_write_32(pv->addr + EFM32_I2C_IEN_ADDR, 0
                     | EFM32_I2C_IF_ADDRA
                     | EFM32_I2C_IF_BITO
                     | EFM32_I2C_IF_SSTOP
                     );
    break;

  case I2C_SLAVE_WAIT_SSEL:
    cpu_mem_write_32(pv->addr + EFM32_I2C_IEN_ADDR, 0
                     | EFM32_I2C_IF_CLTO
                     | EFM32_I2C_IF_BITO
                     | EFM32_I2C_IF_SSTOP
                     );
    break;
    
  case I2C_SLAVE_WAIT_RX_RQ:
  case I2C_SLAVE_WAIT_TX_FIRST:
    cpu_mem_write_32(pv->addr + EFM32_I2C_IEN_ADDR, 0
                     | EFM32_I2C_IF_SSTOP
                     | EFM32_I2C_IF_RSTART
                     | EFM32_I2C_IF_ARBLOST
                     | EFM32_I2C_IF_CLTO
                     | EFM32_I2C_IF_BUSERR
                     );
    break;
    
  case I2C_SLAVE_WAIT_TX_OTHER:
  case I2C_SLAVE_WAIT_RX_BUS:
    cpu_mem_write_32(pv->addr + EFM32_I2C_IEN_ADDR, 0
                     | EFM32_I2C_IF_SSTOP
                     | EFM32_I2C_IF_RSTART
                     | EFM32_I2C_IF_ARBLOST
                     | EFM32_I2C_IF_BUSHOLD
                     | EFM32_I2C_IF_CLTO
                     | EFM32_I2C_IF_BUSERR
                     );
    break;
  }
}

static
void efm32_i2c_slave_saddr_setup(driver_pv_t *pv)
{
  uint32_t addr = 0x00;
  uint32_t mask = 0x7f;

  if (pv->addr_sel)
    {
      addr = pv->addr_sel->selection.saddr;
      mask = pv->addr_sel->selection.saddr_mask;
    }

  logk_debug("%s addr: %02x mask: %02x st %02x", __func__, addr, mask,
             cpu_mem_read_32(pv->addr + EFM32_I2C_STATE_ADDR));

  cpu_mem_write_32(pv->addr + EFM32_I2C_SADDRMASK_ADDR,
                   EFM32_I2C_SADDRMASK_VAL_SHIFT_VAL(mask));
  cpu_mem_write_32(pv->addr + EFM32_I2C_SADDR_ADDR,
                   EFM32_I2C_SADDR_VAL_SHIFT_VAL(addr));

  switch (EFM32_I2C_STATE_STATE_GET(cpu_mem_read_32(pv->addr + EFM32_I2C_STATE_ADDR)))
    {
    case EFM32_I2C_STATE_STATE_IDLE:
    case EFM32_I2C_STATE_STATE_WAIT:
      cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR, 0
                       | EFM32_I2C_CMD_ABORT
                       | EFM32_I2C_CMD_CLEARTX
                       | EFM32_I2C_CMD_CLEARPC);
      cpu_mem_write_32(pv->addr + EFM32_I2C_SADDRMASK_ADDR,
                       EFM32_I2C_SADDRMASK_VAL_SHIFT_VAL(mask));
      cpu_mem_write_32(pv->addr + EFM32_I2C_SADDR_ADDR,
                       EFM32_I2C_SADDR_VAL_SHIFT_VAL(addr));

      break;
    default:
      break;
    }


  logk_debug(" readback %02x/%02x",
             cpu_mem_read_32(pv->addr + EFM32_I2C_SADDR_ADDR),
             cpu_mem_read_32(pv->addr + EFM32_I2C_SADDRMASK_ADDR));

  efm32_i2c_slave_irq_setup(pv);
}

static
void efm32_i2c_slave_data_queue_cancel(driver_pv_t *pv)
{
  for (;;)
    {
      struct dev_i2c_slave_rq_s *rq
        = dev_i2c_slave_rq_pop(&pv->queue);

      logk_debug("%s %p", __func__, rq);

      if (!rq)
        break;

      dev_i2c_slave_rq_done(rq);
    }
}

static
void efm32_i2c_slave_rq_end(driver_pv_t *pv,
                            struct dev_i2c_slave_rq_s *rq,
                            error_t err)
{
  rq->error = err;
  dev_i2c_slave_rq_remove(&pv->queue, rq);
  dev_i2c_slave_rq_done(rq);
}

static
void efm32_i2c_slave_ssel_done(driver_pv_t *pv)
{
  struct dev_i2c_slave_rq_s *s = pv->addr_sel;
  uint8_t data = cpu_mem_read_32(pv->addr + EFM32_I2C_RXDATA_ADDR);
  uint8_t addr = data >> 1;
  uint8_t read = data & 1;

  logk_debug(" %02x addra %02x for %02x/%02x, I2C %s",
             data, addr,
             cpu_mem_read_32(pv->addr + EFM32_I2C_SADDR_ADDR),
             cpu_mem_read_32(pv->addr + EFM32_I2C_SADDRMASK_ADDR),
             read ? "Read" : "Write");

  if ((s->selection.saddr ^ addr) & s->selection.saddr_mask)
    {
      uint32_t state = cpu_mem_read_32(pv->addr + EFM32_I2C_STATE_ADDR);
      bool_t held = !!(state & EFM32_I2C_STATE_BUSHOLD);

      // Not actually matching
      logk_debug(" Not for us, %s", held ? "held" : "not held");

      if (held)
        cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR, EFM32_I2C_CMD_NACK);

      pv->state = I2C_SLAVE_IDLE;
      efm32_i2c_slave_irq_setup(pv);
      return;
    }

  pv->addr_sel = NULL;
  pv->timeout = CLTO;

  s->selection.saddr = addr;
  s->selection.read = read;
  s->error = 0;

  if (read)
    pv->state = I2C_SLAVE_WAIT_TX_FIRST;
  else
    pv->state = I2C_SLAVE_WAIT_RX_RQ;
  
  dev_i2c_slave_rq_done(s);
  efm32_i2c_slave_irq_setup(pv);
}

static
void efm32_i2c_slave_addr_selection_handle(driver_pv_t *pv)
{
  struct dev_i2c_slave_rq_s *s = pv->addr_sel;

  if (pv->state == I2C_SLAVE_WAIT_TX_FIRST)
    return;

  if (!s)
    {
      logk_debug(" no ssel req but matches last, wait");
      pv->state = I2C_SLAVE_WAIT_SSEL;
      efm32_i2c_slave_irq_setup(pv);
      return;
    }

  efm32_i2c_slave_ssel_done(pv);
}

static
void efm32_slave_data_put(driver_pv_t *pv, struct dev_i2c_slave_rq_s *rq)
{
  uint8_t data = *rq->transfer.data;
  
  pv->timeout = CLTO;
  rq->error = 0;

  logk_debug("%s %02x", __func__, data);

  cpu_mem_write_32(pv->addr + EFM32_I2C_TXDATA_ADDR, data);
  rq->transfer.data++;
  rq->transfer.size--;
}

static
void efm32_i2c_slave_data_next(driver_pv_t *pv, struct dev_i2c_slave_rq_s *rq)
{
  uint32_t state = cpu_mem_read_32(pv->addr + EFM32_I2C_STATE_ADDR);

  if (!(state & EFM32_I2C_STATE_BUSHOLD))
    {
      logk_debug("Bus not held");
      return;
    }

  if (state & EFM32_I2C_STATE_TRANSMITTER) {
    if (pv->state != I2C_SLAVE_WAIT_TX_FIRST && pv->state != I2C_SLAVE_WAIT_TX_OTHER)
      {
        logk_debug("not wait tx");
        goto cancel_all;
      }

    logk_debug("%s tx %d %02x", __func__, pv->state, state);

    switch (EFM32_I2C_STATE_STATE_GET(state)) {
    case EFM32_I2C_STATE_STATE_ADDR:
      // State #73
      // 1: load tx data to tx buffer
      // 2: ack saddr byte
      // Next interaction is after (N)ACK, or after a restart
      pv->state = I2C_SLAVE_WAIT_TX_OTHER;
      efm32_slave_data_put(pv, rq);
      cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR, EFM32_I2C_CMD_ACK);
      break;

    case EFM32_I2C_STATE_STATE_DATAACK:
      // State #D5 or #DD
      // After reception of ACK by master
      if (state & EFM32_I2C_STATE_NACKED)
        {
          // Supposedly nothing to do, slave will go idle.
          rq->transfer.end_ack = 0;
          efm32_i2c_slave_rq_end(pv, rq, 0);
          break;
        }

      if (!rq->transfer.size)
        {
          // OK, transfer is finished, give it back to user, signal we
          // got an ACK, and therefore expect more data
          rq->transfer.end_ack = 1;
          efm32_i2c_slave_rq_end(pv, rq, 0);

          // Maybe subsequent transfer is here already
          rq = dev_i2c_slave_rq_head(&pv->queue);
          if (!rq)
            break;
        }

      // Either first TX byte or one following an ACK.
      // Just send next byte
      // Next interaction is after (N)ACK, or after a restart
      efm32_slave_data_put(pv, rq);
      break;

    default:
      // All other cases are broken
      logk_debug("Bad hw state for transmitter: %02x", state);
      goto cancel_all;
    }
  } else {
    if (pv->state != I2C_SLAVE_WAIT_RX_RQ && pv->state != I2C_SLAVE_WAIT_RX_BUS)
      {
        logk_debug("not wait rx");
        goto cancel_all;
      }

    logk_debug("%s rx %d %02x", __func__, pv->state, state);
    
    switch (EFM32_I2C_STATE_STATE_GET(state)) {
    case EFM32_I2C_STATE_STATE_DATA:
      // State #B1

      // We just received a byte. Take it and store it if not done yet.
      if (pv->state == I2C_SLAVE_WAIT_RX_BUS
          && rq->transfer.size) {
        uint32_t data = cpu_mem_read_32(pv->addr + EFM32_I2C_RXDATA_ADDR);

        pv->timeout = CLTO;

        *rq->transfer.data = data;
        rq->transfer.data++;
        rq->transfer.size--;

        rq->error = 0;

        pv->state = I2C_SLAVE_WAIT_RX_RQ;
      }

      if (pv->state != I2C_SLAVE_WAIT_RX_RQ)
        break;

      // fallthrough
    case EFM32_I2C_STATE_STATE_ADDR:
      // State #71

      // We did not acknowledge the previous transfer yet. We will
      // only do if user posts a request with non-empty buffer and
      // acknowledge.

      assert(pv->state == I2C_SLAVE_WAIT_RX_RQ);

      if (rq->transfer.size) {
        cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR, EFM32_I2C_CMD_ACK);
        pv->state = I2C_SLAVE_WAIT_RX_BUS;

        // Now we wait for byte transfer to happen.

        break;
      }

      // Transfer is over

      // Either we got to the end of transfer, or end of transaction.
      if (rq->transfer.end_ack)
        {
          // End of transfer, expect other transactions to come back.
          efm32_i2c_slave_rq_end(pv, rq, 0);

          break;
        }

      cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR, EFM32_I2C_CMD_NACK);

      // Dont expect anything else to happen.
      pv->state = I2C_SLAVE_IDLE;
      // Terminate this one cleanly, not others
      efm32_i2c_slave_rq_end(pv, rq, 0);
      logk_debug("Cancel rest of queue after RX nack");
      goto cancel_all;

    default:
      logk_debug("Bad hw state for receiver: %02x", state);
      goto cancel_all;
    }
  }

  return;

 cancel_all:
  pv->state = I2C_SLAVE_IDLE;
  efm32_i2c_slave_data_queue_cancel(pv);
}

static
DEV_IRQ_SRC_PROCESS(efm32_i2c_slave_irq)
{
  struct device_s *dev = ep->base.dev;
  DEVICE_PV(pv, dev);

  LOCK_SPIN_SCOPED(&dev->lock);

  while (dev->start_count)
    {
      uint32_t rirq = cpu_mem_read_32(pv->addr + EFM32_I2C_IF_ADDR);
      uint32_t mask = cpu_mem_read_32(pv->addr + EFM32_I2C_IEN_ADDR);
      uint32_t irq_left = rirq & mask;
      uint32_t state = cpu_mem_read_32(pv->addr + EFM32_I2C_STATE_ADDR);

      cpu_mem_write_32(pv->addr + EFM32_I2C_IFC_ADDR, rirq);

      if (!irq_left)
        return;

      if (irq_left & EFM32_I2C_IF_CLTO && state & EFM32_I2C_STATE_BUSHOLD) {
        if (!pv->timeout)
          {
            logk_debug(" Clock low timeout");
            continue;
            cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR, EFM32_I2C_CMD_ABORT);
            pv->state = I2C_SLAVE_IDLE;
            efm32_i2c_slave_data_queue_cancel(pv);
          }
        else
          pv->timeout--;

        continue;
      }

      logk_debug("%s irq %04x -> %04x hw_st %02x/%02x",
                 __func__, rirq, irq_left, state,
                 cpu_mem_read_32(pv->addr + EFM32_I2C_STATUS_ADDR));

      if (irq_left & EFM32_I2C_IF_BUSERR && state & EFM32_I2C_STATE_TRANSMITTER) {
        logk_debug(" Bus error");

        cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR, EFM32_I2C_CMD_ABORT);
        pv->state = I2C_SLAVE_IDLE;
        efm32_i2c_slave_data_queue_cancel(pv);
        continue;
      }
      
      switch (EFM32_I2C_STATE_STATE_GET(state)) {
      case EFM32_I2C_STATE_STATE_IDLE:
        // TODO: unmask me
        if (irq_left & EFM32_I2C_IF_BITO) {
          logk_debug(" Bito");
        }

        efm32_i2c_slave_data_queue_cancel(pv);
        // efm32_i2c_slave_saddr_setup(pv);
        pv->state = I2C_SLAVE_IDLE;
        break;

      case EFM32_I2C_STATE_STATE_WAIT:
      case EFM32_I2C_STATE_STATE_START:
        break;

      case EFM32_I2C_STATE_STATE_ADDR:
        logk_debug(" addr");
        if (pv->state == I2C_SLAVE_WAIT_TX_FIRST)
          goto data;
        
        efm32_i2c_slave_addr_selection_handle(pv);
        break;

      case EFM32_I2C_STATE_STATE_ADDRACK:
        logk_debug(" addrack");
        break;

      case EFM32_I2C_STATE_STATE_DATAACK:
      case EFM32_I2C_STATE_STATE_DATA:
      data: {
        struct dev_i2c_slave_rq_s *rq = dev_i2c_slave_rq_head(&pv->queue);

        if (!rq)
          return;

        switch (pv->state) {
        case I2C_SLAVE_IDLE:
        case I2C_SLAVE_WAIT_SSEL:
          // Should not happen ?
          cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR, EFM32_I2C_CMD_ABORT);
          efm32_i2c_slave_data_queue_cancel(pv);
          break;

        case I2C_SLAVE_WAIT_RX_RQ:
        case I2C_SLAVE_WAIT_RX_BUS:
          if (rq->type != DEV_I2C_SLAVE_RECEIVE) {
            logk_debug(" not a rx\n");
            rq->error = -EIO;
            dev_i2c_slave_rq_done(rq);
            continue;
          }
          break;

        case I2C_SLAVE_WAIT_TX_FIRST:
        case I2C_SLAVE_WAIT_TX_OTHER:
          if (rq->type != DEV_I2C_SLAVE_TRANSMIT) {
            logk_debug(" not a tx\n");
            rq->error = -EIO;
            dev_i2c_slave_rq_done(rq);
            continue;
          }
          break;
        }

        efm32_i2c_slave_data_next(pv, rq);
        efm32_i2c_slave_irq_setup(pv);
        
        break;
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

  uint32_t state = cpu_mem_read_32(pv->addr + EFM32_I2C_STATE_ADDR);

  logk_debug("%s st %d hwst %02x", __func__, pv->state, state);

  switch (rq->type)
    {
    case DEV_I2C_SLAVE_SELECTION:
      logk_debug(" slave select");
      
      if (pv->addr_sel)
        {
          logk_debug(" busy");
          rq->error = -ENOTSUP;
          dev_i2c_slave_rq_done(rq);
          return;
        }

      pv->addr_sel = rq;
      dev->start_count |= 1;
#ifdef CONFIG_DEVICE_CLOCK_GATING
      dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER_CLOCK);
#endif
      
      switch (pv->state) {
      default:
        break;

      case I2C_SLAVE_IDLE:
        logk_debug(" ssel idle");
        efm32_i2c_slave_saddr_setup(pv);
        break;

      case I2C_SLAVE_WAIT_SSEL:
        logk_debug(" ssel waiting");
        efm32_i2c_slave_ssel_done(pv);
        break;
      }
      return;

    case DEV_I2C_SLAVE_TRANSMIT:
      logk_debug(" transmit");
      if (pv->state != I2C_SLAVE_WAIT_TX_FIRST && pv->state != I2C_SLAVE_WAIT_TX_OTHER)
        goto bad_state;
      goto data_io;

    case DEV_I2C_SLAVE_RECEIVE:
      logk_debug(" receive");
      if (pv->state != I2C_SLAVE_WAIT_RX_RQ
          && pv->state != I2C_SLAVE_WAIT_RX_BUS)
        goto bad_state;

    data_io:
      switch (EFM32_I2C_STATE_STATE_GET(state))
        {
        case EFM32_I2C_STATE_STATE_DATAACK:
        case EFM32_I2C_STATE_STATE_ADDR:
        case EFM32_I2C_STATE_STATE_DATA: {
          rq->error = -ECANCELED;
          bool_t empty = dev_rq_queue_isempty(&pv->queue);
          dev_i2c_slave_rq_pushback(&pv->queue, rq);
          if (empty)
            {
              efm32_i2c_slave_data_next(pv, rq);
              efm32_i2c_slave_irq_setup(pv);
            }

          return;
        }

        default:
          logk_debug(" bad sequencing, hwst %02x", state);
          rq->error = -EINVAL;
          dev_i2c_slave_rq_done(rq);
          return;
        }
      goto bad_state;

    bad_state:
      logk_debug(" bad state");
      rq->error = -EIO;
      dev_i2c_slave_rq_done(rq);
      return;

    default:
      break;
    }

  logk_debug(" wtf ?");
  rq->error = -ENOTSUP;
  dev_i2c_slave_rq_done(rq);
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
      dev_clock_sink_gate(&pv->clk_ep, dev->start_count
                          ? DEV_CLOCK_EP_POWER_CLOCK
                          : DEV_CLOCK_EP_POWER);
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

  cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR, cmd);

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

  cpu_mem_write_32(pv->addr + EFM32_I2C_ROUTELOC0_ADDR, route);
  cpu_mem_write_32(pv->addr + EFM32_I2C_ROUTEPEN_ADDR, enable);
#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFM
  uint32_t route = EFM32_I2C_ROUTE_SCLPEN | EFM32_I2C_ROUTE_SDAPEN;
  EFM32_I2C_ROUTE_LOCATION_SETVAL(route, loc[0]);
  cpu_mem_write_32(pv->addr + EFM32_I2C_ROUTE_ADDR, route);
#else
# error
#endif

  device_irq_source_init(dev, &pv->irq_ep, 1, &efm32_i2c_slave_irq);
  if (device_irq_source_link(dev, &pv->irq_ep, 1, -1))
    goto err_clk;

  cpu_mem_write_32(pv->addr + EFM32_I2C_CLKDIV_ADDR, 0x20);
  cpu_mem_write_32(pv->addr + EFM32_I2C_IEN_ADDR, 0);
  cpu_mem_write_32(pv->addr + EFM32_I2C_IFC_ADDR, -1);

  cpu_mem_write_32(pv->addr + EFM32_I2C_CTRL_ADDR, 0
                   | EFM32_I2C_CTRL_EN
                   | EFM32_I2C_CTRL_SLAVE
                   | EFM32_I2C_CTRL_BITO_SHIFT_VAL(160PCC)
                   | EFM32_I2C_CTRL_GIBITO_SHIFT_VAL(NEW_TRANSFER)
                   | EFM32_I2C_CTRL_CLHR_SHIFT_VAL(ASYMMETRIC)
                   | (5 << EFM32_I2C_CTRL_CLTO_SHIFT)
                   );

  cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR, EFM32_I2C_CMD_ABORT);

#ifdef CONFIG_DEVICE_CLOCK_GATING
  dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
#endif

  dev_rq_queue_init(&pv->queue);
  pv->addr_sel = NULL;
  pv->state = I2C_SLAVE_IDLE;

  efm32_i2c_slave_saddr_setup(pv);

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

  if (pv->addr_sel || !dev_rq_queue_isempty(&pv->queue))
    return -EBUSY;

  dev_rq_queue_destroy(&pv->queue);

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

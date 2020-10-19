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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2020
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

#include <arch/nrf5x/twis.h>
#include <arch/nrf5x/gpio.h>

enum pin_id
{
  PIN_SCL,
  PIN_SDA,
  PIN_COUNT,
};

enum twis_state
{
  STATE_IDLE,
  STATE_SELECT_READ,
  STATE_SELECT_WRITE,
  STATE_TX_WAIT,
  STATE_RX_WAIT,
  STATE_TX,
  STATE_RX,
};

struct nrf5x_i2cs_pv_s
{
  uintptr_t addr;
  struct dev_irq_src_s irq_ep;
  struct dev_i2c_slave_rq_s *addr_sel;
  struct dev_i2c_slave_rq_s *tx;
  struct dev_i2c_slave_rq_s *rx;
  iomux_io_id_t pin[PIN_COUNT];
  enum twis_state state;
};

DRIVER_PV(struct nrf5x_i2cs_pv_s);

static
void nrf5x_i2cs_data_cancel(struct nrf5x_i2cs_pv_s *pv)
{
  if (pv->tx) {
    uint32_t amount = nrf_reg_get(pv->addr, NRF_TWIS_TXD_AMOUNT);
    logk_trace("TX rq done, %d bytes transmitted", amount);
    pv->tx->transfer.data += amount;
    pv->tx->transfer.size -= amount;
    pv->tx->transfer.end_ack = 1;
    dev_i2c_slave_rq_done(pv->tx);
    pv->tx = NULL;
  }

  if (pv->rx) {
    uint32_t amount = nrf_reg_get(pv->addr, NRF_TWIS_RXD_AMOUNT);
    logk_trace("RX rq done, %d bytes received", amount);
    pv->rx->transfer.data += amount;
    pv->rx->transfer.size -= amount;
    dev_i2c_slave_rq_done(pv->rx);
    pv->rx = NULL;
  }
}

static
DEV_IRQ_SRC_PROCESS(nrf5x_i2c_slave_irq)
{
  struct device_s *dev = ep->base.dev;
  bool_t once;
  DEVICE_PV(pv, dev);

  LOCK_SPIN_SCOPED(&dev->lock);

  logk_trace("IRQ");

  nrf_it_set_mask(pv->addr, 0
                  | bit(NRF_TWIS_ERROR)
                  | bit(NRF_TWIS_STOP)
                  | bit(NRF_TWIS_READ)
                  | bit(NRF_TWIS_WRITE)
                  | bit(NRF_TWIS_TXSTARTED)
                  | bit(NRF_TWIS_RXSTARTED)
                  );
  
  do {
    once = 0;

    if (nrf_event_check(pv->addr, NRF_TWIS_RXSTARTED)) {
      once = 1;
      nrf_event_clear(pv->addr, NRF_TWIS_RXSTARTED);

      pv->state = STATE_RX;
      logk_trace("RX Started RX: %p", pv->rx);
      nrf_it_set_mask(pv->addr, 0
                      | bit(NRF_TWIS_ERROR)
                      | bit(NRF_TWIS_STOP)
                      | bit(NRF_TWIS_READ)
                      | bit(NRF_TWIS_WRITE)
                      );

      if (pv->rx)
        pv->rx->error = 0;
    }
  
    if (nrf_event_check(pv->addr, NRF_TWIS_TXSTARTED)) {
      once = 1;
      nrf_event_clear(pv->addr, NRF_TWIS_TXSTARTED);

      pv->state = STATE_TX;
      logk_trace("TX Started TX: %p", pv->tx);
      nrf_it_set_mask(pv->addr, 0
                      | bit(NRF_TWIS_ERROR)
                      | bit(NRF_TWIS_STOP)
                      | bit(NRF_TWIS_READ)
                      | bit(NRF_TWIS_WRITE)
                      );

      if (pv->tx)
        pv->tx->error = 0;
    }
  
    if (nrf_event_check(pv->addr, NRF_TWIS_READ)) {
      once = 1;
      nrf_event_clear(pv->addr, NRF_TWIS_READ);
      nrf_event_clear(pv->addr, NRF_TWIS_STOPPED);
      nrf_event_clear(pv->addr, NRF_TWIS_ERROR);

      nrf5x_i2cs_data_cancel(pv);

      logk_trace("Read addr_sel: %p", pv->addr_sel);

      if (pv->addr_sel) {
        uint8_t slot = nrf_reg_get(pv->addr, NRF_TWIS_MATCH) & 1;
      
        pv->addr_sel->selection.saddr
          = nrf_reg_get(pv->addr, NRF_TWIS_ADDR0 + slot);
        pv->addr_sel->selection.read = 1;
        dev_i2c_slave_rq_done(pv->addr_sel);
        pv->addr_sel = NULL;

        pv->state = STATE_SELECT_READ;
        nrf_it_set_mask(pv->addr, 0
                        | bit(NRF_TWIS_ERROR)
                        | bit(NRF_TWIS_STOP)
                        );

        if (pv->tx) {
          nrf_it_enable(pv->addr, NRF_TWIS_TXSTARTED);
          nrf_task_trigger(pv->addr, NRF_TWIS_PREPARETX);
          nrf_task_trigger(pv->addr, NRF_TWIS_RESUME);
          logk_trace("TX prepared, resume");
          pv->state = STATE_TX_WAIT;
        } else {
          logk_trace("TX pending");
        }
      } else {
        nrf_task_trigger(pv->addr, NRF_TWIS_STOP);
        nrf_it_set_mask(pv->addr, 0
                        | bit(NRF_TWIS_ERROR)
                        | bit(NRF_TWIS_STOP)
                        );
        pv->state = STATE_IDLE;
      }
    }

    if (nrf_event_check(pv->addr, NRF_TWIS_WRITE)) {
      once = 1;
      nrf_event_clear(pv->addr, NRF_TWIS_WRITE);
      nrf_event_clear(pv->addr, NRF_TWIS_STOPPED);
      nrf_event_clear(pv->addr, NRF_TWIS_ERROR);

      nrf5x_i2cs_data_cancel(pv);

      logk_trace("Write addr_sel: %p", pv->addr_sel);

      if (pv->addr_sel) {
        uint8_t slot = nrf_reg_get(pv->addr, NRF_TWIS_MATCH) & 1;
      
        pv->addr_sel->selection.saddr
          = nrf_reg_get(pv->addr, NRF_TWIS_ADDR0 + slot);
        pv->addr_sel->selection.read = 0;
        dev_i2c_slave_rq_done(pv->addr_sel);
        pv->addr_sel = NULL;

        pv->state = STATE_SELECT_WRITE;
        nrf_it_set_mask(pv->addr, 0
                        | bit(NRF_TWIS_ERROR)
                        | bit(NRF_TWIS_STOP)
                        );

        if (pv->rx) {
          nrf_it_enable(pv->addr, NRF_TWIS_RXSTARTED);
          nrf_task_trigger(pv->addr, NRF_TWIS_PREPARERX);
          nrf_task_trigger(pv->addr, NRF_TWIS_RESUME);
          logk_trace("RX prepared, resume");
          pv->state = STATE_RX_WAIT;
        } else {
          logk_trace("RX pending");
        }
      } else {
        nrf_task_trigger(pv->addr, NRF_TWIS_STOP);
        nrf_it_set_mask(pv->addr, 0
                        | bit(NRF_TWIS_ERROR)
                        | bit(NRF_TWIS_STOP)
                        );
        pv->state = STATE_IDLE;
      }
    }

    if (nrf_event_check(pv->addr, NRF_TWIS_STOPPED)) {
      once = 1;
      nrf_event_clear(pv->addr, NRF_TWIS_STOPPED);
      pv->state = STATE_IDLE;
      nrf_it_set_mask(pv->addr, 0
                      | bit(NRF_TWIS_READ)
                      | bit(NRF_TWIS_WRITE)
                      );

      logk_trace("Stopped");

      nrf5x_i2cs_data_cancel(pv);
    }

    if (nrf_event_check(pv->addr, NRF_TWIS_ERROR)) {
      once = 1;
      nrf_event_clear(pv->addr, NRF_TWIS_ERROR);
      pv->state = STATE_IDLE;
      nrf_it_set_mask(pv->addr, 0
                      | bit(NRF_TWIS_READ)
                      | bit(NRF_TWIS_WRITE)
                      );

      logk_trace("Error");

      nrf5x_i2cs_data_cancel(pv);
    }
  } while (once);

  for (int8_t i = 0; i < 32; ++i) {
    if (nrf_event_check(pv->addr, i)) {
      static const uint32_t mask = ~(0
                                     | bit(NRF_TWIS_STOPPED)
                                     | bit(NRF_TWIS_ERROR)
                                     | bit(NRF_TWIS_RXSTARTED)
                                     | bit(NRF_TWIS_TXSTARTED)
                                     | bit(NRF_TWIS_WRITE)
                                     | bit(NRF_TWIS_READ)
                                     );
        
      logk_trace("IRQ %d", i);
      if ((mask >> i) & 1)
        nrf_event_clear(pv->addr, i);
    }
  }
}

static
void nrf5x_i2c_slave_saddr_setup(struct nrf5x_i2cs_pv_s *pv)
{
  if (pv->addr_sel) {
    logk_trace("Addr setup 0x%02x/0x%02x",
               pv->addr_sel->selection.saddr,
               pv->addr_sel->selection.saddr_mask);
    nrf_reg_set(pv->addr, NRF_TWIS_ADDR0, pv->addr_sel->selection.saddr);
    if (pv->addr_sel->selection.saddr != 0x7f) {
      nrf_reg_set(pv->addr, NRF_TWIS_ADDR1, 0x7f
                  ^ pv->addr_sel->selection.saddr
                  ^ pv->addr_sel->selection.saddr_mask);
      nrf_reg_set(pv->addr, NRF_TWIS_CONFIG, 0
                  | NRF_TWIS_CONFIG_ADDR0
                  | NRF_TWIS_CONFIG_ADDR1);
    } else {
      nrf_reg_set(pv->addr, NRF_TWIS_CONFIG, NRF_TWIS_CONFIG_ADDR0);
    }
  } else {
    logk_trace("Addr setup -");
    nrf_reg_set(pv->addr, NRF_TWIS_CONFIG, 0);
  }
}

static
DEV_I2C_SLAVE_REQUEST(nrf5x_i2c_slave_request)
{
  struct device_s *dev = accessor->dev;
  DEVICE_PV(pv, dev);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  logk_trace("Rq %p %d", rq, pv->state);

  switch (rq->type) {
  case DEV_I2C_SLAVE_SELECTION:
    logk_trace("Rq slave sel");
    if (pv->addr_sel || bit_popc8(~rq->selection.saddr_mask & 0x7f) > 1) {
      rq->error = -ENOTSUP;
      dev_i2c_slave_rq_done(rq);
      return;
    }

    pv->addr_sel = rq;

    nrf5x_i2c_slave_saddr_setup(pv);
    return;

  case DEV_I2C_SLAVE_TRANSMIT:
    logk_trace("Rq TX %p -> %p", pv->tx, rq);
    if (pv->tx) {
      rq->error = -EBUSY;
      dev_i2c_slave_rq_done(rq);
      return;
    }

    rq->error = -ECANCELED;
    pv->tx = rq;
    nrf_reg_set(pv->addr, NRF_TWIS_TXD_PTR, (uint32_t)rq->transfer.data);
    nrf_reg_set(pv->addr, NRF_TWIS_TXD_MAXCOUNT, rq->transfer.size);
    nrf_reg_set(pv->addr, NRF_TWIS_TXD_AMOUNT, 0);

    if (pv->state == STATE_SELECT_READ) {
      nrf_it_enable(pv->addr, NRF_TWIS_TXSTARTED);
      nrf_task_trigger(pv->addr, NRF_TWIS_PREPARETX);
      nrf_task_trigger(pv->addr, NRF_TWIS_RESUME);
      logk_trace("TX prepared, resume");
      pv->state = STATE_TX_WAIT;
    }
    return;

  case DEV_I2C_SLAVE_RECEIVE:
    logk_trace("Rq RX %p -> %p", pv->rx, rq);
    if (pv->rx) {
      rq->error = -EBUSY;
      dev_i2c_slave_rq_done(rq);
      return;
    }

    rq->error = -ECANCELED;
    pv->rx = rq;
    nrf_reg_set(pv->addr, NRF_TWIS_RXD_PTR, (uint32_t)rq->transfer.data);
    nrf_reg_set(pv->addr, NRF_TWIS_RXD_MAXCOUNT, rq->transfer.size);
    nrf_reg_set(pv->addr, NRF_TWIS_RXD_AMOUNT, 0);
    if (pv->state == STATE_SELECT_WRITE) {
      nrf_it_enable(pv->addr, NRF_TWIS_RXSTARTED);
      nrf_task_trigger(pv->addr, NRF_TWIS_PREPARERX);
      nrf_task_trigger(pv->addr, NRF_TWIS_RESUME);
      logk_trace("RX prepared, resume");
      pv->state = STATE_RX_WAIT;
    }
    return;

  default:
    rq->error = -ENOTSUP;
    dev_i2c_slave_rq_done(rq);
    return;
  }
}

static DEV_USE(nrf5x_i2c_slave_use)
{
  switch (op) {
  case DEV_USE_START: {
    struct device_accessor_s *acc = param;
    struct device_s *dev = acc->dev;
    struct nrf5x_i2cs_pv_s *pv = dev->drv_pv;

    logk_trace("Use start");

    return 0;
  }

  case DEV_USE_STOP: {
    struct device_accessor_s *acc = param;
    struct device_s *dev = acc->dev;

    logk_trace("Use stop");

    return 0;
  }

  default:
    return dev_use_generic(param, op);
  }
}

static
DEV_INIT(nrf5x_i2c_slave_init)
{
  struct nrf5x_i2cs_pv_s *pv;
  error_t err = -ENOMEM;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL)) {
    err = -EDESTADDRREQ;
    goto free_pv;
  }

  err = device_iomux_setup(dev, ",scl ,sda", NULL, pv->pin, NULL);
  if (err)
    goto free_pv;

  if (pv->pin[PIN_SCL] == IOMUX_INVALID_ID
      || pv->pin[PIN_SDA] == IOMUX_INVALID_ID) {
    err = -ENOTCONN;
    goto free_pv;
  }

  dev->drv_pv = pv;

  pv->addr_sel = NULL;
  pv->tx = NULL;
  pv->rx = NULL;

  device_irq_source_init(dev, &pv->irq_ep, 1, &nrf5x_i2c_slave_irq);

  err = device_irq_source_link(dev, &pv->irq_ep, 1, -1);
  if (err)
    goto free_ios;

  pv->state = STATE_IDLE;
  nrf_reg_set(pv->addr, NRF_TWIS_POWER, 1);
  nrf_reg_set(pv->addr, NRF_TWIS_ENABLE, NRF_TWIS_ENABLE_ENABLED);
  nrf_reg_set(pv->addr, NRF_TWIS_PSEL_SCL, pv->pin[PIN_SCL]);
  nrf_reg_set(pv->addr, NRF_TWIS_PSEL_SDA, pv->pin[PIN_SDA]);
  nrf_short_disable_mask(pv->addr, -1);
  nrf_short_enable_mask(pv->addr, 0
                        | bit(NRF_TWIS_READ_SUSPEND)
                        | bit(NRF_TWIS_WRITE_SUSPEND)
                        );
  nrf_it_set_mask(pv->addr, 0
                  | bit(NRF_TWIS_ERROR)
                  | bit(NRF_TWIS_STOP)
                  | bit(NRF_TWIS_READ)
                  | bit(NRF_TWIS_WRITE)
                  | bit(NRF_TWIS_TXSTARTED)
                  | bit(NRF_TWIS_RXSTARTED)
                  );

  return 0;

 free_ios:
  device_iomux_cleanup(dev);
 free_pv:
  mem_free(pv);
 err_out:
  return err;
}

static
DEV_CLEANUP(nrf5x_i2c_slave_cleanup)
{
  DEVICE_PV(pv, dev);

  if (pv->tx || pv->rx || pv->addr_sel)
    return -EBUSY;

  nrf_reg_set(pv->addr, NRF_TWIS_PSEL_SCL, (uint32_t)-1);
  nrf_reg_set(pv->addr, NRF_TWIS_PSEL_SDA, (uint32_t)-1);
  nrf_reg_set(pv->addr, NRF_TWIS_ENABLE, NRF_TWIS_ENABLE_DISABLED);

  nrf_it_set_mask(pv->addr, 0);

  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  device_iomux_cleanup(dev);
  mem_free(pv);
  return 0;
}

DRIVER_DECLARE(nrf5x_i2c_slave_drv, 0, "nRF5x i2c slave", nrf5x_i2c_slave,
               DRIVER_I2C_SLAVE_METHODS(nrf5x_i2c_slave));

DRIVER_REGISTER(nrf5x_i2c_slave_drv);

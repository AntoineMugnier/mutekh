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

    Copyright (c) 2021, Nicolas Pouillon, <nipo@ssji.net>
*/

#include <stdint.h>
#define LOGK_MODULE_ID "n1ws"

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/bit.h>
#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/irq.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/onewire.h>
#include <device/class/iomux.h>

#include <arch/nrf5x/gpio.h>
#include <arch/nrf5x/gpiote.h>
#include <arch/nrf5x/timer.h>
#include <arch/nrf5x/ppi.h>

#define GPIOTE_ADDR NRF_PERIPHERAL_ADDR(NRF5X_GPIOTE)

enum ppi_id_e
{
  PPI_BEGIN_FALL = CONFIG_DRIVER_NRF5X_ONEWIRE_PPI_FIRST,
  PPI_END_RISE,
  PPI_RISE_CAPTURE
};

enum gpiote_id_e
{
  GPIOTE_TX = CONFIG_DRIVER_NRF5X_ONEWIRE_GPIOTE_FIRST,
  GPIOTE_RX,
};

enum timer_chan_e
{
  CC_BEGIN,
  CC_END,
  CC_RISE,
  CC_SLOT,
};

#define TIMER_PRESCALER 1
#define T_BEGIN 4
#define T_B0_RISE   (T_BEGIN + 60/TIMER_PRESCALER)
#define T_B1_RISE   (T_BEGIN + 6/TIMER_PRESCALER)
#define T_BIT_TH    (T_BEGIN + 15/TIMER_PRESCALER)
#define T_BIT_SLOT  (T_BEGIN + 70/TIMER_PRESCALER)
#define T_RST_RISE  (T_BEGIN + 480/TIMER_PRESCALER)
#define T_RST_TH    (T_BEGIN + 495/TIMER_PRESCALER)
#define T_RST_SLOT  (T_BEGIN + 960/TIMER_PRESCALER)

enum nrf5x_1wire_state_e
{
  N1W_IDLE,
  N1W_RESET,
  N1W_ROM_COMMAND,
  N1W_ROM_MATCH,
  N1W_ROM_READ_P,
  N1W_ROM_READ_N,
  N1W_ROM_BIT_SEL,
  N1W_DATA_WRITE,
  N1W_DATA_READ,
  N1W_WAITING_BEFORE,
  N1W_WAITING_AFTER
};

struct nrf5x_1wire_ctx_s
{
  uintptr_t timer_addr;
  struct dev_irq_src_s irq_ep;

  iomux_io_id_t io[2];

  enum nrf5x_1wire_state_e state;
  dev_request_queue_root_t queue;
  struct dev_onewire_rq_s *current;

  uint32_t bitbang_delay;
  uint8_t buffer;
  uint8_t bit_ptr;
  size_t byte_index;
  size_t transfer_index;

  
};

DRIVER_PV(struct nrf5x_1wire_ctx_s);

static void n1w_next_slot_start(struct nrf5x_1wire_ctx_s *pv);


static void n1w_request_next(struct nrf5x_1wire_ctx_s *pv)
{

  pv->current = dev_onewire_rq_pop(&pv->queue);

  if (pv->current) {
    logk_trace("rq %p start", pv->current);
    if (pv->current->delay_before_communication_us > 0) {
      pv->state = N1W_WAITING_BEFORE;
      logk_trace("sleep before");
      nrf_reg_set(pv->timer_addr, NRF_TIMER_CC(CC_SLOT), pv->current->delay_before_communication_us / TIMER_PRESCALER);
      nrf_task_trigger(pv->timer_addr, NRF_TIMER_CLEAR);
      nrf_task_trigger(pv->timer_addr, NRF_TIMER_START);
    }
    else{
        n1w_next_slot_start(pv);
    }
  } 
}

static void n1w_end_communication(struct nrf5x_1wire_ctx_s *pv, error_t err)
{
  struct dev_onewire_rq_s *rq = pv->current;

  rq->error = err;

  logk_trace("Tx output high only");
  // High output enabled to power on strongly the bus when not performing 
  //communication with onewire slave
  nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_PIN_CNF(pv->io[0]), 0
              | NRF_GPIO_PIN_CNF_DIR_OUTPUT
              | NRF_GPIO_PIN_CNF_DRIVE_D0S1);

  if (rq->delay_after_communication_us > 0) {
    pv->state = N1W_WAITING_AFTER;
    logk_trace("sleep after");
    nrf_reg_set(pv->timer_addr, NRF_TIMER_CC(CC_SLOT), rq->delay_after_communication_us / TIMER_PRESCALER);
    nrf_task_trigger(pv->timer_addr, NRF_TIMER_CLEAR);
    nrf_task_trigger(pv->timer_addr, NRF_TIMER_START);
  }
  else{
    pv->state = N1W_IDLE;
    logk_trace("rq %p done", rq);
    dev_onewire_rq_done(rq);
    n1w_request_next(pv);
  }
}

static void dump(struct nrf5x_1wire_ctx_s *pv)
{
  logk_trace("MOD: %08x", nrf_reg_get(pv->timer_addr, NRF_TIMER_MODE));
  logk_trace("BIM: %08x", nrf_reg_get(pv->timer_addr, NRF_TIMER_BITMODE));
  logk_trace("PRE: %08x", nrf_reg_get(pv->timer_addr, NRF_TIMER_PRESCALER));
  logk_trace("CC0: %08x", nrf_reg_get(pv->timer_addr, NRF_TIMER_CC(0)));
  logk_trace("CC1: %08x", nrf_reg_get(pv->timer_addr, NRF_TIMER_CC(1)));
  logk_trace("CC2: %08x", nrf_reg_get(pv->timer_addr, NRF_TIMER_CC(2)));
  logk_trace("CC3: %08x", nrf_reg_get(pv->timer_addr, NRF_TIMER_CC(3)));
  logk_trace("IRQ: %08x", cpu_mem_read_32(pv->timer_addr + NRF_INTENSET));
}

static void n1w_reset_start(struct nrf5x_1wire_ctx_s *pv)
{
  logk_trace("%s", __func__);

  logk_trace("Tx output low only");
  // High output is disabled for master data line so the onewire slave can drive it high 
  // to signal ever a 1 or 0
  nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_PIN_CNF(pv->io[0]), 0
              | NRF_GPIO_PIN_CNF_DIR_OUTPUT
              | NRF_GPIO_PIN_CNF_DRIVE_S0D1);

  nrf_reg_set(pv->timer_addr, NRF_TIMER_CC(CC_BEGIN), pv->bitbang_delay + T_BEGIN);
  nrf_reg_set(pv->timer_addr, NRF_TIMER_CC(CC_END), pv->bitbang_delay + T_RST_RISE);
  nrf_reg_set(pv->timer_addr, NRF_TIMER_CC(CC_RISE), 0);
  nrf_reg_set(pv->timer_addr, NRF_TIMER_CC(CC_SLOT), pv->bitbang_delay + T_RST_SLOT);
  
  nrf_task_trigger(pv->timer_addr, NRF_TIMER_CLEAR);
  nrf_task_trigger(pv->timer_addr, NRF_TIMER_START);

  dump(pv);
}

static bool_t n1w_reset_collect(struct nrf5x_1wire_ctx_s *pv)
{
  uint32_t cc = nrf_reg_get(pv->timer_addr, NRF_TIMER_CC(CC_RISE));

  logk_trace("%s %d", __func__, cc);

  return cc > (pv->bitbang_delay + T_RST_TH);
}

static void n1w_bit_tx_start(struct nrf5x_1wire_ctx_s *pv, bool_t value)
{
  logk_trace("%s %d", __func__, value);

  nrf_reg_set(pv->timer_addr, NRF_TIMER_CC(CC_BEGIN), pv->bitbang_delay + T_BEGIN);
  nrf_reg_set(pv->timer_addr, NRF_TIMER_CC(CC_END), value ? pv->bitbang_delay + T_B1_RISE : pv->bitbang_delay + T_B0_RISE);
  nrf_reg_set(pv->timer_addr, NRF_TIMER_CC(CC_RISE), 0);
  nrf_reg_set(pv->timer_addr, NRF_TIMER_CC(CC_SLOT), pv->bitbang_delay + T_BIT_SLOT);
  
  nrf_task_trigger(pv->timer_addr, NRF_TIMER_CLEAR);
  nrf_task_trigger(pv->timer_addr, NRF_TIMER_START);
}

static void n1w_bit_rx_start(struct nrf5x_1wire_ctx_s *pv)
{
  logk_trace("%s", __func__);

  n1w_bit_tx_start(pv, 1);
}

static bool_t n1w_bit_rx_collect(struct nrf5x_1wire_ctx_s *pv)
{
  uint32_t cc = nrf_reg_get(pv->timer_addr, NRF_TIMER_CC(CC_RISE));

  logk_trace("%s %d", __func__, cc);

  return cc < (pv->bitbang_delay + T_BIT_TH);
}

static void n1w_next_slot_start(struct nrf5x_1wire_ctx_s *pv)
{

  if (!pv->current)
    return;

  switch (pv->state) {
  case N1W_IDLE:
    pv->state = N1W_RESET;
    n1w_reset_start(pv);
    break;
  case N1W_RESET:
    logk_error("A new slot should not be started when doing a reset");
    break;

  case N1W_ROM_MATCH:
    n1w_bit_tx_start(pv, (pv->current->data.rom->raw >> pv->bit_ptr) & 1);
    break;

  case N1W_ROM_COMMAND:
  case N1W_DATA_WRITE:
    n1w_bit_tx_start(pv, (pv->buffer >> pv->bit_ptr) & 1);
    break;

  case N1W_ROM_READ_P:
  case N1W_ROM_READ_N:
  case N1W_DATA_READ:
    n1w_bit_rx_start(pv);
    break;
  case N1W_ROM_BIT_SEL:
    if (pv->bit_ptr < pv->current->search.discover_after) {
      n1w_bit_tx_start(pv, bit_get(pv->current->search.rom.raw, pv->bit_ptr));
    } else {
      if (pv->buffer == 1 || pv->buffer == 2)
        n1w_bit_tx_start(pv, pv->buffer & 1);
      else if (pv->buffer == 0)
        n1w_bit_tx_start(pv, 0);
      else {
        logk_error("Should not happen");
        n1w_bit_tx_start(pv, 0);
      }
    }
    break;
  default:
    return;
  }

  logk_trace("slot start %p", pv->current);
}

static void n1w_transfer_setup(struct nrf5x_1wire_ctx_s *pv)
{
  logk_trace("transfer setup %p", pv->current);

  if (!pv->current)
    return;

  struct dev_onewire_transfer_s *cur = &pv->current->data.transfer[pv->transfer_index];

  pv->byte_index = 0;
  pv->bit_ptr = 0;

  if (cur->direction == DEV_ONEWIRE_WRITE) {
    pv->buffer = cur->data[pv->byte_index];
    pv->state = N1W_DATA_WRITE;
  } else {
    pv->buffer = 0;
    pv->state = N1W_DATA_READ;
  }
}

static void n1w_slot_done(struct nrf5x_1wire_ctx_s *pv)
{
  if (!pv->current) {
    logk_trace("slot done -none-");
    return;
  }
  
  logk_trace("slot done type %d st %d bi %d", pv->current->type, pv->state, pv->bit_ptr);

  switch (pv->state) {
  case N1W_IDLE:
    break;

  case N1W_RESET:
    if (n1w_reset_collect(pv)) {
      // Presence detected, go on
      switch (pv->current->type) {
      case DEV_ONEWIRE_DATA:
        pv->state = N1W_ROM_COMMAND;
        pv->buffer = pv->current->data.rom ? DEV_ONEWIRE_OPCODE_MATCH_ROM : DEV_ONEWIRE_OPCODE_SKIP_ROM;
        pv->transfer_index = 0;
        pv->byte_index = 0;
        break;
        
      case DEV_ONEWIRE_RAW:
        pv->transfer_index = 0;
        n1w_transfer_setup(pv);
        break;
        
      case DEV_ONEWIRE_SEARCH:
        pv->state = N1W_ROM_COMMAND;
        pv->buffer = pv->current->search.alarm_only ? DEV_ONEWIRE_OPCODE_ALARM_SEARCH : DEV_ONEWIRE_OPCODE_SEARCH_ROM;
        break;
      }
      pv->bit_ptr = 0;
    } else {
      // Presence failure
      logk_error("Presence failure");
      return n1w_end_communication(pv, -ENOENT);
    }
    break;

  case N1W_ROM_COMMAND:
    pv->bit_ptr++;
    if (pv->bit_ptr > 7) {
      switch (pv->current->type) {
      case DEV_ONEWIRE_RAW: 
      case DEV_ONEWIRE_DATA: 
        if (pv->current->data.rom) {
          pv->state = N1W_ROM_MATCH;
          pv->bit_ptr = 0;
        } else {
          pv->transfer_index = 0;
          n1w_transfer_setup(pv);
        }
        break;
      case DEV_ONEWIRE_SEARCH:
        pv->state = N1W_ROM_READ_P;
        pv->bit_ptr = 0;
        break;
      }
    }
    break;

  case N1W_ROM_MATCH:
    pv->bit_ptr++;
    if (pv->bit_ptr >= 64) {
      pv->transfer_index = 0;
      n1w_transfer_setup(pv);
    }
    break;

  case N1W_DATA_WRITE:
    pv->bit_ptr++;
    if (pv->bit_ptr > 7) {
      // Byte done.
      struct dev_onewire_transfer_s *cur = &pv->current->data.transfer[pv->transfer_index];
      pv->byte_index++;

      if (pv->byte_index >= cur->size) {
        pv->transfer_index++;
        if (pv->transfer_index >= pv->current->data.transfer_count)
          return n1w_end_communication(pv, 0);
        else{
          n1w_transfer_setup(pv);
        }
      } else {
        pv->buffer = cur->data[pv->byte_index];
        pv->bit_ptr = 0;
      }
    }
    break;

  case N1W_ROM_READ_P:
    pv->buffer = n1w_bit_rx_collect(pv);
    pv->state = N1W_ROM_READ_N;
    break;

  case N1W_ROM_READ_N:
    pv->state = N1W_ROM_BIT_SEL;
    pv->buffer |= n1w_bit_rx_collect(pv) << 1;
    if (pv->buffer == 3)
      return n1w_end_communication(pv, -EIO);
    break;

  case N1W_DATA_READ:
    pv->buffer |= (n1w_bit_rx_collect(pv)) << pv->bit_ptr;
    pv->bit_ptr++;
    if (pv->bit_ptr > 7) {
      // Byte done.
      struct dev_onewire_transfer_s *cur = &pv->current->data.transfer[pv->transfer_index];
      cur->data[pv->byte_index] = pv->buffer;
      pv->byte_index++;
      pv->buffer = 0;

      if (pv->byte_index >= cur->size) {
        pv->transfer_index++;
        if (pv->transfer_index >= pv->current->data.transfer_count)
          return n1w_end_communication(pv, 0);
        else{
          n1w_transfer_setup(pv);
        }
      } else {
        pv->bit_ptr = 0;
      }
    }
    break;

  case N1W_ROM_BIT_SEL:
    if (pv->bit_ptr < pv->current->search.discover_after) {
      bool_t expected = bit_get(pv->current->search.rom.raw, pv->bit_ptr);
      uint8_t expected_buffer = 1 << !expected;
      if (expected_buffer == pv->buffer || pv->buffer == 0) {
        pv->buffer = expected_buffer;
      } else {
        return n1w_end_communication(pv, -ENOENT);
      }
    } else {
      if (pv->buffer == 1)
        pv->current->search.rom.raw |= bit(pv->bit_ptr);
      else if (pv->buffer == 2)
        pv->current->search.rom.raw &= ~bit(pv->bit_ptr);
      else if (pv->buffer == 0) {
        pv->current->search.rom.raw &= ~bit(pv->bit_ptr);
        pv->current->search.collision.raw |= bit(pv->bit_ptr);
        // Choose 0
        pv->buffer = 2;
      } else {
        return n1w_end_communication(pv, -EIO);
      }
    }

    pv->bit_ptr++;
    if (pv->bit_ptr > 63) {
      // Selection DONE
      return n1w_end_communication(pv, 0);
    } else {
      pv->state = N1W_ROM_READ_P;
    }
    break;
  }

  n1w_next_slot_start(pv);
}


static DEV_IRQ_SRC_PROCESS(nrf5x_1wire_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_1wire_ctx_s *pv = dev->drv_pv;
  LOCK_SPIN_SCOPED(&dev->lock);

  logk_trace("irq");

  if (nrf_event_check(pv->timer_addr, NRF_TIMER_COMPARE(CC_SLOT))) {
    nrf_task_trigger(pv->timer_addr, NRF_TIMER_STOP);
    nrf_task_trigger(pv->timer_addr, NRF_TIMER_CLEAR);
    nrf_event_clear(pv->timer_addr, NRF_TIMER_COMPARE(CC_SLOT));
    if (pv->state == N1W_WAITING_BEFORE) {
        logk_trace("wakeup before");
        pv->state = N1W_IDLE;
        n1w_next_slot_start(pv);
    }
    else if(pv->state == N1W_WAITING_AFTER){
        logk_trace("wakeup after");
        pv->state = N1W_IDLE;
        logk_trace("rq %p done", pv->current);
        dev_onewire_rq_done(pv->current);
        n1w_request_next(pv);    
    }
    else{
      n1w_slot_done(pv);
    }
  }
}

static
DEV_ONEWIRE_REQUEST(nrf5x_1wire_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_1wire_ctx_s *pv = dev->drv_pv;
  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  logk_trace("request %p", rq);


  if (rq->type == DEV_ONEWIRE_SEARCH) {
    rq->search.collision.raw = 0;
  }
  
  dev_onewire_rq_pushback(&pv->queue, rq);

  if (!pv->current)
    n1w_request_next(pv);
}

#define nrf5x_1wire_use dev_use_generic

static DEV_INIT(nrf5x_1wire_init)
{
  struct nrf5x_1wire_ctx_s *pv;
  error_t err;
  uintptr_t addr;

  err = device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL);
  if (err)
    return err;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  pv->timer_addr = addr;
  dev->drv_pv = pv;
  err = device_iomux_setup(dev, ",dq ^dqpw?", NULL, pv->io, NULL);
  if (err)
    goto err_gpio;


  // Setting maximum frequency of the onewire bus, note that this frequency is theoretical and may be hard to reach
  // due to other CPU tasks running in parallel of the onewire communication
  uint32_t bus_max_frequency_hz;
  err = device_get_param_uint(dev, "bus_max_frequency_hz", &bus_max_frequency_hz);
  if(err){
    return err;
  }
  int possible_bitbang_delay = (1000000/bus_max_frequency_hz) - T_BIT_SLOT;
  pv->bitbang_delay = possible_bitbang_delay >0 ? possible_bitbang_delay : 0;

  device_irq_source_init(dev, &pv->irq_ep, 1, &nrf5x_1wire_irq);
  if (device_irq_source_link(dev, &pv->irq_ep, 1, -1))
    goto err_gpio;

  dev_rq_queue_init(&pv->queue);

  nrf_it_disable_mask(pv->timer_addr, -1);
  nrf_task_trigger(pv->timer_addr, NRF_TIMER_STOP);
  nrf_task_trigger(pv->timer_addr, NRF_TIMER_CLEAR);

  nrf_reg_set(pv->timer_addr, NRF_TIMER_BITMODE, NRF_TIMER_BITMODE_32);
  nrf_reg_set(pv->timer_addr, NRF_TIMER_MODE, NRF_TIMER_MODE_TIMER);
  nrf_reg_set(pv->timer_addr, NRF_TIMER_PRESCALER,
#if TIMER_PRESCALER == 1
              NRF_TIMER_FREQ_1MHz
#elif TIMER_PRESCALER == 2
              NRF_TIMER_FREQ_500kHz
#else
# error TBD
#endif
               );

  nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_PIN_CNF(pv->io[1]), 0
              | NRF_GPIO_PIN_CNF_DIR_INPUT);

  nrf_short_set(pv->timer_addr, 0
                | bit(NRF_TIMER_COMPARE_STOP(CC_SLOT))
                | bit(NRF_TIMER_COMPARE_CLEAR(CC_SLOT)));

  nrf_reg_set(GPIOTE_ADDR, NRF_GPIOTE_CONFIG(GPIOTE_TX), 0
              | NRF_GPIOTE_CONFIG_MODE_TASK
              | NRF_GPIOTE_CONFIG_PSEL(pv->io[0])
              | NRF_GPIOTE_CONFIG_OUTINIT_HIGH
              | NRF_GPIOTE_CONFIG_POLARITY_TOGGLE);
  nrf_reg_set(GPIOTE_ADDR, NRF_GPIOTE_CONFIG(GPIOTE_RX), 0
              | NRF_GPIOTE_CONFIG_MODE_EVENT
              | NRF_GPIOTE_CONFIG_PSEL(pv->io[1])
              | NRF_GPIOTE_CONFIG_POLARITY_LOTOHI);

  nrf_ppi_setup(PPI_BEGIN_FALL,
                pv->timer_addr, NRF_TIMER_COMPARE(CC_BEGIN),
                GPIOTE_ADDR, NRF_GPIOTE_OUT(GPIOTE_TX));
  nrf_ppi_setup(PPI_END_RISE,
                pv->timer_addr, NRF_TIMER_COMPARE(CC_END),
                GPIOTE_ADDR, NRF_GPIOTE_OUT(GPIOTE_TX));
  nrf_ppi_setup(PPI_RISE_CAPTURE,
                GPIOTE_ADDR, NRF_GPIOTE_IN(GPIOTE_RX),
                pv->timer_addr, NRF_TIMER_CAPTURE(CC_RISE));

  nrf_ppi_enable_mask(0
                      | bit(PPI_BEGIN_FALL)
                      | bit(PPI_END_RISE)
                      | bit(PPI_RISE_CAPTURE)
                      );

  nrf_it_enable(pv->timer_addr, NRF_TIMER_COMPARE(CC_SLOT));

  return 0;

 err_gpio:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(nrf5x_1wire_cleanup)
{
  struct nrf5x_1wire_ctx_s *pv = dev->drv_pv;
  uintptr_t addr = pv->timer_addr;

  if (!dev_rq_queue_isempty(&pv->queue))
    return -EBUSY;

  nrf_it_disable_mask(addr, -1);
  nrf_task_trigger(addr, NRF_TIMER_STOP);
  nrf_task_trigger(addr, NRF_TIMER_CLEAR);
  device_irq_source_unlink(dev, &pv->irq_ep, 1);
  dev_rq_queue_destroy(&pv->queue);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(nrf5x_1wire_drv, 0, "1-Wire", nrf5x_1wire,
               DRIVER_ONEWIRE_METHODS(nrf5x_1wire));

DRIVER_REGISTER(nrf5x_1wire_drv);

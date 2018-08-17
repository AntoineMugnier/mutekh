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

    Copyright (c) 2016, Nicolas Pouillon <nipo@ssji.net>
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/valio.h>
#include <device/class/iomux.h>
#include <device/valio/ir.h>

#include <arch/nrf5x/pwm.h>
#include <arch/nrf5x/ids.h>
#include <arch/nrf5x/peripheral.h>

enum ir_symbol_e
{
  RC6_LEAD_START,
  RC6_BIT_0,
  RC6_BIT_1,
  RC6_TOGGLE_0,
  RC6_TOGGLE_1,
  RC6_CLEAR,
  RC5_START,
  RC5_BIT_0,
  RC5_BIT_1,
};

struct ir_symbol_seq_s
{
  uint16_t repeat;
  uint16_t top;
  uint16_t seq_count;
  const uint16_t *seq;
};

#define PWM_CLOCK 16000000ULL

#define SEQ(r, t, x...) {r, t, ARRAY_SIZE(((uint16_t[]){x})), (uint16_t[]){x}}
#define SYMBOL_REP(carrier, symus) ((((symus) * (carrier) + 500000) / 1000000))
#define STROBE_TOP(carrier, symus) (PWM_CLOCK * (symus) / 1000000 / SYMBOL_REP(carrier, symus))

#define RC5_SYMBOL_US 889
#define RC5_CARRIER 36000
#define RC5_STROBE_COUNT SYMBOL_REP(RC5_CARRIER, RC5_SYMBOL_US)
#define RC5_STROBE_TOP STROBE_TOP(RC5_CARRIER, RC5_SYMBOL_US)
#define RC5_S 0
#define RC5_M (RC6_STROBE_TOP / 3)
#define RC5_SEQ(x...) SEQ(RC5_STROBE_COUNT, RC5_STROBE_TOP, x)

#define RC6_CARRIER 36000
#define RC6_SYMBOL_US 444
#define RC6_STROBE_COUNT SYMBOL_REP(RC6_CARRIER, RC6_SYMBOL_US)
#define RC6_STROBE_TOP STROBE_TOP(RC6_CARRIER, RC6_SYMBOL_US)
#define RC6_S 0
#define RC6_M (RC6_STROBE_TOP / 4)
#define RC6_SEQ(x...) SEQ(RC6_STROBE_COUNT, RC6_STROBE_TOP, x)

static const struct ir_symbol_seq_s ir_seq[]  = {
  [RC6_LEAD_START] = RC6_SEQ(RC6_M, RC6_M, RC6_M, RC6_M, RC6_M, RC6_M, RC6_S, RC6_S, RC6_M, RC6_S),
  [RC6_BIT_0] =      RC6_SEQ(RC6_S, RC6_M),
  [RC6_BIT_1] =      RC6_SEQ(RC6_M, RC6_S),
  [RC6_TOGGLE_0] =   RC6_SEQ(RC6_S, RC6_S, RC6_M, RC6_M),
  [RC6_TOGGLE_1] =   RC6_SEQ(RC6_M, RC6_M, RC6_S, RC6_S),
  [RC6_CLEAR] =      RC6_SEQ(RC6_S, RC6_S, RC6_S, RC6_S, RC6_S, RC6_S),
  [RC5_START] =      RC5_SEQ(RC5_S, RC5_M, RC5_S, RC5_M),
  [RC5_BIT_0] =      RC5_SEQ(RC5_M, RC5_S),
  [RC5_BIT_1] =      RC5_SEQ(RC5_S, RC5_M),
};

enum ir_blaster_state_e
{
  ST_IDLE,
  ST_RC6_LEAD_START,
  ST_RC6_MODE,
  ST_RC6_TOGGLE,
  ST_RC6_DATA,
  ST_RC6_CLEAR,
  ST_RC5_START,
  ST_RC5_DATA,
};

struct nrf52_ir_pv_s
{
  uintptr_t addr;
  dev_request_queue_root_t queue;
  struct dev_valio_rq_s * current;

  struct dev_irq_src_s irq_ep;

  enum ir_blaster_state_e state;
  size_t bit;

  bool_t seq_cur;
};

DRIVER_PV(struct nrf52_ir_pv_s);

static void nrf52_ir_symbol_send(struct nrf52_ir_pv_s *pv,
                                 enum ir_symbol_e symbol)
{
  const struct ir_symbol_seq_s *seq = &ir_seq[symbol];
  uintptr_t off = pv->seq_cur ? NRF_PWM_SEQ1_PTR - NRF_PWM_SEQ0_PTR : 0;

  logk_trace("%s %d\n", __FUNCTION__, symbol);

  nrf_reg_set(pv->addr, off + NRF_PWM_SEQ0_PTR, (uintptr_t)seq->seq);
  nrf_reg_set(pv->addr, off + NRF_PWM_SEQ0_CNT, seq->seq_count);
  nrf_reg_set(pv->addr, off + NRF_PWM_SEQ0_REFRESH, seq->repeat - 1);
  nrf_reg_set(pv->addr, off + NRF_PWM_SEQ0_ENDDELAY, 0);
  nrf_reg_set(pv->addr, NRF_PWM_COUNTERTOP, seq->top);
}

static void nrf52_ir_disable(struct nrf52_ir_pv_s *pv)
{
  nrf_short_set(pv->addr, 0);
  nrf_task_trigger(pv->addr, NRF_PWM_STOP);
  nrf_it_disable_mask(pv->addr, -1);
  nrf_reg_set(pv->addr, NRF_PWM_ENABLE, 0);
}

static void nrf52_ir_step(struct device_s *dev)
{
  struct nrf52_ir_pv_s *pv = dev->drv_pv;

 again:
  if (!pv->current) {
    nrf52_ir_disable(pv);
    return;
  }

  struct dev_valio_rq_s *rq = pv->current;
  const struct valio_ir_command_s *cmd = rq->data;

  pv->seq_cur = !pv->seq_cur;

  switch (pv->state) {
  case ST_IDLE:
    pv->seq_cur = 0;

    nrf_event_clear(pv->addr, NRF_PWM_SEQSTARTED0);
    nrf_event_clear(pv->addr, NRF_PWM_SEQSTARTED1);
    nrf_event_clear(pv->addr, NRF_PWM_STOPPED);
    nrf_reg_set(pv->addr, NRF_PWM_ENABLE, NRF_PWM_ENABLE_ENABLED);
    nrf_reg_set(pv->addr, NRF_PWM_MODE, NRF_PWM_MODE_UPDOWN_UP);
    nrf_reg_set(pv->addr, NRF_PWM_PRESCALER, 0);

    switch (cmd->type) {
    default:
      assert(0);
      return;

    case VALIO_IR_RC6:
      nrf52_ir_symbol_send(pv, RC6_LEAD_START);
      pv->state = ST_RC6_LEAD_START;
      goto rq_start;

    case VALIO_IR_RC5:
      nrf52_ir_symbol_send(pv, RC5_START);
      pv->state = ST_RC5_START;
      goto rq_start;
    }
    break;

  case ST_RC6_LEAD_START:
    pv->bit = 3;
    pv->state = ST_RC6_MODE;

  case ST_RC6_MODE:
    if (pv->bit == 0) {
      pv->state = ST_RC6_TOGGLE;
      nrf52_ir_symbol_send(pv, bit_get(cmd->value, cmd->bit_count - 4) ? RC6_TOGGLE_1 : RC6_TOGGLE_0);
      goto rq_proceed;
    }
    pv->bit--;
    nrf52_ir_symbol_send(pv, bit_get(cmd->value, cmd->bit_count - 3 + pv->bit) ? RC6_BIT_1 : RC6_BIT_0);
    goto rq_proceed;
    
  case ST_RC6_TOGGLE:
    pv->bit = cmd->bit_count - 4;
    pv->state = ST_RC6_DATA;

  case ST_RC6_DATA:
    if (pv->bit == 0) {
      pv->state = ST_RC6_CLEAR;
      nrf52_ir_symbol_send(pv, RC6_CLEAR);
      goto rq_last;
    }
    pv->bit--;
    nrf52_ir_symbol_send(pv, bit_get(cmd->value, pv->bit) ? RC6_BIT_1 : RC6_BIT_0);
    goto rq_proceed;

  case ST_RC6_CLEAR:
    goto rq_done;
    
  case ST_RC5_START:
    pv->bit = 12;
    pv->state = ST_RC5_DATA;

  case ST_RC5_DATA:
    if (pv->bit == 0)
      goto rq_done;

    pv->bit--;
    nrf52_ir_symbol_send(pv, bit_get(cmd->value, pv->bit) ? RC5_BIT_1 : RC5_BIT_0);
    if (pv->bit == 1)
      goto rq_last;

    goto rq_proceed;

  rq_start:
    logk_debug("%s start\n", __FUNCTION__);

    nrf_reg_set(pv->addr, NRF_PWM_LOOP, 1);
    nrf_short_set(pv->addr, 0);
    nrf_task_trigger(pv->addr, NRF_PWM_SEQSTART0);
    nrf_short_set(pv->addr, bit(NRF_PWM_LOOPSDONE_SEQSTART0));

  rq_proceed:
    logk_debug("%s proceed seq %d state %d\n", __FUNCTION__, pv->seq_cur, pv->state);

    nrf_reg_set(pv->addr, NRF_PWM_LOOP, 1);
    if (pv->seq_cur == 0)
      nrf_it_set_mask(pv->addr, bit(NRF_PWM_SEQSTARTED0));
    else
      nrf_it_set_mask(pv->addr, bit(NRF_PWM_SEQSTARTED1));
    return;

  rq_last:
    logk_debug("%s last seq %d state %d\n", __FUNCTION__, pv->seq_cur, pv->state);

    if (pv->seq_cur == 0)
      nrf_short_set(pv->addr, bit(NRF_PWM_SEQEND0_STOP));
    else
      nrf_short_set(pv->addr, bit(NRF_PWM_SEQEND1_STOP));

    nrf_it_set_mask(pv->addr, bit(NRF_PWM_STOPPED));
    return;

  rq_done:
    logk_debug("%s done\n", __FUNCTION__);

    pv->state = ST_IDLE;
    pv->current = NULL;
    rq->error = 0;
    dev_valio_rq_done(rq);
    nrf_it_set_mask(pv->addr, 0);

    pv->current = dev_valio_rq_pop(&pv->queue);
    goto again;
  }
}

static DEV_IRQ_SRC_PROCESS(nrf52_ir_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf52_ir_pv_s *pv = dev->drv_pv;

  if (nrf_it_is_enabled(pv->addr, NRF_PWM_SEQSTARTED0)
      && nrf_event_check(pv->addr, NRF_PWM_SEQSTARTED0)) {
    nrf_event_clear(pv->addr, NRF_PWM_SEQSTARTED1);
    logk_trace("%s seqstarted0\n", __FUNCTION__);
    goto push_next;
  }

  if (nrf_it_is_enabled(pv->addr, NRF_PWM_SEQSTARTED1)
      && nrf_event_check(pv->addr, NRF_PWM_SEQSTARTED1)) {
    nrf_event_clear(pv->addr, NRF_PWM_SEQSTARTED0);
    logk_trace("%s seqstarted1\n", __FUNCTION__);
    goto push_next;
  }

  if (nrf_it_is_enabled(pv->addr, NRF_PWM_STOPPED)
      && nrf_event_check(pv->addr, NRF_PWM_STOPPED)) {
    nrf_event_clear(pv->addr, NRF_PWM_STOPPED);
    logk_trace("%s stopped\n", __FUNCTION__);
    goto push_next;
  }

  return;

 push_next:

  nrf52_ir_step(dev);
}

static DEV_VALIO_REQUEST(nrf52_ir_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf52_ir_pv_s *pv = dev->drv_pv;
  const struct valio_ir_command_s *cmd = req->data;

  if (req->type != DEVICE_VALIO_WRITE)
    goto nosup;

  if (req->attribute != VALIO_IR_COMMAND)
    goto nosup;

  logk_debug("%s %d\n", __FUNCTION__, cmd->type);

  switch (cmd->type) {
  case VALIO_IR_RC5:
    if (cmd->bit_count != 12)
      goto inval;
    break;
  case VALIO_IR_RC6:
    if ((cmd->bit_count - 4) % 8)
      goto inval;

    if ((cmd->bit_count - 4) / 8 > 4)
      goto inval;

    break;

  default:
    goto nosup;
  }

  if (dev_rq_queue_isempty(&pv->queue) && pv->current == NULL) {
    pv->current = req;
    nrf52_ir_step(dev);
  } else {
    dev_valio_rq_pushback(&pv->queue, req);
  }

  return;

 nosup:
  req->error = -ENOTSUP;
  dev_valio_rq_done(req);
  return;

 inval:
  req->error = -EINVAL;
  dev_valio_rq_done(req);
  return;
}

static DEV_VALIO_CANCEL(nrf52_ir_cancel)
{
  struct device_s *dev = accessor->dev;
  struct nrf52_ir_pv_s *pv = dev->drv_pv;
  error_t err = -ENOENT;

  logk_debug("%s\n", __FUNCTION__);

  if (pv->current == req)
    return -EBUSY;

  GCT_FOREACH(dev_request_queue, &pv->queue, item,
              if (item == &req->base) {
                err = 0;
                GCT_FOREACH_BREAK;
              });

  if (err)
    return err;

  dev_valio_rq_remove(&pv->queue, req);

  return 0;
}

#define nrf52_ir_use dev_use_generic

static DEV_INIT(nrf52_ir_init)
{
  struct nrf52_ir_pv_s *pv;
  iomux_io_id_t id;
  error_t err;
  uintptr_t addr = 0;

  err = device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL);
  if (err)
    return err;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;
  memset(pv, 0, sizeof(*pv));

  pv->addr = addr;

  if (device_iomux_setup(dev, ">out", NULL, &id, NULL))
    goto free_pv;

  pv->state = ST_IDLE;

  CPU_INTERRUPT_SAVESTATE_DISABLE;

  nrf_reg_set(pv->addr, NRF_PWM_ENABLE, 0);

  nrf_reg_set(pv->addr, NRF_PWM_PSEL_OUT0, id);
  nrf_reg_set(pv->addr, NRF_PWM_PSEL_OUT1, (uint32_t)-1);
  nrf_reg_set(pv->addr, NRF_PWM_PSEL_OUT2, (uint32_t)-1);
  nrf_reg_set(pv->addr, NRF_PWM_PSEL_OUT3, (uint32_t)-1);

  nrf_it_disable_mask(pv->addr, -1);

  nrf_short_set(pv->addr, 0);

  nrf_event_clear(pv->addr, NRF_PWM_STOPPED);
  nrf_event_clear(pv->addr, NRF_PWM_SEQSTARTED0);
  nrf_event_clear(pv->addr, NRF_PWM_SEQSTARTED1);
  nrf_event_clear(pv->addr, NRF_PWM_SEQEND0);
  nrf_event_clear(pv->addr, NRF_PWM_SEQEND1);
  nrf_event_clear(pv->addr, NRF_PWM_PWMPERIODEN);
  nrf_event_clear(pv->addr, NRF_PWM_LOOPSDONE);

  CPU_INTERRUPT_RESTORESTATE;

  device_irq_source_init(dev, &pv->irq_ep, 1, &nrf52_ir_irq);
  if (device_irq_source_link(dev, &pv->irq_ep, 1, -1))
    goto free_pv;

  dev_rq_queue_init(&pv->queue);

  return 0;

 free_pv:
  mem_free(pv);

  return -1;
}

static DEV_CLEANUP(nrf52_ir_cleanup)
{
  struct nrf52_ir_pv_s *pv = dev->drv_pv;

  if (!dev_rq_queue_isempty(&pv->queue))
    return -EBUSY;

  nrf_it_disable_mask(pv->addr, -1);

  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  nrf_task_trigger(pv->addr, NRF_PWM_STOP);

  nrf_reg_set(pv->addr, NRF_PWM_ENABLE, 0);

  nrf_reg_set(pv->addr, NRF_PWM_PSEL_OUT0, (uint32_t)-1);
  nrf_reg_set(pv->addr, NRF_PWM_PSEL_OUT1, (uint32_t)-1);
  nrf_reg_set(pv->addr, NRF_PWM_PSEL_OUT2, (uint32_t)-1);
  nrf_reg_set(pv->addr, NRF_PWM_PSEL_OUT3, (uint32_t)-1);

  dev_rq_queue_destroy(&pv->queue);

  device_iomux_cleanup(dev);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(nrf52_ir_drv, 0, "nRF52 IR", nrf52_ir,
               DRIVER_VALIO_METHODS(nrf52_ir));

DRIVER_REGISTER(nrf52_ir_drv);

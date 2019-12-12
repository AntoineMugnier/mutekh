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
*/

#undef LOGK_MODULE_ID
#define LOGK_MODULE_ID "rfpk"

#include <device/class/rfpacket.h>
#include <hexo/enum.h>
#include <stdbool.h>
#include <mutek/printk.h>

const char dev_rfpacket_modulation_e[] = ENUM_DESC_DEV_RFPACKET_MODULATION_E;
const char dev_rfpacket_encoding_e[] = ENUM_DESC_DEV_RFPACKET_ENCODING_E;
const char dev_rfpacket_format_e[] = ENUM_DESC_DEV_RFPACKET_FORMAT_E;
const char dev_rfpacket_lora_encoding_e[] = ENUM_DESC_DEV_RFPACKET_LORA_ENCODING_E;

# ifdef CONFIG_MUTEK_CONTEXT_SCHED

extern inline error_t dev_rfpacket_wait_rq(
       const struct device_rfpacket_s *accessor,
       struct dev_rfpacket_rq_s *rq);

enum dev_rfpacket_wait_state_s
{
  RFPACKET_RX_IDLE,
  RFPACKET_RX_STARTED,
  RFPACKET_RX_WAIT,          /* dev_rfpacket_wait_rx called */
  RFPACKET_RX_WAIT_RXING,     /* dev_rfpacket_wait_alloc called */
  RFPACKET_RX_WAIT_END,     /* dev_rfpacket_wait_alloc called */
  RFPACKET_RX_PREPARED,      /* dev_rfpacket_prepare_rx called */
  RFPACKET_RX_PREPARED_RXING,
  RFPACKET_RX_PREPARED_DONE,
  RFPACKET_RX_PREPARED_END,
  RFPACKET_RX_WAIT_CANCEL,   /* dev_rfpacket_wait_stop called */
};

STRUCT_COMPOSE(dev_rfpacket_wait_ctx_s, rx_rq);
STRUCT_COMPOSE(dev_rfpacket_wait_ctx_s, rx);

static KROUTINE_EXEC(dev_rfpacket_rx_end)
{
  struct dev_rfpacket_rq_s *rx_rq = dev_rfpacket_rq_from_kr(kr);
  struct dev_rfpacket_wait_ctx_s *ctx = dev_rfpacket_wait_ctx_s_from_rx_rq(rx_rq);

  LOCK_SPIN_IRQ(&ctx->lock);
  logk_trace("%s %u %u", __func__, rx_rq->base.pvuint, rx_rq->error);

  switch (rx_rq->base.pvuint)
    {
    case RFPACKET_RX_WAIT:
      ctx->rx.error = rx_rq->error ? -EIO : -ETIMEDOUT;
      /* resume the dev_rfpacket_wait_rx function */
    case RFPACKET_RX_WAIT_CANCEL:
      /* resume the dev_rfpacket_stop_rx function */
      sched_context_start(ctx->sched_ctx);
    case RFPACKET_RX_STARTED:
    case RFPACKET_RX_PREPARED:
    case RFPACKET_RX_PREPARED_DONE:
      rx_rq->base.pvuint = RFPACKET_RX_IDLE;
      break;

    case RFPACKET_RX_WAIT_RXING:
      rx_rq->base.pvuint = RFPACKET_RX_WAIT_END;
      break;

    case RFPACKET_RX_PREPARED_RXING:
      rx_rq->base.pvuint = RFPACKET_RX_PREPARED_END;
      break;

    default:
      UNREACHABLE();
    }

  LOCK_RELEASE_IRQ(&ctx->lock);
}

static KROUTINE_EXEC(dev_rfpacket_rx_packet)
{
  struct dev_rfpacket_rx_s *rx = dev_rfpacket_rx_s_from_kr(kr);
  struct dev_rfpacket_wait_ctx_s *ctx = dev_rfpacket_wait_ctx_s_from_rx(rx);
  struct dev_rfpacket_rq_s *rx_rq = &ctx->rx_rq;

  assert(rx->error != -EBUSY && rx->error != -ETIMEDOUT);

  LOCK_SPIN_IRQ(&ctx->lock);
  logk_trace("%s %u", __func__, rx_rq->base.pvuint);

  switch (rx_rq->base.pvuint)
    {
    case RFPACKET_RX_WAIT_RXING:
      rx_rq->base.pvuint = RFPACKET_RX_STARTED;
      sched_context_start(ctx->sched_ctx);
      break;

    case RFPACKET_RX_PREPARED_RXING:
      rx_rq->base.pvuint = RFPACKET_RX_PREPARED_DONE;
      break;

    case RFPACKET_RX_WAIT_END:
      sched_context_start(ctx->sched_ctx);
    case RFPACKET_RX_PREPARED_END:
      rx_rq->base.pvuint = RFPACKET_RX_IDLE;
      break;

    default:
      UNREACHABLE();
    }

  LOCK_RELEASE_IRQ(&ctx->lock);
}

static struct dev_rfpacket_rx_s *
dev_rfpacket_wait_alloc(struct dev_rfpacket_rq_s *rq, size_t size)
{
  struct dev_rfpacket_wait_ctx_s *ctx = dev_rfpacket_wait_ctx_s_from_rx_rq(rq);
  struct dev_rfpacket_rx_s *rx = &ctx->rx;
  struct dev_rfpacket_rq_s *rx_rq = &ctx->rx_rq;

  LOCK_SPIN_IRQ(&ctx->lock);
  logk_trace("%s %u", __func__, rx_rq->base.pvuint);

  if (rx->size < size)
    rx = NULL;
  else
    {
      switch (rx_rq->base.pvuint)
        {
        case RFPACKET_RX_WAIT:
          rx_rq->base.pvuint = RFPACKET_RX_WAIT_RXING;
          break;

        case RFPACKET_RX_PREPARED:
          rx_rq->base.pvuint = RFPACKET_RX_PREPARED_RXING;
          break;

        default:
          rx = NULL;
          break;
        }
    }

  if (rx)
    rx->size = size;

  LOCK_RELEASE_IRQ(&ctx->lock);
  return rx;
}

error_t dev_rfpacket_wait_init(struct dev_rfpacket_wait_ctx_s *ctx,
                               const struct device_rfpacket_s *rf_dev,
                               const struct dev_rfpacket_rf_cfg_s *rf_cfg,
                               const struct dev_rfpacket_pk_cfg_s *pk_cfg)
{
  logk_trace("%s", __func__);
  struct dev_rfpacket_rq_s *tx_rq = &ctx->tx_rq;
  struct dev_rfpacket_rq_s *rx_rq = &ctx->rx_rq;
  struct dev_rfpacket_rx_s *rx = &ctx->rx;

  ctx->rf_dev = rf_dev;
  rx_rq->rf_cfg = tx_rq->rf_cfg = rf_cfg;
  rx_rq->pk_cfg = tx_rq->pk_cfg = pk_cfg;

  rx_rq->anchor = DEV_RFPACKET_TIMESTAMP_END;
  rx_rq->err_group = 0;
  rx_rq->rx_alloc = &dev_rfpacket_wait_alloc;
  rx_rq->rx_chan_mask = 1;
  rx_rq->base.pvuint = RFPACKET_RX_IDLE;

  dev_rfpacket_rq_init_immediate(rx_rq, &dev_rfpacket_rx_end);
  kroutine_init_immediate(&rx->kr, &dev_rfpacket_rx_packet);
  lock_init(&ctx->lock);

  tx_rq->anchor = DEV_RFPACKET_TIMESTAMP_END;
  tx_rq->err_group = 0;

  return 0;
}

void dev_rfpacket_wait_cleanup(struct dev_rfpacket_wait_ctx_s *ctx)
{
  IFASSERT(struct dev_rfpacket_rq_s *rx_rq = &ctx->rx_rq);
  assert(rx_rq->base.pvuint == RFPACKET_RX_IDLE);
  lock_destroy(&ctx->lock);
}

error_t dev_rfpacket_start_rx(struct dev_rfpacket_wait_ctx_s *ctx,
                              uint_fast16_t channel, dev_timer_delay_t timeout)
{
  struct dev_rfpacket_rq_s *rx_rq = &ctx->rx_rq;
  error_t err;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&ctx->lock);
  logk_trace("%s %u", __func__, rx_rq->base.pvuint);

  switch (rx_rq->base.pvuint)
    {
    case RFPACKET_RX_IDLE:
      ctx->rx.buf = NULL;
      rx_rq->base.pvuint = RFPACKET_RX_STARTED;
      lock_release(&ctx->lock);

      rx_rq->type = timeout ? DEV_RFPACKET_RQ_RX_TIMEOUT : DEV_RFPACKET_RQ_RX_CONT;
      rx_rq->lifetime = timeout;
      rx_rq->channel = channel;

      /* may call dev_rfpacket_rx_end */
      DEVICE_OP(ctx->rf_dev, request, rx_rq, NULL);

      err = 0;
      break;

    default:
      lock_release(&ctx->lock);
      err = -EBUSY;
      break;
    }

  CPU_INTERRUPT_RESTORESTATE;

  return err;
}

error_t dev_rfpacket_stop_rx(struct dev_rfpacket_wait_ctx_s *ctx)
{
  struct dev_rfpacket_rq_s *rx_rq = &ctx->rx_rq;
  error_t err;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&ctx->lock);
  logk_trace("%s %u", __func__, rx_rq->base.pvuint);

  switch (rx_rq->base.pvuint)
    {
    case RFPACKET_RX_STARTED:
      if (!DEVICE_OP(ctx->rf_dev, cancel, &ctx->rx_rq))
        {
          rx_rq->base.pvuint = RFPACKET_RX_IDLE;
        case RFPACKET_RX_IDLE:
          lock_release(&ctx->lock);
        }
      else
        {
          rx_rq->base.pvuint = RFPACKET_RX_WAIT_CANCEL;
          sched_stop_unlock(&ctx->lock);
        }
      err = 0;
      break;

    case RFPACKET_RX_PREPARED:
    case RFPACKET_RX_PREPARED_DONE:
    case RFPACKET_RX_PREPARED_RXING:
    case RFPACKET_RX_PREPARED_END:
      /* dev_rfpacket_wait_rx not called
         after dev_rfpacket_prepare_rx */

    default:
      UNREACHABLE();
    }

  CPU_INTERRUPT_RESTORESTATE;

  return err;
}

error_t dev_rfpacket_wait_tx(struct dev_rfpacket_wait_ctx_s *ctx,
                             const uint8_t *data, size_t size,
                             uint_fast16_t channel, dev_rfpacket_pwr_t pwr,
                             dev_timer_delay_t timeout)
{
  logk_trace("%s", __func__);
  struct dev_rfpacket_rq_s *tx_rq = &ctx->tx_rq;

  if (timeout)
    {
      tx_rq->type = DEV_RFPACKET_RQ_TX_FAIR;
      tx_rq->lifetime = timeout;
    }
  else
    {
      tx_rq->type = DEV_RFPACKET_RQ_TX;
    }

  tx_rq->tx_buf = data;
  tx_rq->tx_size = size;
  tx_rq->tx_pwr = pwr;
  tx_rq->deadline = 0;
  tx_rq->channel = channel;
  return dev_rfpacket_wait_rq(ctx->rf_dev, tx_rq);
}

error_t dev_rfpacket_wait_rx(struct dev_rfpacket_wait_ctx_s *ctx,
                             uint8_t *buffer, size_t *size)
{
  struct dev_rfpacket_rq_s *rx_rq = &ctx->rx_rq;
  struct dev_rfpacket_rx_s *rx = &ctx->rx;
  error_t err = 0;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&ctx->lock);
  logk_trace("%s %u", __func__, rx_rq->base.pvuint);

  switch (rx_rq->base.pvuint)
    {
    case RFPACKET_RX_IDLE:
      err = -EBUSY;
      lock_release(&ctx->lock);
      break;

    case RFPACKET_RX_PREPARED_DONE:
      rx_rq->base.pvuint = RFPACKET_RX_STARTED;
      assert(rx->buf == buffer);
      lock_release(&ctx->lock);
      goto done;

    case RFPACKET_RX_STARTED:
      rx->buf = buffer;
      rx->size = *size;

    case RFPACKET_RX_PREPARED:
      rx_rq->base.pvuint = RFPACKET_RX_WAIT;
      goto wait;

    case RFPACKET_RX_PREPARED_RXING:
      rx_rq->base.pvuint = RFPACKET_RX_WAIT_RXING;
    wait:
      assert(rx->buf == buffer);
      ctx->sched_ctx = sched_get_current();
      sched_stop_unlock(&ctx->lock);

    done:
      err = rx->error;
      if (!err)
        *size = rx->size;
      break;

    default:
      UNREACHABLE();
    }
  CPU_INTERRUPT_RESTORESTATE;

  return err;
}

config_depend_and2(CONFIG_DEVICE_RFPACKET, CONFIG_MUTEK_CONTEXT_SCHED)
error_t dev_rfpacket_prepare_rx(struct dev_rfpacket_wait_ctx_s *ctx,
                                uint8_t *buffer, size_t size)
{
  struct dev_rfpacket_rq_s *rx_rq = &ctx->rx_rq;
  struct dev_rfpacket_rx_s *rx = &ctx->rx;
  error_t err = 0;

  LOCK_SPIN_IRQ(&ctx->lock);
  logk_trace("%s %u", __func__, rx_rq->base.pvuint);

  switch (rx_rq->base.pvuint)
    {
    case RFPACKET_RX_IDLE:
      err = -EBUSY;
      break;

    case RFPACKET_RX_STARTED:
      rx_rq->base.pvuint = RFPACKET_RX_PREPARED;
      rx->buf = buffer;
      rx->size = size;
      err = 0;
      break;

    default:
      UNREACHABLE();
    }
  LOCK_RELEASE_IRQ(&ctx->lock);

  return err;

}
#endif

// Private rfpacket fsm feature functions prototypes
static inline void rfpacket_set_state(struct dev_rfpacket_ctx_s *pv, enum dev_rfpacket_state_s state);
static inline void rfpacket_check_wakeup(struct dev_rfpacket_ctx_s *pv);
static inline void rfpacket_process_group(struct dev_rfpacket_ctx_s *pv, bool_t group);
static inline void rfpacket_end_txrq(struct dev_rfpacket_ctx_s *pv);
static inline void rfpacket_end_rxrq(struct dev_rfpacket_ctx_s *pv);
static void rfpacket_end_rxc(struct dev_rfpacket_ctx_s *pv, error_t err);
static void rfpacket_end_rq(struct dev_rfpacket_ctx_s *pv, error_t err);
static inline void rfpacket_start_rx(struct dev_rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq);
static inline void rfpacket_retry_rx(struct dev_rfpacket_ctx_s *pv);
static inline void rfpacket_start_tx(struct dev_rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq);
static inline void rfpacket_retry_tx(struct dev_rfpacket_ctx_s *pv, bool_t restart);
static inline void rfpacket_error(struct dev_rfpacket_ctx_s *pv);
static error_t rfpacket_check_config(struct dev_rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq);
static void rfpacket_idle(struct dev_rfpacket_ctx_s *pv);

static inline void rfpacket_set_state(struct dev_rfpacket_ctx_s *pv, enum dev_rfpacket_state_s state) {
  logk_trace("state %d", state);
  pv->state = state;
}

// Transceiver is sleeping when this function is called
static inline void rfpacket_check_wakeup(struct dev_rfpacket_ctx_s *pv) {
  bool_t empty = (dev_rq_queue_isempty(&pv->queue) && dev_rq_queue_isempty(&pv->rx_cont_queue));

  if (!empty) {
    if (pv->drv->wakeup(pv)) {
      rfpacket_set_state(pv, DEV_RFPACKET_STATE_AWAKING);
    } else {
      UNREACHABLE();
    }
  } else {
    logk_trace("sleeping");
  }
}

static inline void rfpacket_process_group(struct dev_rfpacket_ctx_s *pv, bool_t group) {
  while (1) {
    struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_head(&pv->queue);

    if (!rq || rq->err_group != group) {
      break;
    }
    assert((rq->type != DEV_RFPACKET_RQ_RX_CONT) && (rq->type != DEV_RFPACKET_RQ_RX_TIMEOUT));
    rq->error = -ECANCELED;
    rq->base.drvdata = NULL;
    dev_rfpacket_rq_pop(&pv->queue);
    dev_rfpacket_rq_done(rq);
  }
}

static inline void rfpacket_end_txrq(struct dev_rfpacket_ctx_s *pv) {
  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_head(&pv->queue);

#ifdef CONFIG_DEVICE_RFPACKET_STATISTICS
  pv->stats.tx_count++;
#endif
  rq->tx_timestamp = pv->timestamp;
  if (rq->anchor == DEV_RFPACKET_TIMESTAMP_START) {
    rq->tx_timestamp -= rq->tx_size * pv->time_byte;
  }
  if (rq->type == DEV_RFPACKET_RQ_TX_FAIR) {
    rq->tx_lbt_td = pv->timestamp - (rq->tx_size * pv->time_byte) - pv->lbt_timestamp;
  }
}

static inline void rfpacket_end_rxrq(struct dev_rfpacket_ctx_s *pv) {
  struct dev_rfpacket_rx_s *rx = pv->rxrq;
  error_t err = 0;

  if (rx == NULL) {
    return;
  }
  if (pv->status == DEV_RFPACKET_STATUS_CRC_ERR) {
    err = -EBADDATA;
  } else if (pv->status == DEV_RFPACKET_STATUS_OTHER_ERR) {
    err = -EIO;
  }
#ifdef CONFIG_DEVICE_RFPACKET_STATISTICS
  pv->stats.rx_count++;
  if (err) {
    pv->stats.rx_err_count++;
  }
#endif
  if (!err) {
    // Calculate timestamp
    rx->timestamp = pv->timestamp - (rx->timestamp * rx->size);
  } else {
    rx->size = 0;
  }
  rx->error = err;
  kroutine_exec(&rx->kr);
  pv->rxrq = NULL;
}

static void rfpacket_end_rxc(struct dev_rfpacket_ctx_s *pv, error_t err) {
  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_head(&pv->rx_cont_queue);

  switch (pv->state) {
    case DEV_RFPACKET_STATE_TX_LBT_STOPPING_RXC:
    case DEV_RFPACKET_STATE_RXC:
    case DEV_RFPACKET_STATE_CONFIG_RXC:
    case DEV_RFPACKET_STATE_CONFIG_RXC_PENDING_STOP:
    case DEV_RFPACKET_STATE_STOPPING_RXC:
    case DEV_RFPACKET_STATE_PAUSE_RXC:
      assert(rq);
      rq->error = err;
      rq->base.drvdata = NULL;
      dev_rfpacket_rq_remove(&pv->rx_cont_queue, rq);
      dev_rfpacket_rq_done(rq);
      rfpacket_idle(pv);
    break;

    default:
      UNREACHABLE();
  }
}

static void rfpacket_end_rq(struct dev_rfpacket_ctx_s *pv, error_t err) {
  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_head(&pv->queue);

  assert(rq && (rq->type != DEV_RFPACKET_RQ_RX_CONT) && (rq->type != DEV_RFPACKET_RQ_RX_TIMEOUT));
  rq->error = err;
  rq->base.drvdata = NULL;
  dev_rfpacket_rq_pop(&pv->queue);
  dev_rfpacket_rq_done(rq);

  if (rq->error) {
    rfpacket_process_group(pv, rq->err_group);
  }
  switch (pv->state) {
    case DEV_RFPACKET_STATE_TX_LBT_STOPPING_RXC:
      return rfpacket_end_rxc(pv, 0);
    default:
      return rfpacket_idle(pv);
  }
}

static inline void rfpacket_start_rx(struct dev_rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  assert(rq && (pv->state == DEV_RFPACKET_STATE_READY));
  pv->rq = rq;
  // Get timer value
  dev_timer_value_t t;
  DEVICE_OP(pv->timer, get_value, &t, 0);

  switch (rq->type) {
    case DEV_RFPACKET_RQ_RX:
      logk_trace("R");
      rfpacket_set_state(pv, DEV_RFPACKET_STATE_RX);
      pv->deadline = rq->deadline ? rq->deadline : t;
      pv->timeout = pv->deadline + rq->lifetime;
    break;

    case DEV_RFPACKET_RQ_RX_TIMEOUT:
      logk_trace("RT");
      rfpacket_set_state(pv, DEV_RFPACKET_STATE_RXC);
      pv->rxc_timeout = rq->deadline;
    break;

    case DEV_RFPACKET_RQ_RX_CONT:
      logk_trace("RC");
      rfpacket_set_state(pv, DEV_RFPACKET_STATE_RXC);
    break;

    default:
      UNREACHABLE();
  }
  pv->drv->rx(pv, rq, false);
}

static inline void rfpacket_retry_rx(struct dev_rfpacket_ctx_s *pv) {
  struct dev_rfpacket_rq_s * rq = dev_rfpacket_rq_head(&pv->queue);
  assert(pv->state == DEV_RFPACKET_STATE_RX);
  assert(rq && (rq->type == DEV_RFPACKET_RQ_RX));
  pv->rq = rq;

  // Get timer value
  dev_timer_value_t t;
  DEVICE_OP(pv->timer, get_value, &t, 0);
  // Check for timeout
  if (t >= pv->timeout) {
    rfpacket_end_rq(pv, 0);
    return;
  }
  // Ask driver to rx
  pv->drv->rx(pv, rq, true);
}

static inline void rfpacket_start_tx(struct dev_rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  assert(rq && (rq->tx_size < DEV_RFPACKET_MAX_PACKET_SIZE));
  assert(pv->state == DEV_RFPACKET_STATE_READY);
  pv->rq = rq;
  pv->size = rq->tx_size;
  pv->buffer = (uint8_t *)rq->tx_buf;
  // Get timer value
  dev_timer_value_t t;
  DEVICE_OP(pv->timer, get_value, &t, 0);
  // Set deadline
  pv->deadline = rq->deadline ? rq->deadline : t;

  switch (rq->type) {
    case DEV_RFPACKET_RQ_TX_FAIR:
      pv->timeout = pv->deadline + rq->lifetime;
      logk_trace("TF");
      if (t >= pv->timeout) {
        // Timeout date is already reached
        return rfpacket_end_rq(pv, -ETIMEDOUT);
      }
      rfpacket_set_state(pv, DEV_RFPACKET_STATE_TX_LBT);
      // Ask driver to tx
      pv->drv->tx(pv, rq, false);
    break;

    case DEV_RFPACKET_RQ_TX:
      logk_trace("T");
      rfpacket_set_state(pv, DEV_RFPACKET_STATE_TX);
      // Ask driver to tx
      pv->drv->tx(pv, rq, false);
    break;

    default:
      UNREACHABLE();
  }
}

// TX with LBT has been interrupted
static inline void rfpacket_retry_tx(struct dev_rfpacket_ctx_s *pv, bool_t restart) {
  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_head(&pv->queue);
  assert(rq && rq->type == DEV_RFPACKET_RQ_TX_FAIR);

  pv->rq = rq;
  pv->size = rq->tx_size;
  pv->buffer = (uint8_t *)rq->tx_buf;

  switch (pv->state) {
    case DEV_RFPACKET_STATE_TX_LBT:
    case DEV_RFPACKET_STATE_TX_LBT_STOPPING_RXC: {
      // Get time value
      dev_timer_value_t t;
      DEVICE_OP(pv->timer, get_value, &t, 0);
      if (t >= pv->timeout) {
        // Timeout date is already reached
        rfpacket_end_rq(pv, -ETIMEDOUT);
        return;
      }
      if (restart) {
        // TX has been interrupted by an error
        pv->drv->tx(pv, rq, false);
      } else {
        // TX has been interrupted by a received packet
        pv->drv->tx(pv, rq, true);
      }
    break; }

    default:
      UNREACHABLE();
  }
}

// Transceiver is idle when this function is called
static inline void rfpacket_error(struct dev_rfpacket_ctx_s *pv) {
  logk_trace("-EIO error %d", pv->state);
  // Terminate allocated rx request
  rfpacket_end_rxrq(pv);

  switch (pv->state) {
    case DEV_RFPACKET_STATE_TX:
      rfpacket_end_rq(pv, -EIO);
      return;
    case DEV_RFPACKET_STATE_RX:
      rfpacket_retry_rx(pv);
      return;
    case DEV_RFPACKET_STATE_TX_LBT:
    case DEV_RFPACKET_STATE_TX_LBT_STOPPING_RXC:
      rfpacket_retry_tx(pv, true);
      return;
    case DEV_RFPACKET_STATE_STOPPING_RXC:
    case DEV_RFPACKET_STATE_PAUSE_RXC:
      rfpacket_end_rxc(pv, 0);
      return;
    case DEV_RFPACKET_STATE_RXC:
      rfpacket_idle(pv);
      return;
    default:
      UNREACHABLE();
  }
}

static error_t rfpacket_check_config(struct dev_rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  switch (rq->type) {
    case DEV_RFPACKET_RQ_RX_TIMEOUT: {
      // Set state to pass assert in case of timeout
      rfpacket_set_state(pv, DEV_RFPACKET_STATE_CONFIG_RXC);
      // Get time value
      dev_timer_value_t t;
      DEVICE_OP(pv->timer, get_value, &t, 0);
      if (t >= rq->deadline) {
        // Timeout date is already reached
        return -ETIMEDOUT;
      }
    break; }

    case DEV_RFPACKET_RQ_RX_CONT:
      rfpacket_set_state(pv, DEV_RFPACKET_STATE_CONFIG_RXC);
    break;

    default:
      rfpacket_set_state(pv, DEV_RFPACKET_STATE_CONFIG);
    break;
  }
  return pv->drv->check_config(pv, rq);
}

// Transceiver is idle
static void rfpacket_idle(struct dev_rfpacket_ctx_s *pv) {
  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_head(&pv->queue);
  rfpacket_set_state(pv, DEV_RFPACKET_STATE_READY);

  if (!rq) {
    rq = dev_rfpacket_rq_head(&pv->rx_cont_queue);
  }
  if (!rq) {
    // No request to do
    pv->drv->idle(pv);
    return;
  }
  pv->rq = rq;
  logk_trace("idle %d", rq->type);

  // Check transceiver configuration
  switch (rfpacket_check_config(pv, rq)) {
    case -EAGAIN:
      // Configuration is being applied
      return;
    case -ENOTSUP:
      // Unsupported configuration
      dev_rfpacket_config_notsup(pv, rq);
      return;
    case -ETIMEDOUT:
      // RQ timeout elapsed
      rfpacket_end_rxc(pv, -ETIMEDOUT);
      return;
    default:
      // Configuration is already applied
      rfpacket_set_state(pv, DEV_RFPACKET_STATE_READY);
    break;
  }
  switch (rq->type) {
    case DEV_RFPACKET_RQ_TX_FAIR:
      pv->lbt_timestamp = 0;
    case DEV_RFPACKET_RQ_TX:
      rfpacket_start_tx(pv, rq);
    break;

    case DEV_RFPACKET_RQ_RX:
    case DEV_RFPACKET_RQ_RX_CONT:
    case DEV_RFPACKET_RQ_RX_TIMEOUT:
      rfpacket_start_rx(pv, rq);
    break;

    default:
      return rfpacket_end_rq(pv, -ENOTSUP);
    break;
  }
}



config_depend(CONFIG_DEVICE_RFPACKET)
bool_t dev_rfpacket_init_done(struct dev_rfpacket_ctx_s *pv) {
  return (pv->state != DEV_RFPACKET_STATE_INITIALISING);
}

config_depend(CONFIG_DEVICE_RFPACKET)
void dev_rfpacket_config_notsup(struct dev_rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  switch (rq->type) {
    case DEV_RFPACKET_RQ_RX_CONT:
    case DEV_RFPACKET_RQ_RX_TIMEOUT:
      rfpacket_end_rxc(pv, -ENOTSUP);
    break;

    default:
      rfpacket_end_rq(pv, -ENOTSUP);
    break;
  }
}

config_depend(CONFIG_DEVICE_RFPACKET)
bool_t dev_rfpacket_config_state_check(struct dev_rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  if (rq->type == DEV_RFPACKET_RQ_RX_CONT || rq->type == DEV_RFPACKET_RQ_RX_TIMEOUT) {
    return((pv->state == DEV_RFPACKET_STATE_CONFIG_RXC)
      || (pv->state == DEV_RFPACKET_STATE_CONFIG_RXC_PENDING_STOP));
  } else {
    return(pv->state == DEV_RFPACKET_STATE_CONFIG);
  }
}

config_depend(CONFIG_DEVICE_RFPACKET)
bool_t dev_rfpacket_can_rxtx(struct dev_rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
    struct dev_rfpacket_rq_s *rx_cont = dev_rfpacket_rq_head(&pv->rx_cont_queue);
    // Check if possible to rx while txlbt (rq config and rx_cont config are the same)
    if (rx_cont && (rq->rf_cfg == rx_cont->rf_cfg) && (rq->pk_cfg == rx_cont->pk_cfg)) {
      return true;
    }
    return false;
}

config_depend(CONFIG_DEVICE_RFPACKET)
void dev_rfpacket_request(struct dev_rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  rq->error = 0;

  switch (rq->type) {
    case DEV_RFPACKET_RQ_RX_TIMEOUT:
      if (rq->lifetime != 0) {
        dev_timer_value_t t;
        DEVICE_OP(pv->timer, get_value, &t, 0);
        rq->deadline = t + rq->lifetime;
      }
    case DEV_RFPACKET_RQ_RX_CONT:
      rq->base.drvdata = pv;
      if (rq->type == DEV_RFPACKET_RQ_RX_CONT) {
        rq->deadline = -1;
      }
      switch (pv->state) {
        case DEV_RFPACKET_STATE_READY:
          assert(dev_rq_queue_isempty(&pv->rx_cont_queue));
          dev_rfpacket_rq_insert(&pv->rx_cont_queue, rq);
          rfpacket_idle(pv);
        break;

        case DEV_RFPACKET_STATE_SLEEP:
          assert(dev_rq_queue_isempty(&pv->rx_cont_queue));
          if (pv->drv->wakeup(pv)) {
            rfpacket_set_state(pv, DEV_RFPACKET_STATE_AWAKING);
          } else {
            UNREACHABLE();
          }
        case DEV_RFPACKET_STATE_ENTER_SLEEP:
        case DEV_RFPACKET_STATE_AWAKING:
        case DEV_RFPACKET_STATE_CONFIG:
        case DEV_RFPACKET_STATE_RX:
        case DEV_RFPACKET_STATE_TX:
        case DEV_RFPACKET_STATE_TX_LBT:
        case DEV_RFPACKET_STATE_STOPPING_RXC:
        case DEV_RFPACKET_STATE_PAUSE_RXC:
        case DEV_RFPACKET_STATE_CONFIG_RXC_PENDING_STOP:
        case DEV_RFPACKET_STATE_CONFIG_RXC:
        case DEV_RFPACKET_STATE_TX_LBT_STOPPING_RXC:
          dev_rfpacket_rq_insert(&pv->rx_cont_queue, rq);
        break;

        case DEV_RFPACKET_STATE_RXC:
          dev_rfpacket_rq_insert(&pv->rx_cont_queue, rq);
          // TODO share the radio when using the same configuration
          if (rq == dev_rfpacket_rq_head(&pv->rx_cont_queue)) {
            rfpacket_set_state(pv, DEV_RFPACKET_STATE_PAUSE_RXC);
            pv->drv->cancel_rxc(pv);
          }
        break;

        case DEV_RFPACKET_STATE_INITIALISING:
          UNREACHABLE();
      }
      break;

    case DEV_RFPACKET_RQ_RX:
    case DEV_RFPACKET_RQ_TX_FAIR:
    case DEV_RFPACKET_RQ_TX: {
      bool_t empty = dev_rq_queue_isempty(&pv->queue);
      dev_rfpacket_rq_pushback(&pv->queue, rq);
      rq->base.drvdata = pv;

      if (!empty) {
        break;
      }
      switch (pv->state) {
        case DEV_RFPACKET_STATE_RXC:
          assert(!dev_rq_queue_isempty(&pv->rx_cont_queue));
          assert(rq->deadline == 0);
          rfpacket_set_state(pv, DEV_RFPACKET_STATE_PAUSE_RXC);
          pv->drv->cancel_rxc(pv);
        break;

        case DEV_RFPACKET_STATE_READY:
          rfpacket_idle(pv);
        break;

        case DEV_RFPACKET_STATE_SLEEP:
          if (pv->drv->wakeup(pv)) {
            rfpacket_set_state(pv, DEV_RFPACKET_STATE_AWAKING);
          } else {
            UNREACHABLE();
          }
        case DEV_RFPACKET_STATE_ENTER_SLEEP:
        default:
        break;

        case DEV_RFPACKET_STATE_INITIALISING:
          UNREACHABLE();
      }
    }
  }
}

config_depend(CONFIG_DEVICE_RFPACKET)
error_t dev_rfpacket_cancel(struct dev_rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  struct dev_rfpacket_rq_s *hrq = dev_rfpacket_rq_head(&pv->queue);
  struct dev_rfpacket_rq_s *hrqcont = dev_rfpacket_rq_head(&pv->rx_cont_queue);
  error_t err = -EBUSY;

  if (rq == hrqcont) {
    switch (pv->state) {
      case DEV_RFPACKET_STATE_CONFIG_RXC:
        rfpacket_set_state(pv, DEV_RFPACKET_STATE_CONFIG_RXC_PENDING_STOP);
      break;

      case DEV_RFPACKET_STATE_RXC:
        pv->drv->cancel_rxc(pv);
      case DEV_RFPACKET_STATE_PAUSE_RXC:
        rfpacket_set_state(pv, DEV_RFPACKET_STATE_STOPPING_RXC);
      case DEV_RFPACKET_STATE_STOPPING_RXC:
      break;

      case DEV_RFPACKET_STATE_TX_LBT:
        rfpacket_set_state(pv, DEV_RFPACKET_STATE_TX_LBT_STOPPING_RXC);
      break;

      case DEV_RFPACKET_STATE_CONFIG_RXC_PENDING_STOP:
      case DEV_RFPACKET_STATE_TX_LBT_STOPPING_RXC:
      break;

      default:
        err = 0;
        rq->base.drvdata = NULL;
        dev_rfpacket_rq_remove(&pv->rx_cont_queue, rq);
      break;
    }
  } else if ((rq->base.drvdata == pv) && (rq != hrq)) {
    // Request is in queue and is not being processed
    err = 0;
    rq->base.drvdata = NULL;
    switch (rq->type) {
      case DEV_RFPACKET_RQ_TX:
      case DEV_RFPACKET_RQ_TX_FAIR:
      case DEV_RFPACKET_RQ_RX:
        dev_rfpacket_rq_remove(&pv->queue, rq);
      break;

      case DEV_RFPACKET_RQ_RX_CONT:
      case DEV_RFPACKET_RQ_RX_TIMEOUT:
        dev_rfpacket_rq_remove(&pv->rx_cont_queue, rq);
      break;
    }
  }
  return err;
}

config_depend(CONFIG_DEVICE_RFPACKET)
uintptr_t dev_rfpacket_alloc(struct dev_rfpacket_ctx_s *pv) {
  struct dev_rfpacket_rq_s *rq;

  switch (pv->state) {
    case DEV_RFPACKET_STATE_RX:
      rq = dev_rfpacket_rq_head(&pv->queue);
    break;

    case DEV_RFPACKET_STATE_TX_LBT:
    case DEV_RFPACKET_STATE_TX_LBT_STOPPING_RXC:
    case DEV_RFPACKET_STATE_PAUSE_RXC:
    case DEV_RFPACKET_STATE_RXC:
      rq = dev_rfpacket_rq_head(&pv->rx_cont_queue);
    break;

    case DEV_RFPACKET_STATE_STOPPING_RXC:
      // RX continous is being cancelled
      rq = NULL;
    break;

    default:
      UNREACHABLE();
  }
  if (rq == NULL) {
    return 0;
  }
  struct dev_rfpacket_rx_s *rx = rq->rx_alloc(rq, pv->size);
  if (rx == NULL) {
    return 0;
  }
  rx->channel = rq->channel;
  // Note anchor status for end rxrq function
  if (rq->anchor == DEV_RFPACKET_TIMESTAMP_START) {
    rx->timestamp = pv->time_byte;
  } else {
    rx->timestamp = 0;
  }
  pv->rxrq = rx;
  pv->buffer = (uint8_t*)rx->buf;

  assert(rx->size == pv->size);
  logk_trace("%d", pv->size);
  return (uintptr_t)pv->buffer;
}

config_depend(CONFIG_DEVICE_RFPACKET)
void dev_rfpacket_req_done(struct device_s *dev, struct dev_rfpacket_ctx_s *pv) {
  if (pv->status == DEV_RFPACKET_STATUS_OTHER_ERR) {
      rfpacket_error(pv);
      return;
  }
  switch (pv->state) {
    case DEV_RFPACKET_STATE_ENTER_SLEEP:
      rfpacket_set_state(pv, DEV_RFPACKET_STATE_SLEEP);
      rfpacket_check_wakeup(pv);
    break;

    case DEV_RFPACKET_STATE_AWAKING:
      logk_trace("awaken");
      rfpacket_idle(pv);
    break;

    case DEV_RFPACKET_STATE_INITIALISING:
      device_async_init_done(dev, 0);
    case DEV_RFPACKET_STATE_CONFIG_RXC:
    case DEV_RFPACKET_STATE_CONFIG:
      rfpacket_idle(pv);
    break;

    case DEV_RFPACKET_STATE_RX:
      if (pv->status == DEV_RFPACKET_STATUS_RX_TIMEOUT) {
        rfpacket_end_rxrq(pv);
        rfpacket_end_rq(pv, 0);
      } else { // Rx related event
        rfpacket_end_rxrq(pv);
        rfpacket_retry_rx(pv);
      }
    break;

    case DEV_RFPACKET_STATE_PAUSE_RXC:
    case DEV_RFPACKET_STATE_RXC:
      if (pv->status == DEV_RFPACKET_STATUS_JAMMING_ERR) {
        logk_trace("Jamming");
        assert(pv->rxrq == NULL);
        rfpacket_end_rxc(pv, -EAGAIN);
      } else if (pv->status == DEV_RFPACKET_STATUS_RX_TIMEOUT) {
        rfpacket_end_rxrq(pv);
        rfpacket_end_rxc(pv, 0);
      } else { // Rx related event
        rfpacket_end_rxrq(pv);
        rfpacket_idle(pv);
      }
    break;

    case DEV_RFPACKET_STATE_TX_LBT:
    case DEV_RFPACKET_STATE_TX_LBT_STOPPING_RXC:
      if (pv->status == DEV_RFPACKET_STATUS_TX_TIMEOUT) {
#ifdef CONFIG_DEVICE_RFPACKET_STATISTICS
        pv->stats.tx_err_count++;
#endif
        rfpacket_end_rq(pv, -ETIMEDOUT);
      } else if (pv->status == DEV_RFPACKET_STATUS_TX_DONE) {
        // Packet has been transmitted
        rfpacket_end_txrq(pv);
        rfpacket_end_rq(pv, 0);
      } else { // Rx related event
        rfpacket_end_rxrq(pv);
        rfpacket_retry_tx(pv, false);
      }
    break;

    case DEV_RFPACKET_STATE_TX:
      // Packet has been transmitted
      rfpacket_end_txrq(pv);
      rfpacket_end_rq(pv, 0);
    break;

    case DEV_RFPACKET_STATE_STOPPING_RXC:
      rfpacket_end_rxrq(pv);
    case DEV_RFPACKET_STATE_CONFIG_RXC_PENDING_STOP:
      rfpacket_end_rxc(pv, 0);
    break;

    case DEV_RFPACKET_STATE_SLEEP:
    default:
      UNREACHABLE();
  }
}

config_depend(CONFIG_DEVICE_RFPACKET)
error_t dev_rfpacket_use(void *param, enum dev_use_op_e op, struct dev_rfpacket_ctx_s *pv) {
  switch (op) {
    case DEV_USE_SLEEP:
      switch (pv->state) {
        case DEV_RFPACKET_STATE_READY:
          logk_trace("sleep");
          if (pv->drv->sleep(pv)) {
            rfpacket_set_state(pv, DEV_RFPACKET_STATE_ENTER_SLEEP);
          }
          break;

        default:
          break;
      }
    default:
      return dev_use_generic(param, op);
    break;
  }
}

config_depend(CONFIG_DEVICE_RFPACKET)
void dev_rfpacket_init(struct dev_rfpacket_ctx_s *pv) {
  // State init
  rfpacket_set_state(pv, DEV_RFPACKET_STATE_INITIALISING);
  // Queue init
  dev_rq_queue_init(&pv->queue);
  dev_rq_queue_init(&pv->rx_cont_queue);
  // Check key structures were filled by driver
  assert(pv->drv);
  assert(pv->timer);
}

config_depend(CONFIG_DEVICE_RFPACKET)
error_t dev_rfpacket_clean_check(struct dev_rfpacket_ctx_s *pv) {
  switch (pv->state) {
    case DEV_RFPACKET_STATE_SLEEP:
    case DEV_RFPACKET_STATE_READY:
      assert(dev_rq_queue_isempty(&pv->queue));
    break;

    default:
      return -EBUSY;
  }
  return 0;
}
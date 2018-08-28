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
  struct dev_rfpacket_rq_s *rx_rq = &ctx->rx_rq;
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

#include <mutek/printk.h>

#include <stdbool.h>

#include "rfpacket_core.h"

// Private functions prototypes
static inline void rfpacket_set_state(struct rfpacket_ctx_s *pv, enum rfpacket_state_s state);
static inline void rfpacket_check_wakeup(struct rfpacket_ctx_s *pv);
static inline void rfpacket_process_group(struct rfpacket_ctx_s *pv, bool_t group);
static inline void rfpacket_end_txrq(struct rfpacket_ctx_s *pv);
static inline void rfpacket_end_rxrq(struct rfpacket_ctx_s *pv);
static void rfpacket_end_rxc(struct rfpacket_ctx_s *pv, error_t err);
static void rfpacket_end_rq(struct rfpacket_ctx_s *pv, error_t err);
static inline void rfpacket_start_rx(struct rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq);
static inline void rfpacket_retry_rx(struct rfpacket_ctx_s *pv);
static inline void rfpacket_start_tx(struct rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq);
static inline void rfpacket_retry_tx(struct rfpacket_ctx_s *pv, bool_t restart);
static inline void rfpacket_error(struct rfpacket_ctx_s *pv);
static error_t rfpacket_check_config(struct rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq);
static void rfpacket_idle(struct rfpacket_ctx_s *pv);






static inline void rfpacket_set_state(struct rfpacket_ctx_s *pv, enum rfpacket_state_s state) {
  logk_trace("state %d", state);
  pv->state = state;
}

// Transceiver is sleeping when this function is called
static inline void rfpacket_check_wakeup(struct rfpacket_ctx_s *pv) {
  bool_t empty = (dev_rq_queue_isempty(&pv->queue) && dev_rq_queue_isempty(&pv->rx_cont_queue));

  if (!empty) {
    if (pv->drv->wakeup(pv)) {
      rfpacket_set_state(pv, RFPACKET_STATE_AWAKING);
    } else {
      UNREACHABLE();
    }
  } else {
    logk_trace("sleeping");
  }
}

static inline void rfpacket_process_group(struct rfpacket_ctx_s *pv, bool_t group) {
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

static inline void rfpacket_end_txrq(struct rfpacket_ctx_s *pv) {
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

static inline void rfpacket_end_rxrq(struct rfpacket_ctx_s *pv) {
  struct dev_rfpacket_rx_s *rx = pv->rxrq;
  error_t err = 0;

  if (rx == NULL) {
    return;
  }
  if (pv->status == RFPACKET_STATUS_CRC_ERR) {
    err = -EBADDATA;
  } else if (pv->status == RFPACKET_STATUS_OTHER_ERR) {
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

static void rfpacket_end_rxc(struct rfpacket_ctx_s *pv, error_t err) {
  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_head(&pv->rx_cont_queue);

  switch (pv->state) {
    case RFPACKET_STATE_TX_LBT_STOPPING_RXC:
    case RFPACKET_STATE_RXC:
    case RFPACKET_STATE_CONFIG_RXC:
    case RFPACKET_STATE_CONFIG_RXC_PENDING_STOP:
    case RFPACKET_STATE_STOPPING_RXC:
    case RFPACKET_STATE_PAUSE_RXC:
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

static void rfpacket_end_rq(struct rfpacket_ctx_s *pv, error_t err) {
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
    case RFPACKET_STATE_TX_LBT_STOPPING_RXC:
      return rfpacket_end_rxc(pv, 0);
    default:
      return rfpacket_idle(pv);
  }
}

static inline void rfpacket_start_rx(struct rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  assert(rq && (pv->state == RFPACKET_STATE_READY));
  pv->rq = rq;
  // Get timer value
  dev_timer_value_t t;
  DEVICE_OP(pv->timer, get_value, &t, 0);

  switch (rq->type) {
    case DEV_RFPACKET_RQ_RX:
      logk_trace("R");
      rfpacket_set_state(pv, RFPACKET_STATE_RX);
      pv->deadline = rq->deadline ? rq->deadline : t;
      pv->timeout = pv->deadline + rq->lifetime;
    break;

    case DEV_RFPACKET_RQ_RX_TIMEOUT:
      logk_trace("RT");
      rfpacket_set_state(pv, RFPACKET_STATE_RXC);
      pv->rxc_timeout = rq->deadline;
    break;

    case DEV_RFPACKET_RQ_RX_CONT:
      logk_trace("RC");
      rfpacket_set_state(pv, RFPACKET_STATE_RXC);
    break;

    default:
      UNREACHABLE();
  }
  pv->drv->rx(pv, rq, false);
}

static inline void rfpacket_retry_rx(struct rfpacket_ctx_s *pv) {
  struct dev_rfpacket_rq_s * rq = dev_rfpacket_rq_head(&pv->queue);
  assert(pv->state == RFPACKET_STATE_RX);
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

static inline void rfpacket_start_tx(struct rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  assert(rq && (rq->tx_size < RFPACKET_MAX_PACKET_SIZE));
  assert(pv->state == RFPACKET_STATE_READY);
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
      rfpacket_set_state(pv, RFPACKET_STATE_TX_LBT);
      // Ask driver to tx
      pv->drv->tx(pv, rq, false);
    break;

    case DEV_RFPACKET_RQ_TX:
      logk_trace("T");
      rfpacket_set_state(pv, RFPACKET_STATE_TX);
      // Ask driver to tx
      pv->drv->tx(pv, rq, false);
    break;

    default:
      UNREACHABLE();
  }
}

// TX with LBT has been interrupted
static inline void rfpacket_retry_tx(struct rfpacket_ctx_s *pv, bool_t restart) {
  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_head(&pv->queue);
  assert(rq && rq->type == DEV_RFPACKET_RQ_TX_FAIR);

  pv->rq = rq;
  pv->size = rq->tx_size;
  pv->buffer = (uint8_t *)rq->tx_buf;

  switch (pv->state) {
    case RFPACKET_STATE_TX_LBT:
    case RFPACKET_STATE_TX_LBT_STOPPING_RXC: {
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
static inline void rfpacket_error(struct rfpacket_ctx_s *pv) {
  logk_trace("-EIO error %d", pv->state);
  // Terminate allocated rx request
  rfpacket_end_rxrq(pv);

  switch (pv->state) {
    case RFPACKET_STATE_TX:
      rfpacket_end_rq(pv, -EIO);
      return;
    case RFPACKET_STATE_RX:
      rfpacket_retry_rx(pv);
      return;
    case RFPACKET_STATE_TX_LBT:
    case RFPACKET_STATE_TX_LBT_STOPPING_RXC:
      rfpacket_retry_tx(pv, true);
      return;
    case RFPACKET_STATE_STOPPING_RXC:
    case RFPACKET_STATE_PAUSE_RXC:
      rfpacket_end_rxc(pv, 0);
      return;
    case RFPACKET_STATE_RXC:
      rfpacket_idle(pv);
      return;
    default:
      UNREACHABLE();
  }
}

static error_t rfpacket_check_config(struct rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  switch (rq->type) {
    case DEV_RFPACKET_RQ_RX_TIMEOUT: {
      // Set state to pass assert in case of timeout
      rfpacket_set_state(pv, RFPACKET_STATE_CONFIG_RXC);
      // Get time value
      dev_timer_value_t t;
      DEVICE_OP(pv->timer, get_value, &t, 0);
      if (t >= rq->deadline) {
        // Timeout date is already reached
        return -ETIMEDOUT;
      }
    break; }

    case DEV_RFPACKET_RQ_RX_CONT:
      rfpacket_set_state(pv, RFPACKET_STATE_CONFIG_RXC);
    break;

    default:
      rfpacket_set_state(pv, RFPACKET_STATE_CONFIG);
    break;
  }
  return pv->drv->check_config(pv, rq);
}

// Transceiver is idle
static void rfpacket_idle(struct rfpacket_ctx_s *pv) {
  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_head(&pv->queue);
  rfpacket_set_state(pv, RFPACKET_STATE_READY);

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
      rfpacket_config_notsup(pv, rq);
      return;
    case -ETIMEDOUT:
      // RQ timeout elapsed
      rfpacket_end_rxc(pv, -ETIMEDOUT);
      return;
    default:
      // Configuration is already applied
      rfpacket_set_state(pv, RFPACKET_STATE_READY);
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






bool_t rfpacket_init_done(struct rfpacket_ctx_s *pv) {
  return (pv->state != RFPACKET_STATE_INITIALISING);
}

void rfpacket_config_notsup(struct rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
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

bool_t rfpacket_config_state_check(struct rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  if (rq->type == DEV_RFPACKET_RQ_RX_CONT || rq->type == DEV_RFPACKET_RQ_RX_TIMEOUT) {
    return((pv->state == RFPACKET_STATE_CONFIG_RXC)
      || (pv->state == RFPACKET_STATE_CONFIG_RXC_PENDING_STOP));
  } else {
    return(pv->state == RFPACKET_STATE_CONFIG);
  }
}

bool_t rfpacket_can_rxtx(struct rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
    struct dev_rfpacket_rq_s *rx_cont = dev_rfpacket_rq_head(&pv->rx_cont_queue);
    // Check if possible to rx while txlbt (rq config and rx_cont config are the same)
    if (rx_cont && (rq->rf_cfg == rx_cont->rf_cfg) && (rq->pk_cfg == rx_cont->pk_cfg)) {
      return true;
    }
    return false;
}

void rfpacket_request(struct rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
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
        case RFPACKET_STATE_READY:
          assert(dev_rq_queue_isempty(&pv->rx_cont_queue));
          dev_rfpacket_rq_insert(&pv->rx_cont_queue, rq);
          rfpacket_idle(pv);
        break;

        case RFPACKET_STATE_SLEEP:
          assert(dev_rq_queue_isempty(&pv->rx_cont_queue));
          if (pv->drv->wakeup(pv)) {
            rfpacket_set_state(pv, RFPACKET_STATE_AWAKING);
          } else {
            UNREACHABLE();
          }
        case RFPACKET_STATE_ENTER_SLEEP:
        case RFPACKET_STATE_AWAKING:
        case RFPACKET_STATE_CONFIG:
        case RFPACKET_STATE_RX:
        case RFPACKET_STATE_TX:
        case RFPACKET_STATE_TX_LBT:
        case RFPACKET_STATE_STOPPING_RXC:
        case RFPACKET_STATE_PAUSE_RXC:
        case RFPACKET_STATE_CONFIG_RXC_PENDING_STOP:
        case RFPACKET_STATE_CONFIG_RXC:
        case RFPACKET_STATE_TX_LBT_STOPPING_RXC:
          dev_rfpacket_rq_insert(&pv->rx_cont_queue, rq);
        break;

        case RFPACKET_STATE_RXC:
          dev_rfpacket_rq_insert(&pv->rx_cont_queue, rq);
          // TODO share the radio when using the same configuration
          if (rq == dev_rfpacket_rq_head(&pv->rx_cont_queue)) {
            rfpacket_set_state(pv, RFPACKET_STATE_PAUSE_RXC);
            pv->drv->cancel_rxc(pv);
          }
        break;

        case RFPACKET_STATE_INITIALISING:
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
        case RFPACKET_STATE_RXC:
          assert(!dev_rq_queue_isempty(&pv->rx_cont_queue));
          assert(rq->deadline == 0);
          rfpacket_set_state(pv, RFPACKET_STATE_PAUSE_RXC);
          pv->drv->cancel_rxc(pv);
        break;

        case RFPACKET_STATE_READY:
          rfpacket_idle(pv);
        break;

        case RFPACKET_STATE_SLEEP:
          if (pv->drv->wakeup(pv)) {
            rfpacket_set_state(pv, RFPACKET_STATE_AWAKING);
          } else {
            UNREACHABLE();
          }
        case RFPACKET_STATE_ENTER_SLEEP:
        default:
        break;

        case RFPACKET_STATE_INITIALISING:
          UNREACHABLE();
      }
    }
  }
}

error_t rfpacket_cancel(struct rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq) {
  struct dev_rfpacket_rq_s *hrq = dev_rfpacket_rq_head(&pv->queue);
  struct dev_rfpacket_rq_s *hrqcont = dev_rfpacket_rq_head(&pv->rx_cont_queue);
  error_t err = -EBUSY;

  if (rq == hrqcont) {
    switch (pv->state) {
      case RFPACKET_STATE_CONFIG_RXC:
        rfpacket_set_state(pv, RFPACKET_STATE_CONFIG_RXC_PENDING_STOP);
      break;

      case RFPACKET_STATE_RXC:
        pv->drv->cancel_rxc(pv);
      case RFPACKET_STATE_PAUSE_RXC:
        rfpacket_set_state(pv, RFPACKET_STATE_STOPPING_RXC);
      case RFPACKET_STATE_STOPPING_RXC:
      break;

      case RFPACKET_STATE_TX_LBT:
        rfpacket_set_state(pv, RFPACKET_STATE_TX_LBT_STOPPING_RXC);
      break;

      case RFPACKET_STATE_CONFIG_RXC_PENDING_STOP:
      case RFPACKET_STATE_TX_LBT_STOPPING_RXC:
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

uintptr_t rfpacket_alloc(struct rfpacket_ctx_s *pv) {
  struct dev_rfpacket_rq_s *rq;

  switch (pv->state) {
    case RFPACKET_STATE_RX:
      rq = dev_rfpacket_rq_head(&pv->queue);
    break;

    case RFPACKET_STATE_TX_LBT:
    case RFPACKET_STATE_TX_LBT_STOPPING_RXC:
    case RFPACKET_STATE_PAUSE_RXC:
    case RFPACKET_STATE_RXC:
      rq = dev_rfpacket_rq_head(&pv->rx_cont_queue);
    break;

    case RFPACKET_STATE_STOPPING_RXC:
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

void rfpacket_req_done(struct device_s *dev, struct rfpacket_ctx_s *pv) {
  if (pv->status == RFPACKET_STATUS_OTHER_ERR) {
      rfpacket_error(pv);
      return;
  }
  switch (pv->state) {
    case RFPACKET_STATE_ENTER_SLEEP:
      rfpacket_set_state(pv, RFPACKET_STATE_SLEEP);
      rfpacket_check_wakeup(pv);
    break;

    case RFPACKET_STATE_AWAKING:
      logk_trace("awaken");
      rfpacket_idle(pv);
    break;

    case RFPACKET_STATE_INITIALISING:
      device_async_init_done(dev, 0);
    case RFPACKET_STATE_CONFIG_RXC:
    case RFPACKET_STATE_CONFIG:
      rfpacket_idle(pv);
    break;

    case RFPACKET_STATE_RX:
      if (pv->status == RFPACKET_STATUS_RX_TIMEOUT) {
        rfpacket_end_rxrq(pv);
        rfpacket_end_rq(pv, 0);
      } else { // Rx related event
        rfpacket_end_rxrq(pv);
        rfpacket_retry_rx(pv);
      }
    break;

    case RFPACKET_STATE_PAUSE_RXC:
    case RFPACKET_STATE_RXC:
      if (pv->status == RFPACKET_STATUS_JAMMING_ERR) {
        logk_trace("Jamming");
        assert(pv->rxrq == NULL);
        rfpacket_end_rxc(pv, -EAGAIN);
      } else if (pv->status == RFPACKET_STATUS_RX_TIMEOUT) {
        rfpacket_end_rxrq(pv);
        rfpacket_end_rxc(pv, 0);
      } else { // Rx related event
        rfpacket_end_rxrq(pv);
        rfpacket_idle(pv);
      }
    break;

    case RFPACKET_STATE_TX_LBT:
    case RFPACKET_STATE_TX_LBT_STOPPING_RXC:
      if (pv->status == RFPACKET_STATUS_TX_TIMEOUT) {
#ifdef CONFIG_DEVICE_RFPACKET_STATISTICS
        pv->stats.tx_err_count++;
#endif
        rfpacket_end_rq(pv, -ETIMEDOUT);
      } else if (pv->status == RFPACKET_STATUS_TX_DONE) {
        // Packet has been transmitted
        rfpacket_end_txrq(pv);
        rfpacket_end_rq(pv, 0);
      } else { // Rx related event
        rfpacket_end_rxrq(pv);
        rfpacket_retry_tx(pv, false);
      }
    break;

    case RFPACKET_STATE_TX:
      // Packet has been transmitted
      rfpacket_end_txrq(pv);
      rfpacket_end_rq(pv, 0);
    break;

    case RFPACKET_STATE_STOPPING_RXC:
      rfpacket_end_rxrq(pv);
    case RFPACKET_STATE_CONFIG_RXC_PENDING_STOP:
      rfpacket_end_rxc(pv, 0);
    break;

    case RFPACKET_STATE_SLEEP:
    default:
      UNREACHABLE();
  }
}

error_t rfpacket_use(void *param, enum dev_use_op_e op, struct rfpacket_ctx_s *pv) {
  switch (op) {
    case DEV_USE_SLEEP:
      switch (pv->state) {
        case RFPACKET_STATE_READY:
          logk_trace("sleep");
          if (pv->drv->sleep(pv)) {
            rfpacket_set_state(pv, RFPACKET_STATE_ENTER_SLEEP);
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

void rfpacket_init(struct rfpacket_ctx_s *pv) {
  // State init
  rfpacket_set_state(pv, RFPACKET_STATE_INITIALISING);
  // Queue init
  dev_rq_queue_init(&pv->queue);
  dev_rq_queue_init(&pv->rx_cont_queue);
  // Check key structures were filled by driver
  assert(pv->drv);
  assert(pv->timer);
}

error_t rfpacket_clean_check(struct rfpacket_ctx_s *pv) {
  switch (pv->state) {
    case RFPACKET_STATE_SLEEP:
    case RFPACKET_STATE_READY:
      assert(dev_rq_queue_isempty(&pv->queue));
    break;

    default:
      return -EBUSY;
  }
  return 0;
}
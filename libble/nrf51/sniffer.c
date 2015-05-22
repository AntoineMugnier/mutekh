#include <mutek/thread.h>
#include <mutek/kroutine.h>
#include <mutek/printk.h>
#include <mutek/semaphore.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/class/icu.h>

#include <arch/nrf51/ppi.h>
#include <arch/nrf51/radio.h>
#include <arch/nrf51/timer.h>
#include <arch/nrf51/ficr.h>

#include <stdlib.h>
#include <stdio.h>

#include <ble/protocol/address.h>
#include <ble/protocol/advertise.h>
#include <ble/protocol/radio.h>
#include <ble/sniffer/radio.h>

static DEV_IRQ_EP_PROCESS(radio_irq);
static DEV_IRQ_EP_PROCESS(timer_irq);
static void radio_setup(struct ble_sniffer *radio);
static void radio_done_flush(struct ble_sniffer *radio);

#define TIMER_OVERFLOW 0
#define TIMER_DEADLINE 1
#define TIMER_VALUE 2
#define TIMER_TIMESTAMP 3

#define TIMER_ADDRESS 0
#define TIMER_END 1
#define TIMER_TMP 2

#define PPI_PACKET_TIMESTAMP 0
#define PPI_PACKET_ADDRESS 1
#define PPI_PACKET_END 2

#define LOWPASS_LOG2 4

#define BLE_T_PREAMBLE 56

typedef uint32_t lowpass_t;

ALWAYS_INLINE uint32_t lowpass_value_get(const lowpass_t *lowpass)
{
    return (*lowpass) >> LOWPASS_LOG2;
}

ALWAYS_INLINE void lowpass_value_reset(lowpass_t *lowpass, uint32_t val)
{
    *lowpass = val << LOWPASS_LOG2;
}

ALWAYS_INLINE void lowpass_value_accumulate(lowpass_t *lowpass, uint32_t val)
{
    *lowpass += val - lowpass_value_get(lowpass);
}

struct rx_config
{
  struct buffer_s *packet;
  struct ble_sniffer_request *rq;
};

static void config_shift(struct rx_config *dst, struct rx_config *src)
{
  CPU_INTERRUPT_SAVESTATE_DISABLE;

    if (dst->packet)
      buffer_refdec(dst->packet);

    dst->packet = src->packet;
    dst->rq = src->rq;
    src->packet = NULL;
    src->rq = NULL;

  CPU_INTERRUPT_RESTORESTATE;
}

struct ble_sniffer
{
    dev_request_queue_root_t queue;
    struct buffer_pool_s packet_pool;

  struct ble_sniffer_request *current;
  struct buffer_s *receiving;

    uint64_t start_deadline;
    uint64_t end_deadline;
    uint64_t ifs_deadline;

    uintptr_t radio, timer, timer2;
    uint32_t timer_base;
  uint16_t last_frequency;
  uint8_t last_white_iv;
  uint8_t t_ifs;
    dev_request_queue_root_t done_list;

    lowpass_t timer_irq_latency;
    lowpass_t radio_setup_time;

    struct dev_irq_ep_s radio_source;
    struct dev_irq_ep_s timer_source;

    struct rx_config configured, committed;
};

static error_t timer_init(struct ble_sniffer *radio,
                       struct device_icu_s *icu)
{
    struct dev_irq_ep_s *timer_sink;

    timer_sink = DEVICE_OP(icu, get_endpoint, DEV_IRQ_EP_SINK, NRF51_TIMER0);

    if (!timer_sink)
        return -ENOENT;

    lowpass_value_reset(&radio->timer_irq_latency, 23);

    device_irq_source_init(NULL, &radio->timer_source, 1, timer_irq,
                           DEV_IRQ_SENSE_HIGH_LEVEL);

    device_irq_ep_link(&radio->timer_source, timer_sink);

    nrf_task_trigger(radio->timer, NRF51_TIMER_STOP);
    nrf_reg_set(radio->timer, NRF51_TIMER_PRESCALER, 4);
    nrf_reg_set(radio->timer, NRF51_TIMER_MODE, NRF51_TIMER_MODE_TIMER);
    nrf_reg_set(radio->timer, NRF51_TIMER_BITMODE, NRF51_TIMER_BITMODE_32);
    nrf_reg_set(radio->timer, NRF51_TIMER_CC(TIMER_OVERFLOW), 0xffffffff);
    nrf_task_trigger(radio->timer, NRF51_TIMER_CLEAR);
    nrf_task_trigger(radio->timer, NRF51_TIMER_START);
    radio->timer_base = 0;

    nrf_it_enable(radio->timer, NRF51_TIMER_COMPARE(TIMER_OVERFLOW));
    nrf_it_enable(radio->timer, NRF51_TIMER_COMPARE(TIMER_DEADLINE));

    return DEVICE_OP(icu, enable_irq, timer_sink, 0,
                     &radio->timer_source, &radio->timer_source)
        ? 0 : -EINVAL;
}

static void timer2_init(struct ble_sniffer *radio)
{
    nrf_task_trigger(radio->timer2, NRF51_TIMER_STOP);
    nrf_reg_set(radio->timer2, NRF51_TIMER_PRESCALER, 4);
    nrf_reg_set(radio->timer2, NRF51_TIMER_MODE, NRF51_TIMER_MODE_TIMER);
    nrf_reg_set(radio->timer2, NRF51_TIMER_BITMODE, NRF51_TIMER_BITMODE_16);
    nrf_task_trigger(radio->timer2, NRF51_TIMER_START);
}

static error_t radio_init(struct ble_sniffer *radio,
                       struct device_icu_s *icu)
{
    struct dev_irq_ep_s *radio_sink;

    radio_sink = DEVICE_OP(icu, get_endpoint, DEV_IRQ_EP_SINK, NRF51_RADIO);

    if (!radio_sink)
        return -ENOENT;

    lowpass_value_reset(&radio->radio_setup_time, 194);

    device_irq_source_init(NULL, &radio->radio_source, 1, radio_irq,
                           DEV_IRQ_SENSE_HIGH_LEVEL);
    device_irq_ep_link(&radio->radio_source, radio_sink);

    nrf_reg_set(radio->radio, NRF51_RADIO_POWER, 0);

    for (uint16_t i = 0; i < 320; ++i)
        asm volatile("");

    nrf_reg_set(radio->radio, NRF51_RADIO_POWER, 1);

    for (uint16_t i = 0; i < 320; ++i)
        asm volatile("");

    for (uint8_t i = 0; i < 5; ++i)
        nrf_reg_set(radio->radio, NRF51_RADIO_OVERRIDE(i),
                    cpu_mem_read_32(NRF51_FICR_BLE_1MBIT(i)));

    nrf_reg_set(radio->radio, NRF51_RADIO_MODE, NRF51_RADIO_MODE_BLE_1MBIT);
    nrf_reg_set(radio->radio, NRF51_RADIO_BCC, 16);

    return DEVICE_OP(icu, enable_irq, radio_sink, 0,
                     &radio->radio_source, &radio->radio_source)
        ? 0 : -EINVAL;
}

int64_t ble_sniffer_time_get(const struct ble_sniffer *radio)
{
    nrf_task_trigger(radio->timer, NRF51_TIMER_CAPTURE(TIMER_VALUE));

    return ((int64_t)radio->timer_base << 32)
        + nrf_reg_get(radio->timer, NRF51_TIMER_CC(TIMER_VALUE));
}

error_t ble_sniffer_create(size_t priv_size, struct ble_sniffer **rradio)
{
    error_t err;
    struct ble_sniffer *radio = malloc(sizeof(*radio) + priv_size);
    struct device_icu_s icu;

    if (!radio)
        return -ENOMEM;

    memset(radio, 0, sizeof(*radio));

    dev_request_queue_init(&radio->queue);
    buffer_pool_init(&radio->packet_pool);

    radio->radio = nrf_peripheral_addr(NRF51_RADIO);
    radio->timer = nrf_peripheral_addr(NRF51_TIMER0);
    radio->timer2 = nrf_peripheral_addr(NRF51_TIMER1);

    dev_request_queue_init(&radio->done_list);

    err = device_get_accessor_by_path(&icu, NULL, "/cpu", DRIVER_CLASS_ICU);
    if (err)
        return err;

    err = radio_init(radio, &icu);
    if (err)
        goto err_free;

    err = timer_init(radio, &icu);
    if (err)
        goto err_radio_deinit;

    timer2_init(radio);

    radio_setup(radio);

    *rradio = radio;
    goto done;

  err_radio_deinit:
  err_free:
    free(radio);

  done:
    device_put_accessor(&icu);

    return err;
}

static void radio_deadline_set(struct ble_sniffer *radio)
{
  uint64_t deadline;

  deadline = __MIN(radio->end_deadline,
                   __MIN(radio->ifs_deadline,
                         radio->start_deadline));

  if (deadline == -1) {
    nrf_it_disable(radio->timer, NRF51_TIMER_COMPARE(TIMER_DEADLINE));
    return;
  }

  deadline -= lowpass_value_get(&radio->timer_irq_latency);
  // deadline += 1;

  int64_t now = ble_sniffer_time_get(radio);

  assert(deadline < now + 10000000);

  if (deadline < now + 2)
    deadline = now + 2 + lowpass_value_get(&radio->timer_irq_latency);

  nrf_it_enable(radio->timer, NRF51_TIMER_COMPARE(TIMER_DEADLINE));
  nrf_event_clear(radio->timer, NRF51_TIMER_COMPARE(TIMER_DEADLINE));
  nrf_reg_set(radio->timer, NRF51_TIMER_CC(TIMER_DEADLINE), deadline);
}

static
void radio_rx_config_radio_set(
  struct ble_sniffer *radio,
  struct rx_config *config)
{
  struct buffer_s *packet = config->packet;
  struct ble_sniffer_request *rq = config->rq;

  assert(packet && rq);

  nrf_reg_set(radio->radio, NRF51_RADIO_FREQUENCY, rq->frequency - 2400);
  nrf_reg_set(radio->radio, NRF51_RADIO_DATAWHITEIV, rq->white_iv | 0x40);
  nrf_reg_set(radio->radio, NRF51_RADIO_PCNF1, 0
              | ((packet->end - packet->begin - 2)
                 << NRF51_RADIO_PCNF1_MAXLEN_OFFSET)
              | (3 << NRF51_RADIO_PCNF1_BALEN_OFFSET)
              | NRF51_RADIO_PCNF1_ENDIAN_LITTLE
              | NRF51_RADIO_PCNF1_WHITEEN_ENABLED);
}

static
void radio_rx_config_packet_set(
  struct ble_sniffer *radio,
  struct rx_config *config)
{
  struct buffer_s *packet = config->packet;
  struct ble_sniffer_request *rq = config->rq;

  assert(packet && rq);

  packet->timestamp = ble_sniffer_time_get(radio);

  nrf_reg_set(radio->radio, NRF51_RADIO_BASE0, rq->access << 8);
  nrf_reg_set(radio->radio, NRF51_RADIO_PREFIX0, rq->access >> 24);
  nrf_reg_set(radio->radio, NRF51_RADIO_CRCINIT, rq->crc_init);
  nrf_reg_set(radio->radio, NRF51_RADIO_PACKETPTR,
              (uintptr_t)packet->data + packet->begin);

  packet->data[packet->begin + 1] = 0;
}

static
void radio_request_prepare(
  struct ble_sniffer *radio,
  struct ble_sniffer_request *rq)
{
  struct buffer_s *packet;

  radio->configured.rq = rq;
  if (radio->configured.packet)
    packet = radio->configured.packet;
  else
    packet = radio->configured.packet = buffer_pool_alloc(&radio->packet_pool);

  assert(packet);

  packet->begin = 1;
  packet->end = buffer_size(packet);
  packet->data[packet->begin + 1] = 0;
}

static
void radio_configured_kick(struct ble_sniffer *radio)
{
  nrf_it_disable_mask(radio->radio, -1);

  if (!radio->configured.rq) {
    nrf_short_set(radio->radio, 0);

    nrf_task_trigger(radio->radio, NRF51_RADIO_STOP);
    nrf_task_trigger(radio->radio, NRF51_RADIO_DISABLE);
    nrf_event_clear(radio->radio, NRF51_RADIO_BCMATCH);
    nrf_event_clear(radio->radio, NRF51_RADIO_ADDRESS);
    nrf_event_clear(radio->radio, NRF51_RADIO_PAYLOAD);
    nrf_event_clear(radio->radio, NRF51_RADIO_END);

    config_shift(&radio->committed, &radio->configured);
    radio->last_frequency = 0;
    radio->last_white_iv = 0;

    if (radio->receiving) {
      buffer_refdec(radio->receiving);
      radio->receiving = NULL;
    }

    return;
  }

  uint64_t begin = ble_sniffer_time_get(radio);
  bool_t acc = 1;

  nrf_short_set(radio->radio, 0
                | (1 << NRF51_RADIO_ADDRESS_BCSTART)
                | (1 << NRF51_RADIO_ADDRESS_RSSISTART));

  uint8_t st = nrf_reg_get(radio->radio, NRF51_RADIO_STATE);

  switch (st) {
  case NRF51_RADIO_STATE_RXRU:
    do {
      st = nrf_reg_get(radio->radio, NRF51_RADIO_STATE);
    } while (st != NRF51_RADIO_STATE_RXIDLE);

  case NRF51_RADIO_STATE_RXIDLE:
    if (radio->configured.rq->frequency == radio->last_frequency
        && radio->configured.rq->white_iv == radio->last_white_iv) {
      //printk("st");
      nrf_task_trigger(radio->radio, NRF51_RADIO_START);
      acc = 0;
      break;
    }

  case NRF51_RADIO_STATE_RX:
    if (radio->committed.rq == radio->configured.rq) {
      acc = 0;
      break;
    }

    nrf_task_trigger(radio->radio, NRF51_RADIO_STOP);

  default:
    //printk("re");
    nrf_task_trigger(radio->radio, NRF51_RADIO_DISABLE);
    while (nrf_reg_get(radio->radio, NRF51_RADIO_STATE) != NRF51_RADIO_STATE_DISABLED)
      ;

  case NRF51_RADIO_STATE_DISABLED:
    //printk("ru");

    nrf_short_set(radio->radio, 0
                  | (1 << NRF51_RADIO_READY_START)
                  | (1 << NRF51_RADIO_ADDRESS_BCSTART)
                  | (1 << NRF51_RADIO_ADDRESS_RSSISTART));
    nrf_task_trigger(radio->radio, NRF51_RADIO_RXEN);
    break;
  }

  nrf_it_enable(radio->radio, NRF51_RADIO_ADDRESS);

  if (radio->receiving) {
    buffer_refdec(radio->receiving);
    radio->receiving = NULL;
  }

  //printk("\n");
  while (nrf_reg_get(radio->radio, NRF51_RADIO_STATE) != NRF51_RADIO_STATE_RX)
    ;

  nrf_event_clear(radio->radio, NRF51_RADIO_BCMATCH);
  nrf_event_clear(radio->radio, NRF51_RADIO_ADDRESS);
  nrf_event_clear(radio->radio, NRF51_RADIO_PAYLOAD);
  nrf_event_clear(radio->radio, NRF51_RADIO_END);

  config_shift(&radio->committed, &radio->configured);
  radio->last_frequency = radio->committed.rq->frequency;
  radio->last_white_iv = radio->committed.rq->white_iv;

  nrf_event_clear(radio->radio, NRF51_RADIO_BCMATCH);

  if (acc)
    lowpass_value_accumulate(&radio->radio_setup_time,
                             ble_sniffer_time_get(radio) - begin);

  radio_deadline_set(radio);
}

static
void radio_configured_pipeline(struct ble_sniffer *radio)
{
  if (!radio->configured.rq) {
    nrf_short_set(radio->radio, 0
                  | (1 << NRF51_RADIO_END_DISABLE)
                  | (1 << NRF51_RADIO_ADDRESS_BCSTART)
                  | (1 << NRF51_RADIO_ADDRESS_RSSISTART)
                  );
    config_shift(&radio->committed, &radio->configured);
    radio->last_frequency = 0;
    radio->last_white_iv = 0;
    return;
  }

  bool_t fast = radio->configured.rq->frequency == radio->last_frequency
    && radio->configured.rq->white_iv == radio->last_white_iv;

  radio_rx_config_packet_set(radio, &radio->configured);

  if (fast) {
    nrf_short_set(radio->radio, 0
                  | (1 << NRF51_RADIO_ADDRESS_BCSTART)
                  | (1 << NRF51_RADIO_ADDRESS_RSSISTART)
                  | (1 << NRF51_RADIO_END_START)
                  );
  } else {
    radio_rx_config_radio_set(radio, &radio->configured);
    nrf_short_set(radio->radio, 0
                  | (1 << NRF51_RADIO_ADDRESS_BCSTART)
                  | (1 << NRF51_RADIO_ADDRESS_RSSISTART)
                  | (1 << NRF51_RADIO_END_DISABLE)
                  | (1 << NRF51_RADIO_DISABLED_RXEN)
                  | (1 << NRF51_RADIO_READY_START)
                  );
  }

  nrf_it_enable(radio->radio, NRF51_RADIO_ADDRESS);

  config_shift(&radio->committed, &radio->configured);
  radio->last_frequency = radio->committed.rq->frequency;
  radio->last_white_iv = radio->committed.rq->white_iv;

  radio_deadline_set(radio);

  if (nrf_event_check(radio->radio, NRF51_RADIO_END)) {
    /*
      End has trigged, but we dont know whether it was before or after
      shorts were enabled.
     */

    switch (nrf_reg_get(radio->radio, NRF51_RADIO_STATE)) {
    case NRF51_RADIO_STATE_RXDISABLE:
    case NRF51_RADIO_STATE_DISABLED:
    case NRF51_RADIO_STATE_RXRU:
    case NRF51_RADIO_STATE_RX:
      // A short had some effect
      break;

    case NRF51_RADIO_STATE_RXIDLE:
      // We are stuck after END
      nrf_task_trigger(radio->radio, fast ? NRF51_RADIO_START : NRF51_RADIO_DISABLE);
      break;
    }
  }
}

static void radio_current_configure(struct ble_sniffer *radio)
{
  struct ble_sniffer_request *rq = radio->current;

  if (rq) {
    radio_request_prepare(radio, rq);
    radio_rx_config_radio_set(radio, &radio->configured);
    radio_rx_config_packet_set(radio, &radio->configured);
  } else {
    nrf_short_set(radio->radio, 0
                  | (1 << NRF51_RADIO_END_DISABLE)
                  | (1 << NRF51_RADIO_ADDRESS_BCSTART)
                  | (1 << NRF51_RADIO_ADDRESS_RSSISTART));
  }
}

static void radio_current_setup(struct ble_sniffer *radio)
{
  assert(!radio->current);

  struct ble_sniffer_request *rq
    = ble_sniffer_request_cast(dev_request_queue_head(&radio->queue));

  if (!rq)
    return;

  uint64_t now = ble_sniffer_time_get(radio);
  uint32_t radio_setup_time = lowpass_value_get(&radio->radio_setup_time);
  uint32_t timer_irq_latency = lowpass_value_get(&radio->timer_irq_latency);

  radio->end_deadline = -1;
  radio->ifs_deadline = -1;

  if (!rq->start)
    rq->start = now;

  if (rq->start > now + radio_setup_time + timer_irq_latency + 135) {
    radio->start_deadline = rq->start - radio_setup_time - 135;
    radio_deadline_set(radio);
  } else {
    struct ble_sniffer_request *next;

    radio->start_deadline = -1;
    radio->current = rq;
    radio->t_ifs = rq->t_ifs_max;

    next = ble_sniffer_request_cast(dev_request_queue_next(&radio->queue, &rq->base));

    // Preempt previous request if next has a strong anchor point
    if (next && next->start)
      radio->end_deadline = next->start - lowpass_value_get(&radio->radio_setup_time);

    if (rq->initial_timeout
        && rq->start + rq->initial_timeout < radio->end_deadline)
      radio->end_deadline = rq->start + rq->initial_timeout;
  }
}

void ble_sniffer_request(struct ble_sniffer *radio, ...)
{
    va_list ap;
    struct ble_sniffer_request *rq;
    bool_t was_empty = dev_request_queue_isempty(&radio->queue);

    va_start(ap, radio);

    while ((rq = va_arg(ap, struct ble_sniffer_request *))) {
        buffer_queue_init(&rq->queue);
        rq->packet_count = 0;

        dev_request_queue_pushback(&radio->queue, &rq->base);
    }

    va_end(ap);

    if (was_empty) {
      radio_current_setup(radio);
      radio_current_configure(radio);
      radio_configured_kick(radio);
    }
}

static void radio_rq_done(struct ble_sniffer *radio, struct ble_sniffer_request *rq, enum ble_sniffer_status status)
{
    assert(rq);

    if (radio->current == rq)
      radio->current = NULL;

    dev_request_queue_remove(&radio->queue, &rq->base);
    dev_request_queue_pushback(&radio->done_list, &rq->base);

    rq->status = status;

    while ((rq = ble_sniffer_request_cast(dev_request_queue_head(&radio->queue)))) {
      if (!rq->chain_mode || rq->chain_mode & (1 << status))
        break;

      dev_request_queue_remove(&radio->queue, &rq->base);
      dev_request_queue_pushback(&radio->done_list, &rq->base);
      status = rq->status = BLE_SNIFFER_CANCELLED;
    }
}

struct ble_sniffer *ble_sniffer_from_priv(void *priv)
{
    return (struct ble_sniffer *)priv - 1;
}

void *ble_sniffer_to_priv(struct ble_sniffer *radio)
{
    return radio + 1;
}

static void radio_setup(struct ble_sniffer *radio)
{
    nrf_reg_set(radio->radio, NRF51_RADIO_PCNF0, 0
                | (8 << NRF51_RADIO_PCNF0_LFLEN_OFFSET)
                | (1 << NRF51_RADIO_PCNF0_S0LEN_OFFSET));

    nrf_reg_set(radio->radio, NRF51_RADIO_RXADDRESSES, 1 << 0);
    nrf_reg_set(radio->radio, NRF51_RADIO_CRCPOLY, 0x00065B);
    nrf_reg_set(radio->radio, NRF51_RADIO_CRCCNF, 0
                | NRF51_RADIO_CRCCNF_SKIPADDR
                | (3 << NRF51_RADIO_CRCCNF_LEN_OFFSET));
    nrf_reg_set(radio->radio, NRF51_RADIO_TIFS, 150);

    nrf51_ppi_setup(NRF51_PPI, PPI_PACKET_TIMESTAMP,
                    radio->radio, NRF51_RADIO_ADDRESS,
                    radio->timer, NRF51_TIMER_CAPTURE(TIMER_TIMESTAMP));
    nrf51_ppi_enable(NRF51_PPI, PPI_PACKET_TIMESTAMP);

    nrf51_ppi_setup(NRF51_PPI, PPI_PACKET_ADDRESS,
                    radio->radio, NRF51_RADIO_ADDRESS,
                    radio->timer2, NRF51_TIMER_CAPTURE(TIMER_ADDRESS));
    nrf51_ppi_enable(NRF51_PPI, PPI_PACKET_ADDRESS);

    nrf51_ppi_setup(NRF51_PPI, PPI_PACKET_END,
                    radio->radio, NRF51_RADIO_END,
                    radio->timer2, NRF51_TIMER_CAPTURE(TIMER_END));
    nrf51_ppi_enable(NRF51_PPI, PPI_PACKET_END);
}

struct buffer_s *ble_sniffer_packet_alloc(struct ble_sniffer *radio)
{
  return buffer_pool_alloc(&radio->packet_pool);
}

static void radio_wait_byte(struct ble_sniffer *radio, uint16_t byte)
{
  int16_t target = nrf_reg_get(radio->timer2, NRF51_TIMER_CC(TIMER_ADDRESS))
    + byte * 8 + 2;
  volatile register int16_t diff;

  do {
    nrf_task_trigger(radio->timer2, NRF51_TIMER_CAPTURE(TIMER_TMP));
    diff = target - nrf_reg_get(radio->timer2, NRF51_TIMER_CC(TIMER_TMP));
  } while (diff > 0 && !nrf_event_check(radio->radio, NRF51_RADIO_END));
}

static void radio_rq_packet_push(struct ble_sniffer *radio,
                                 struct ble_sniffer_request *rq,
                                 struct buffer_s *packet)
{
  if ((rq->filter & BLE_SNIFFER_FILTER_RSSI) && packet->rssi > rq->rssi_min)
    goto nope;

  if ((rq->filter & BLE_SNIFFER_FILTER_DATAPDU) && packet->end - packet->begin == 2)
    goto nope;

  if (rq->filter & (BLE_SNIFFER_FILTER_ADVCONSTANT | BLE_SNIFFER_FILTER_ADVPUBLIC)) {
    uint8_t addr_msb_offset = 0;
    uint8_t addr_random_mask = 0;
    const uint8_t *data = packet->data + packet->begin;

    switch (ble_advertise_packet_type_get(packet)) {
    case BLE_ADV_NONCONN_IND:
    case BLE_SCAN_RSP:
    case BLE_ADV_IND:
    case BLE_ADV_SCAN_IND:
    case BLE_ADV_DIRECT_IND:
      addr_random_mask = 0x40;
      addr_msb_offset = 7;
      break;

    case BLE_SCAN_REQ:
    case BLE_CONNECT_REQ:
      addr_random_mask = 0x80;
      addr_msb_offset = 13;
      break;
    }

    if (rq->filter & BLE_SNIFFER_FILTER_ADVCONSTANT) {
      radio_wait_byte(radio, addr_msb_offset);
      if (data[0] & addr_random_mask
          && (data[addr_msb_offset] >> 6) != BLE_ADDR_RANDOM_STATIC)
        goto nope;
    }

    if (rq->filter & BLE_SNIFFER_FILTER_ADVPUBLIC
        && data[0] & addr_random_mask)
        goto nope;
  }

  packet->index = rq->packet_count;

  radio->ifs_deadline
    = packet->timestamp + packet->duration
    + radio->t_ifs + BLE_T_PREAMBLE;

  if (!rq->packet_count) {
    if (rq->window)
      radio->end_deadline = packet->timestamp + rq->window;
    else
      radio->end_deadline = -1;
  }

  rq->packet_count++;
  buffer_queue_pushback(&rq->queue, packet);

 nope:
  if (rq->rx_packet_max && rq->packet_count >= rq->rx_packet_max)
    radio_rq_done(radio, rq, BLE_SNIFFER_DONE);
  else if (radio->end_deadline < packet->timestamp + packet->duration + radio->t_ifs)
    radio_rq_done(radio, rq, rq->packet_count ? BLE_SNIFFER_WINDOW_TIMEOUT : BLE_SNIFFER_INITIAL_TIMEOUT);
  else if (radio->ifs_deadline < packet->timestamp + packet->duration
           + radio->t_ifs + BLE_T_PREAMBLE)
    radio_rq_done(radio, rq, BLE_SNIFFER_IFS_TIMEOUT);
}

static void radio_done_flush(struct ble_sniffer *radio)
{
  GCT_FOREACH(dev_request_queue, &radio->done_list, _rq,
    struct ble_sniffer_request *rq = ble_sniffer_request_cast(_rq);
    kroutine_exec(&rq->base.kr, 0);
    assert(rq != radio->committed.rq);
    assert(rq != radio->configured.rq);
    dev_request_queue_remove(&radio->done_list, &rq->base);
    );
}

static void radio_packet_done(struct ble_sniffer *radio);

static void radio_packet_address(struct ble_sniffer *radio)
{
    struct ble_sniffer_request *rq;
    int64_t ts;
    struct buffer_s *packet;
    uint8_t *data;
    uint64_t now, lat;

    nrf_event_clear(radio->radio, NRF51_RADIO_ADDRESS);

    nrf_short_set(radio->radio, 0
                  | (1 << NRF51_RADIO_ADDRESS_BCSTART)
                  | (1 << NRF51_RADIO_ADDRESS_RSSISTART));

    /* Get address timestamp */
    ts = nrf_reg_get(radio->timer, NRF51_TIMER_CC(TIMER_TIMESTAMP));
    if ((int32_t)ts >= 0
        && nrf_event_check(radio->timer, NRF51_TIMER_COMPARE(TIMER_OVERFLOW)))
      ts += (int64_t)1 << 32;
    ts += (int64_t)radio->timer_base << 32;

    packet = radio->committed.packet;
    rq = radio->committed.rq;

    radio->committed.packet = NULL;
    radio->committed.rq = NULL;

    if (!rq || !packet)
      goto bug;

    /* Packet timing, rssi */
    packet->timestamp = ts - 40;
    packet->rssi = nrf_reg_get(radio->radio, NRF51_RADIO_RSSISAMPLE);

    /* Wait for packet size byte */
    while (!nrf_event_check(radio->radio, NRF51_RADIO_BCMATCH))
      ;
    nrf_event_clear(radio->radio, NRF51_RADIO_BCMATCH);

    /* Packet contents */
    data = packet->data + packet->begin;
    if (packet->begin + data[1] + 2 < buffer_size(packet))
        packet->end = packet->begin + data[1] + 2;
    packet->duration = BUFFER_TIME(packet->end - packet->begin - 2);

    /* Push packet back to its request, updates request queue */
    radio_rq_packet_push(radio, rq, packet);

    /* Now we updated deadlines, requests, etc.  Current may be
       done.  */
    if (!radio->current)
      radio_current_setup(radio);
    radio_current_configure(radio);
    radio_configured_pipeline(radio);

    assert(!radio->receiving);
    radio->receiving = packet;

    /* Go back to scheduler if time permits */
    now = ble_sniffer_time_get(radio);
    lat = lowpass_value_get(&radio->timer_irq_latency);

    if (now + lat * 2 < packet->timestamp + packet->duration) {
      nrf_it_enable(radio->radio, NRF51_RADIO_PAYLOAD);
    } else {
      /* This is in case pipelining enable came too late, if it
         happens, END has triggered already */
      if (nrf_reg_get(radio->radio, NRF51_RADIO_STATE) == NRF51_RADIO_STATE_RXIDLE)
        nrf_task_trigger(radio->radio, NRF51_RADIO_START);

      while (!nrf_event_check(radio->radio, NRF51_RADIO_PAYLOAD))
        ;

      radio_packet_done(radio);
    }
    return;

 bug:
    printk("\nBug in rx: size OK without rq configured\n");
    printk("\n packet: %p\n", packet);
    printk("  start %lld, ifs %lld, end %lld\n",
           radio->start_deadline, radio->ifs_deadline, radio->end_deadline);
    printk("Queue:\n");
    GCT_FOREACH(dev_request_queue, &radio->queue, rq,
                struct ble_sniffer_request *req = ble_sniffer_request_cast(rq);

                printk(" freq %d, start %lld, initial %lld, window %lld\n",
                       req->frequency, req->start, req->initial_timeout, req->window);

                GCT_FOREACH(buffer_queue, &req->queue, packet,
                            printk("  packet rx at %lld: %d\n",
                                   packet->timestamp, packet->duration);
                            );
                );
}

static void radio_packet_done(struct ble_sniffer *radio)
{
  struct buffer_s *packet = radio->receiving;
  radio->receiving = NULL;

  nrf_event_clear(radio->radio, NRF51_RADIO_PAYLOAD);
  nrf_it_disable(radio->radio, NRF51_RADIO_PAYLOAD);

  if (!packet)
    return;

  while (!nrf_event_check(radio->radio, NRF51_RADIO_END))
    ;

  /* Now, we have actual packet data and flags */
  packet->crc = nrf_reg_get(radio->radio, NRF51_RADIO_RXCRC);
  packet->crc_valid = !!nrf_reg_get(radio->radio, NRF51_RADIO_CRCSTATUS);
  uint32_t delta
    = (uint16_t)nrf_reg_get(radio->timer2, NRF51_TIMER_CC(TIMER_END))
    - (uint16_t)nrf_reg_get(radio->timer2, NRF51_TIMER_CC(TIMER_ADDRESS));
  delta &= 0xffff;
  packet->duration = delta + 40;

  buffer_refdec(packet);

  radio_done_flush(radio);
}

static DEV_IRQ_EP_PROCESS(radio_irq)
{
  struct ble_sniffer *radio = KROUTINE_CONTAINER(ep, *radio, radio_source);

  if (nrf_event_check(radio->radio, NRF51_RADIO_PAYLOAD))
    radio_packet_done(radio);

  if (nrf_event_check(radio->radio, NRF51_RADIO_ADDRESS))
    radio_packet_address(radio);
}

static DEV_IRQ_EP_PROCESS(timer_irq)
{
  struct ble_sniffer *radio = KROUTINE_CONTAINER(ep, *radio, timer_source);

  for (;;) {
    if (nrf_event_check(radio->timer, NRF51_TIMER_COMPARE(TIMER_OVERFLOW))) {
      nrf_event_clear(radio->timer, NRF51_TIMER_COMPARE(TIMER_OVERFLOW));
      radio->timer_base += 1;
    }

    if (nrf_event_check(radio->timer, NRF51_TIMER_COMPARE(TIMER_DEADLINE))) {
      nrf_event_clear(radio->timer, NRF51_TIMER_COMPARE(TIMER_DEADLINE));
      nrf_task_trigger(radio->timer, NRF51_TIMER_CAPTURE(TIMER_VALUE));
      int32_t value = nrf_reg_get(radio->timer, NRF51_TIMER_CC(TIMER_VALUE));
      int32_t dead = nrf_reg_get(radio->timer, NRF51_TIMER_CC(TIMER_DEADLINE));
      int32_t lat = value - dead;
      struct ble_sniffer_request *rq
        = ble_sniffer_request_cast(dev_request_queue_head(&radio->queue));

      if (lat > 0)
        lowpass_value_accumulate(&radio->timer_irq_latency, lat);

      if (!rq)
        continue;

      uint64_t now = ble_sniffer_time_get(radio);

      if (!radio->current && radio->start_deadline + 5 < now) {
        radio_current_setup(radio);
        radio_current_configure(radio);
        radio_configured_kick(radio);
        continue;
      }

      if (!nrf_event_check(radio->radio, NRF51_RADIO_ADDRESS)) {
        if (radio->end_deadline < now)
          radio_rq_done(radio, rq, rq->packet_count ? BLE_SNIFFER_WINDOW_TIMEOUT : BLE_SNIFFER_INITIAL_TIMEOUT);
        else if (radio->ifs_deadline < now)
          radio_rq_done(radio, rq, BLE_SNIFFER_IFS_TIMEOUT);

        if (!radio->current)
          radio_current_setup(radio);
        radio_current_configure(radio);
        radio_configured_kick(radio);

        radio_done_flush(radio);
      }

      continue;
    }

    break;
  }
}

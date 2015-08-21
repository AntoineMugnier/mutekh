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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2015
*/

#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>

#include <hexo/types.h>
#include <hexo/endian.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/clock.h>
#include <device/class/rfpacket.h>
#include <device/class/timer.h>
#include <device/request.h>

#include <arch/nrf5x/peripheral.h>
#include <arch/nrf5x/radio.h>
#include <arch/nrf5x/power.h>
#include <arch/nrf5x/ids.h>
#include <arch/nrf5x/rtc.h>
#include <arch/nrf5x/ficr.h>
#include <arch/ppi.h>
#include <arch/nrf5x/ppi.h>

#define RADIO_ADDR NRF_PERIPHERAL_ADDR(NRF5X_RADIO)
#define RTC_ADDR NRF_PERIPHERAL_ADDR(NRF5X_RTC0)

enum ppi_id_e
{
  PPI_RTC_TIMEOUT,
  PPI_RTC_MATCH_START,
  PPI_COUNT,
};

#define RTC_ENABLE             0
#define PPI_RTC_ENABLE_TXEN    NRF_PPI_RTC0_COMPARE_0_RADIO_TXEN
#define PPI_RTC_ENABLE_RXEN    NRF_PPI_RTC0_COMPARE_0_RADIO_RXEN
#define RTC_TIMEOUT            1
#define RTC_START              2

struct nrf5x_radio_private_s
{
  struct dev_irq_src_s irq_source[NRF5X_RADIO_IRQ_COUNT];

#if defined(CONFIG_DEVICE_CLOCK)
  struct dev_clock_sink_ep_s clock_sink[NRF5X_RADIO_CLK_COUNT];

  bool_t accurate_clock_requested;
  bool_t accurate_clock_running;
#endif

  struct dev_freq_accuracy_s sleep_acc;

  uint8_t ppi[PPI_COUNT];
  dev_request_queue_root_t queue;
  dev_request_queue_root_t rx_queue;

  dev_timer_value_t base;

  bool_t callbacking;

  uint32_t use_count;
};

static void radio_clock_release(struct nrf5x_radio_private_s *pv);
static bool_t radio_clock_request(struct nrf5x_radio_private_s *pv);
static void radio_next_action(struct nrf5x_radio_private_s *pv);

static void radio_init(void)
{
  nrf_reg_set(RADIO_ADDR, NRF_RADIO_POWER, 0);

  for (uint16_t i = 0; i < 32; ++i)
    asm volatile("");

  nrf_reg_set(RADIO_ADDR, NRF_RADIO_POWER, 1);

  for (uint16_t i = 0; i < 32; ++i)
    asm volatile("");

  nrf_reg_set(NRF_PERIPHERAL_ADDR(NRF5X_POWER), NRF_POWER_DCDCEN,
              _CONFIG_NRF5X_RADIO_DCDC ? NRF_POWER_DCDCEN_ENABLE : 0);

  nrf_it_disable_mask(RADIO_ADDR, -1);

  for (uint8_t i = 0; i < 5; ++i)
    nrf_reg_set(RADIO_ADDR, NRF_RADIO_OVERRIDE(i),
                cpu_mem_read_32(NRF_FICR_BLE_1MBIT(i)));

  nrf_task_trigger(RADIO_ADDR, NRF_RADIO_STOP);
  nrf_task_trigger(RADIO_ADDR, NRF_RADIO_DISABLE);
}

static void rtc_init(void)
{
    nrf_task_trigger(RTC_ADDR, NRF_RTC_STOP);
    nrf_reg_set(RTC_ADDR, NRF_RTC_PRESCALER, 0);
    nrf_task_trigger(RTC_ADDR, NRF_RTC_CLEAR);
}

static void ppi_init(struct nrf5x_radio_private_s *pv)
{
  nrf_ppi_setup(NRF_PPI_ADDR, pv->ppi[PPI_RTC_TIMEOUT],
                  RTC_ADDR, NRF_RTC_COMPARE(RTC_TIMEOUT),
                  RADIO_ADDR, NRF_RADIO_DISABLE);

  nrf_ppi_setup(NRF_PPI_ADDR, pv->ppi[PPI_RTC_MATCH_START],
                  RTC_ADDR, NRF_RTC_COMPARE(RTC_START),
                  RADIO_ADDR, NRF_RADIO_START);
}

static void rtc_start(struct nrf5x_radio_private_s *pv)
{
#if defined(CONFIG_DEVICE_CLOCK)
  dev_clock_sink_hold(&pv->clock_sink[NRF5X_RADIO_CLK_SLEEP], 1);
#endif

  nrf_it_disable_mask(RTC_ADDR, -1);
  nrf_it_enable(RTC_ADDR, NRF_RTC_OVERFLW);
  nrf_evt_disable_mask(RTC_ADDR, -1);
  nrf_task_trigger(RTC_ADDR, NRF_RTC_START);
}

static void rtc_stop(struct nrf5x_radio_private_s *pv)
{
#if defined(CONFIG_DEVICE_CLOCK)
  dev_clock_sink_release(&pv->clock_sink[NRF5X_RADIO_CLK_SLEEP]);
#endif

  nrf_task_trigger(RTC_ADDR, NRF_RTC_STOP);
  nrf_it_disable_mask(RTC_ADDR, -1);
  nrf_evt_disable_mask(RTC_ADDR, -1);
}

static dev_timer_value_t rtc_value_get(struct nrf5x_radio_private_s *pv)
{
  uint32_t counter;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  counter = nrf_reg_get(RTC_ADDR, NRF_RTC_COUNTER);
  if (!(counter & 0x800000)
      && nrf_event_check(RTC_ADDR, NRF_RTC_OVERFLW))
    counter += 0x1000000;
  CPU_INTERRUPT_RESTORESTATE;

  return pv->base + counter;
}

#if defined(CONFIG_DEVICE_CLOCK)

static bool_t radio_clock_request(struct nrf5x_radio_private_s *pv)
{
  if (!pv->accurate_clock_requested) {
    dev_clock_sink_hold(&pv->clock_sink[NRF5X_RADIO_CLK_RADIO], 0);
    pv->accurate_clock_requested = 1;
  }

  return pv->accurate_clock_running;
}

/* TODO: Do this later on */
static void radio_clock_release(struct nrf5x_radio_private_s *pv)
{
  if (pv->accurate_clock_requested) {
    dev_clock_sink_release(&pv->clock_sink[NRF5X_RADIO_CLK_RADIO]);

    pv->accurate_clock_requested = 0;
  }
}

static DEV_CLOCK_SINK_CHANGED(nrf5x_radio_clock_changed)
{
  struct nrf5x_radio_private_s *pv = ep->dev->drv_pv;
  uint8_t clk = ep - pv->clock_sink;

  if (clk == NRF5X_RADIO_CLK_SLEEP
      && ep->src->flags & DEV_CLOCK_SRC_EP_RUNNING)
    pv->sleep_acc = *acc;

  if (clk == NRF5X_RADIO_CLK_RADIO) {
    bool_t was_running = pv->accurate_clock_running;

    pv->accurate_clock_running = !!(ep->src->flags & DEV_CLOCK_SRC_EP_RUNNING);

    if (!was_running && pv->accurate_clock_running)
      radio_next_action(pv);
  }
}

#else

static bool_t radio_clock_request(struct nrf5x_radio_private_s *pv)
{
  return 1;
}

static void radio_clock_release(struct nrf5x_radio_private_s *pv)
{
}

#endif

static
void radio_disable(struct nrf5x_radio_private_s *pv)
{
  nrf_it_disable_mask(RADIO_ADDR, -1);
  nrf_short_set(RADIO_ADDR, 0);

  nrf_ppi_disable_mask(NRF_PPI_ADDR, 0
                         | (1 << pv->ppi[PPI_RTC_MATCH_START])
                         | (1 << PPI_RTC_ENABLE_RXEN)
                         | (1 << PPI_RTC_ENABLE_TXEN)
                         | (1 << pv->ppi[PPI_RTC_TIMEOUT])
                         );

  nrf_evt_disable_mask(RTC_ADDR, 0
                       | (1 << NRF_RTC_COMPARE(RTC_ENABLE))
                       | (1 << NRF_RTC_COMPARE(RTC_START))
                       );

  nrf_task_trigger(RADIO_ADDR, NRF_RADIO_STOP);
  nrf_task_trigger(RADIO_ADDR, NRF_RADIO_DISABLE);
}

static DEV_TIMER_CONFIG(nrf5x_radio_rtc_config)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_radio_private_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (cfg) {
    cfg->freq.num = 32768;
    cfg->freq.denom = 1;
    cfg->acc = pv->sleep_acc;
    cfg->rev = 0;
    cfg->res = 1;
    cfg->cap = DEV_TIMER_CAP_STOPPABLE | DEV_TIMER_CAP_HIGHRES;
    cfg->max = (dev_timer_value_t)-1;
  }

  if (res > 1)
    err = -ERANGE;

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_TIMER_GET_VALUE(nrf5x_radio_rtc_get_value)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_radio_private_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (value)
    *value = rtc_value_get(pv);

  if (rev)
    err = -EAGAIN;

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}
#define nrf5x_radio_rtc_request (dev_timer_request_t*)dev_driver_notsup_fcn
#define nrf5x_radio_rtc_cancel (dev_timer_request_t*)dev_driver_notsup_fcn

static DEV_IRQ_SRC_PROCESS(rtc_irq)
{

}

static DEV_IRQ_SRC_PROCESS(radio_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_radio_private_s *pv = dev->drv_pv;
  struct dev_rfpacket_rq_s *rq;

  lock_spin(&dev->lock);

  rq = dev_rfpacket_rq_s_cast(dev_request_queue_head(&pv->queue));
  if (!rq)
    goto stop;

  /* Do something */

  goto out;

 callback:
  dev_request_queue_pop(&pv->queue);

  rq->base.drvdata = 0;

  pv->callbacking = 1;
  lock_release(&dev->lock);
  kroutine_exec(&rq->base.kr);
  lock_spin(&dev->lock);
  pv->callbacking = 0;

  rq = dev_rfpacket_rq_s_cast(dev_request_queue_head(&pv->queue));
  if (!rq)
    goto stop;

 stop:
  nrf_task_trigger(RADIO_ADDR, NRF_RADIO_STOP);

 out:
  lock_release(&dev->lock);
}

static void nrf5x_radio_rq_next(struct device_s *dev)
{
  struct nrf5x_radio_private_s *pv = dev->drv_pv;

  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_s_cast(dev_request_queue_head(&pv->queue));

  switch (rq->type) {
  case DEV_RFPACKET_RQ_CONFIG:
  case DEV_RFPACKET_RQ_TX:
  case DEV_RFPACKET_RQ_RX:
  case DEV_RFPACKET_RQ_IDLE:
    break;
  }
}

static void radio_next_action(struct nrf5x_radio_private_s *pv)
{
  struct dev_rfpacket_rq_s *rq;

  rq = dev_rfpacket_rq_s_cast(dev_request_queue_head(&pv->queue));

  if (!rq) {
    radio_disable();
    return;
  }

  switch (rq->type) {
  case DEV_RFPACKET_RQ_CONFIG:
    // Should not happen
    return;

  case DEV_RFPACKET_RQ_TX:
    
  case DEV_RFPACKET_RQ_RX:
  case DEV_RFPACKET_RQ_IDLE:
    break;
  }
}

static DEV_RFPACKET_REQUEST(nrf5x_radio_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_radio_private_s *pv = dev->drv_pv;
  bool_t start;
  struct dev_rfpacket_rq_s *rq;
  va_list arg;

  LOCK_SPIN_IRQ(&dev->lock);

  start = dev_request_queue_isempty(&pv->queue);

  va_start(arg, accessor);
  for (;;) {
    rq = va_arg(arg, struct dev_rfpacket_rq_s *);

    if (!rq)
      break;

    dev_request_queue_pushback(&pv->queue, &rq->base);
    rq->base.drvdata = dev;
  }

  va_end(arg);

  if (start)
    nrf5x_radio_rq_next(dev);

  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_INIT(nrf5x_radio_init);
static DEV_CLEANUP(nrf5x_radio_cleanup);
static DEV_USE(nrf5x_radio_use);

DRIVER_DECLARE(nrf5x_radio_drv, 0, "nRF5x RADIO", nrf5x_radio,
               DRIVER_RFPACKET_METHODS(nrf5x_radio),
               DRIVER_TIMER_METHODS(nrf5x_radio_rtc));

DRIVER_REGISTER(nrf5x_radio_drv);

static DEV_USE(nrf5x_radio_use)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_radio_private_s *pv = dev->drv_pv;
  uint32_t old;

  switch (op) {
  default:
    return 0;

  case DEV_USE_START:
    old = pv->use_count;

    pv->use_count++;

    if (!old)
      rtc_start(pv);
    break;

  case DEV_USE_STOP:
    pv->use_count--;

    if (!pv->use_count)
      rtc_stop(pv);
    break;
  }

  return 0;
}

static DEV_INIT(nrf5x_radio_init)
{
  struct nrf5x_radio_private_s *pv;
  error_t err;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = malloc(sizeof(*pv));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  __unused__ uintptr_t addr = 0;
  assert(device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL) == 0 &&
         NRF_PERIPHERAL_ADDR(NRF5X_RADIO) == addr);

  assert(device_res_get_uint(dev, DEV_RES_MEM, 1, &addr, NULL) == 0 &&
         NRF_PERIPHERAL_ADDR(NRF5X_RTC0) == addr);

  device_irq_source_init(dev, &pv->irq_source[NRF5X_RADIO_IRQ_RADIO], 1, &radio_irq);
  device_irq_source_init(dev, &pv->irq_source[NRF5X_RADIO_IRQ_RTC], 1, &rtc_irq);

  err = device_irq_source_link(dev, pv->irq_source, NRF5X_RADIO_IRQ_COUNT, -1);
  if (err)
    goto free_pv;

#if defined(CONFIG_DEVICE_CLOCK)
  pv->accurate_clock_requested = 0;
  pv->accurate_clock_running = 0;

  dev_clock_sink_init(dev, &pv->clock_sink[NRF5X_RADIO_CLK_RADIO], &nrf5x_radio_clock_changed);
  dev_clock_sink_init(dev, &pv->clock_sink[NRF5X_RADIO_CLK_SLEEP], &nrf5x_radio_clock_changed);

  struct dev_clock_link_info_s ckinfo[2];
  if (dev_clock_sink_link(dev, pv->clock_sink, ckinfo, 0, 1))
    goto free_pv;
#endif

  dev_request_queue_init(&pv->queue);
  dev_request_queue_init(&pv->rx_queue);

  radio_init();
  rtc_init();
  ppi_init(pv);

  dev->drv = &nrf5x_radio_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;
  dev->drv_pv = pv;

  return 0;

 free_pv:
  free(pv);

  return err;
}

static DEV_CLEANUP(nrf5x_radio_cleanup)
{
  struct nrf5x_radio_private_s *pv = dev->drv_pv;

  nrf_it_disable_mask(RADIO_ADDR, -1);
  nrf_it_disable_mask(RTC_ADDR, -1);

  radio_disable(pv);

  nrf_task_trigger(RTC_ADDR, NRF_RTC_STOP);

  dev_request_queue_destroy(&pv->queue);
  dev_request_queue_destroy(&pv->rx_queue);

  device_irq_source_unlink(dev, pv->irq_source, NRF5X_RADIO_IRQ_COUNT);

  mem_free(pv);
}

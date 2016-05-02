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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2014
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <string.h>

#include <mutek/mem_alloc.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/char.h>
#include <device/class/timer.h>

#include <mutek/printk.h>

#include <drivers/rtt/rtt.h>

struct rtt_private_s {
  uint8_t tx_buffer[CONFIG_DRIVER_CHAR_RTT_TX_BUFFER_SIZE];
  uint8_t rx_buffer[CONFIG_DRIVER_CHAR_RTT_RX_BUFFER_SIZE];
  struct device_timer_s timer;
  struct dev_timer_rq_s timer_rq;
  dev_request_queue_root_t tx_queue, rx_queue;
  struct rtt_channel_s *tx, *rx;
  bool_t callbacking;
};

/* static void rtt_poll_disable(struct device_s *dev) */
/* { */
/*   struct rtt_private_s *pv = dev->drv_pv; */

/*   if (pv->timer_rq.rq.pvdata */
/*       && DEVICE_OP(&pv->timer, cancel, &pv->timer_rq) == 0) */
/*     pv->timer_rq.rq.pvdata = NULL; */
/* } */

static void rtt_poll_enable(struct device_s *dev)
{
  struct rtt_private_s *pv = dev->drv_pv;

  if (!pv->timer_rq.rq.pvdata) {
    pv->timer_rq.rq.pvdata = dev;
    pv->timer_rq.deadline = 0;
    ensure(DEVICE_OP(&pv->timer, request, &pv->timer_rq) == 0);
  }
}

static void rtt_request_finish(
    struct device_s *dev,
    struct dev_char_rq_s *rq)
{
  struct rtt_private_s *pv = dev->drv_pv;

  pv->callbacking = 1;
  lock_release(&dev->lock);

  rq->error = 0;
  kroutine_exec(&rq->base.kr);

  lock_spin(&dev->lock);
  pv->callbacking = 0;
}

static void rtt_try_io(struct device_s *dev)
{
  struct rtt_private_s *pv = dev->drv_pv;
  struct dev_char_rq_s *rq;

  while ((rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->tx_queue)))) {
    uint32_t done = rtt_channel_write(pv->tx, rq->data, rq->size);

    if (!done)
      break;

    rq->data += done;
    rq->size -= done;

    if (rq->size == 0 || (rq->type & _DEV_CHAR_PARTIAL)) {
      dev_request_queue_remove(&pv->tx_queue, &rq->base);
      rtt_request_finish(dev, rq);
    }
  }

  while ((rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->rx_queue)))) {
    uint32_t done = rtt_channel_read(pv->rx, rq->data, rq->size);

    if (!done)
      break;

    rq->data += done;
    rq->size -= done;

    if (rq->size == 0 || (rq->type & _DEV_CHAR_PARTIAL)) {
      dev_request_queue_remove(&pv->rx_queue, &rq->base);
      rtt_request_finish(dev, rq);
    }
  }

  if (!dev_request_queue_isempty(&pv->rx_queue)
      || !dev_request_queue_isempty(&pv->tx_queue))
    rtt_poll_enable(dev);
}

static KROUTINE_EXEC(rtt_tick)
{
  struct rtt_private_s *pv = KROUTINE_CONTAINER(kr, *pv, timer_rq.rq.kr);
  struct device_s *dev = pv->timer_rq.rq.pvdata;

  if (!dev)
    return;

  LOCK_SPIN_IRQ(&dev->lock);
  pv->timer_rq.rq.pvdata = NULL;
  rtt_try_io(dev);
  LOCK_RELEASE_IRQ(&dev->lock);
}

#define char_rtt_cancel (dev_char_cancel_t*)&dev_driver_notsup_fcn

DEV_CHAR_REQUEST(char_rtt_request)
{
  struct device_s *dev = accessor->dev;
  struct rtt_private_s *pv = dev->drv_pv;
  dev_request_queue_root_t *q = NULL;
  bool_t empty;

  assert(rq->size);

  switch (rq->type) {
  case DEV_CHAR_READ_PARTIAL:
  case DEV_CHAR_READ:
    q = &pv->rx_queue;
    break;

  case DEV_CHAR_WRITE_PARTIAL_FLUSH:
  case DEV_CHAR_WRITE_FLUSH:
  case DEV_CHAR_WRITE_PARTIAL:
  case DEV_CHAR_WRITE:
    q = &pv->tx_queue;
    break;
  default:
    rq->error = -ENOTSUP;
    kroutine_exec(&rq->base.kr);
    return;
  }

  LOCK_SPIN_IRQ(&dev->lock);

  empty = dev_request_queue_isempty(q);
  dev_request_queue_pushback(q, &rq->base);

  if (empty && !pv->callbacking)
    rtt_try_io(dev);

  LOCK_RELEASE_IRQ(&dev->lock);
}

#define char_rtt_use dev_use_generic

static DEV_CLEANUP(char_rtt_cleanup)
{
  struct rtt_private_s *pv = dev->drv_pv;

  if (!dev_request_queue_isempty(&pv->rx_queue) ||
      !dev_request_queue_isempty(&pv->tx_queue) ||
      pv->callbacking)
    return -EBUSY;

  rtt_channel_cleanup(pv->tx);
  rtt_channel_cleanup(pv->rx);

  device_put_accessor(&pv->timer.base);

  mem_free(pv);

  return 0;
}

static DEV_INIT(char_rtt_init)
{
  struct rtt_private_s *pv;


  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  if (device_get_param_dev_accessor(dev, "timer", &pv->timer, DRIVER_CLASS_TIMER)) {
    goto err_pv;
  }

  dev_request_queue_init(&pv->tx_queue);
  dev_request_queue_init(&pv->rx_queue);

  pv->tx = rtt_channel_init(RTT_CHANNEL_TX_ID(CONFIG_DRIVER_CHAR_RTT_CHANNEL_FIRST),
                            "console_tx", pv->tx_buffer, sizeof(pv->tx_buffer),
                            RTT_CHANNEL_MODE_BLOCKING);

  pv->rx = rtt_channel_init(RTT_CHANNEL_RX_ID(CONFIG_DRIVER_CHAR_RTT_CHANNEL_FIRST),
                            "console_rx", pv->rx_buffer, sizeof(pv->rx_buffer),
                            RTT_CHANNEL_MODE_BLOCKING);

  dev->drv_pv = pv;

  kroutine_init_sched_switch(&pv->timer_rq.rq.kr, rtt_tick);
  dev_timer_init_sec(&pv->timer, &pv->timer_rq.delay, 0, 1, 20);

  return 0;

 err_pv:
  mem_free(pv);
  return 1;
}

DRIVER_DECLARE(char_rtt_drv, 0, "RTT Char", char_rtt,
               DRIVER_CHAR_METHODS(char_rtt));

DRIVER_REGISTER(char_rtt_drv);

DEV_DECLARE_STATIC(rtt_console_dev, "console1", 0, char_rtt_drv,
                   DEV_STATIC_RES_DEV_TIMER("rtc* timer*"),
                   );


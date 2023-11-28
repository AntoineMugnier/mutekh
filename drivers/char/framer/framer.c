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

  Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2023
*/

#define LOGK_MODULE_ID "frmr"

#include <hexo/types.h>
#include <hexo/endian.h>
#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/char.h>

/*
  This driver provides one char device that accepts read/write
  requests for streaming data (non-bounded), and transforms them as
  char frames in the underlying device. This allows to use HDLC or Mux
  transport for streaming data.

  MTU for frames, frame header size and header to insert on incoming
  frames are device parameters.

  Frame MTU includes header to insert/remove. This is the size of the
  RX/TX request sent to underlying device.

  Here is a sample static char_framer device declaration:

  static const uint8_t framer0_tx_header[] = { 0x00, 0x03 };

  DEV_DECLARE_STATIC(framer0, "framer0", 0, char_framer_drv,
                     DEV_STATIC_RES_DEV_PARAM("io", "/hdlc0[0]"),
                     DEV_STATIC_RES_UINT_PARAM("header_size", sizeof(framer0_tx_header)),
                     DEV_STATIC_RES_BLOB_PARAM("tx_header", framer0_tx_header),
                     DEV_STATIC_RES_UINT_PARAM("mtu", 64),
                    );
*/

struct framer_rx_s
{
  // pvdata points to pv
  struct dev_char_rq_s rx_rq;
  uint8_t *rx_frame, *rx_data;
  size_t rx_size;
  bool_t BITFIELD(rx_busy, 1);
};

struct framer_context_s
{
  struct device_char_s io;

  // pvdata points to dev
  struct dev_char_rq_s tx_rq;

  dev_request_queue_root_t rx_q;
  dev_request_queue_root_t tx_q;

  struct kroutine_s rx_service;
  
  size_t header_size;
  size_t mtu;
  uint8_t *tx_frame;
  const uint8_t *tx_header;

  bool_t BITFIELD(tx_busy, 1);
  uint8_t BITFIELD(rx_head, 1);

  struct framer_rx_s rx[2];
};
  

STRUCT_COMPOSE(framer_context_s, tx_rq);
STRUCT_COMPOSE(framer_rx_s, rx_rq);
STRUCT_COMPOSE(framer_context_s, rx_service);

DRIVER_PV(struct framer_context_s);

static
DEV_CHAR_CANCEL(char_framer_cancel)
{
  struct device_s *dev = accessor->dev;
  struct framer_context_s *pv = dev->drv_pv;

  logk_debug("cancel %p", rq);

  if (rq->base.drvdata != pv)
    return -ENOENT;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  switch (rq->type)
    {
    case DEV_CHAR_READ:
    case DEV_CHAR_READ_PARTIAL:
    case DEV_CHAR_READ_NONBLOCK:
      rq->base.drvdata = NULL;
      dev_char_rq_remove(&pv->rx_q, rq);
      return 0;

    case DEV_CHAR_WRITE:
    case DEV_CHAR_WRITE_FLUSH:
    case DEV_CHAR_WRITE_PARTIAL:
    case DEV_CHAR_WRITE_NONBLOCK:
      rq->base.drvdata = NULL;
      dev_char_rq_remove(&pv->tx_q, rq);
      return 0;
      
    default:
      return -ENOTSUP;
    }
}

static
void framer_tx_prepare(struct framer_context_s *pv)
{
  assert(!pv->tx_busy);

  pv->tx_rq.size = pv->header_size;
  pv->tx_rq.data = pv->tx_frame;
  memcpy(pv->tx_frame, pv->tx_header, pv->header_size);

  GCT_FOREACH(dev_request_queue, &pv->tx_q, item, {
      struct dev_char_rq_s *rq = dev_char_rq_s_cast(item);
      size_t to_copy = pv->mtu - pv->tx_rq.size;

      if (to_copy > rq->size)
        to_copy = rq->size;

      if (to_copy) {
        memcpy(pv->tx_rq.data + pv->tx_rq.size, rq->data, to_copy);
        pv->tx_rq.size += to_copy;
        rq->size -= to_copy;
        rq->data += to_copy;
      }

      if (rq->size == 0) {
        rq->error = 0;
        rq->base.drvdata = NULL;
        dev_char_rq_remove(&pv->tx_q, rq);
        dev_char_rq_done(rq);
        GCT_FOREACH_CONTINUE;
      }

      if (rq->type & _DEV_CHAR_NONBLOCK) {
        rq->error = 0;
        rq->base.drvdata = NULL;
        dev_char_rq_remove(&pv->tx_q, rq);
        dev_char_rq_done(rq);
        GCT_FOREACH_CONTINUE;
      }
    });

  if (pv->tx_rq.size == pv->header_size)
    return;

  pv->tx_busy = 1;
  logk_trace("< %P", pv->tx_rq.data, pv->tx_rq.size);
  DEVICE_OP(&pv->io, request, &pv->tx_rq);
}

static
struct framer_context_s *framer_context_s_from_rx(struct framer_rx_s *rx)
{
  return rx->rx_rq.pvdata;
}

static
void framer_rx_schedule(struct framer_rx_s *rx)
{
  struct framer_context_s *pv = framer_context_s_from_rx(rx);
  assert(!rx->rx_busy);

  rx->rx_rq.data = rx->rx_frame;
  rx->rx_rq.size = pv->mtu;
  rx->rx_data = NULL;
  rx->rx_size = 0;
  rx->rx_busy = 1;
  DEVICE_OP(&pv->io, request, &rx->rx_rq);
}

static
KROUTINE_EXEC(framer_rx_serve)
{
  struct framer_context_s *pv = framer_context_s_from_rx_service(kr);
  uint8_t to_serve = pv->rx_head;

  for (uint8_t off = 0; off < 2; ++off, to_serve = !to_serve) {
    struct framer_rx_s *rx = pv->rx + to_serve;
    
    // Do this sub-optimal loop to liberate NONBLOCK requests if
    // needed.
    GCT_FOREACH(dev_request_queue, &pv->rx_q, item, {
        struct dev_char_rq_s *rq = dev_char_rq_s_cast(item);
        size_t to_copy = rx->rx_size;

        if (rx->rx_busy)
          to_copy = 0;
        
        if (to_copy > rq->size)
          to_copy = rq->size;

        if (to_copy) {
          memcpy(rq->data, rx->rx_data, to_copy);
          rx->rx_size -= to_copy;
          rx->rx_data += to_copy;
          rq->size -= to_copy;
          rq->data += to_copy;
        }

        if (rq->size == 0) {
          rq->error = 0;
          rq->base.drvdata = NULL;
          dev_char_rq_remove(&pv->rx_q, rq);
          dev_char_rq_done(rq);
          GCT_FOREACH_CONTINUE;
        }

        if (rq->type & _DEV_CHAR_NONBLOCK) {
          rq->error = -EAGAIN;
          rq->base.drvdata = NULL;
          dev_char_rq_remove(&pv->rx_q, rq);
          dev_char_rq_done(rq);
          GCT_FOREACH_CONTINUE;
        }
      });

    if (!rx->rx_busy && rx->rx_size == 0) {
      framer_rx_schedule(rx);
      pv->rx_head = !to_serve;
    }
  }
}

static
DEV_CHAR_REQUEST(char_framer_request)
{
  struct device_s *dev = accessor->dev;
  struct framer_context_s *pv = dev->drv_pv;

  logk_debug("rq %p", rq);
  
  switch (rq->type & ~(_DEV_CHAR_FLUSH | _DEV_CHAR_NONBLOCK)) {
  case DEV_CHAR_READ_PARTIAL:
  case DEV_CHAR_READ: {
    LOCK_SPIN_IRQ_SCOPED(&dev->lock);

    logk_debug("read %p", rq);

    dev_char_rq_pushback(&pv->rx_q, rq);
    rq->base.drvdata = pv;
    rq->base.error = 0;
    kroutine_exec(&pv->rx_service);
    return;
  }

  case DEV_CHAR_WRITE_PARTIAL:
  case DEV_CHAR_WRITE: {
    LOCK_SPIN_IRQ_SCOPED(&dev->lock);

    logk_debug("write %p", rq);

    dev_char_rq_pushback(&pv->tx_q, rq);
    rq->base.drvdata = pv;
    rq->base.error = 0;
    if (!pv->tx_busy)
      framer_tx_prepare(pv);
    return;
  }

  default:
    logk_debug("unsup %p %02x", rq, rq->type);
    rq->error = -ENOTSUP;
    dev_char_rq_done(rq);
  }
}

static
KROUTINE_EXEC(framer_io_rx_done)
{
  struct dev_char_rq_s *rq = dev_char_rq_from_kr(kr);
  struct framer_rx_s *rx = framer_rx_s_from_rx_rq(rq);
  struct framer_context_s *pv = rq->pvdata;
  struct device_s *dev = pv->tx_rq.pvdata;

  logk_debug("rx io err %d", rq->error);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  rx->rx_busy = 0;
  rx->rx_size = rq->data - rx->rx_frame;

  if (rq->error == 0 && rx->rx_size > pv->header_size) {
    rx->rx_size -= pv->header_size;
    rx->rx_data = rx->rx_frame + pv->header_size;
  } else {
    rx->rx_size = 0;
    rx->rx_data = NULL;
  }

  kroutine_exec(&pv->rx_service);
}

static
KROUTINE_EXEC(framer_io_tx_done)
{
  struct dev_char_rq_s *rq = dev_char_rq_from_kr(kr);
  struct device_s *dev = rq->pvdata;
  struct framer_context_s *pv = dev->drv_pv;

  logk_debug("tx io err %d", rq->error);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  pv->tx_busy = 0;
  framer_tx_prepare(pv);
}


static
DEV_USE(char_framer_use)
{
  struct device_accessor_s *accessor = param;

  switch (op)
    {
    case DEV_USE_START: {
      struct device_s *dev = accessor->dev;
      struct framer_context_s *pv = dev->drv_pv;
      if (dev->start_count == 0) {
        for (uint8_t off = 0; off < 2; ++off) {
          struct framer_rx_s *rx = pv->rx + off;
          framer_rx_schedule(rx);
        }

        return device_start(&pv->io.base);
      }
    }

    case DEV_USE_STOP: {
      struct device_s *dev = accessor->dev;
      struct framer_context_s *pv = dev->drv_pv;
      if (dev->start_count == 0)
        device_stop(&pv->io.base);
      return 0;
    }

    default:
      return dev_use_generic(param, op);
    }
}

static
DEV_INIT(char_framer_init)
{
  struct framer_context_s	*pv;
  size_t pv_size;
  error_t err;
  const uint8_t *tx_header;
  uintptr_t header_size, mtu;

  err = device_get_param_blob(dev, "tx_header", 0, (const void **)&tx_header);
  if (err)
    return err;

  err = device_get_param_uint(dev, "mtu", &mtu);
  if (err)
    return err;

  err = device_get_param_uint(dev, "header_size", &header_size);
  if (err)
    return err;

  if (header_size >= mtu)
    return -EINVAL;

  pv_size = sizeof(*pv) + 3 * mtu;
  pv = mem_alloc(pv_size, mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, pv_size);
  dev->drv_pv = pv;
  pv->mtu = mtu;
  pv->header_size = header_size;
  pv->tx_header = tx_header;
  pv->tx_frame = (uint8_t *)(pv + 1);

  err = device_get_param_dev_accessor(dev, "io", &pv->io.base, DRIVER_CLASS_CHAR);
  if (err)
    goto err_mem;

  dev_rq_queue_init(&pv->tx_q);
  dev_rq_queue_init(&pv->rx_q);
    
  pv->tx_rq.type = DEV_CHAR_WRITE_FRAME;
  pv->tx_rq.pvdata = dev;
  dev_char_rq_init(&pv->tx_rq, framer_io_tx_done);
  pv->tx_busy = 0;

  kroutine_init_interruptible(&pv->rx_service, framer_rx_serve);
  
  for (uint8_t off = 0; off < 2; ++off) {
    struct framer_rx_s *rx = pv->rx + off;

    rx->rx_frame = pv->tx_frame + pv->mtu * off;
    rx->rx_rq.type = DEV_CHAR_READ_FRAME;
    rx->rx_rq.pvdata = pv;
    dev_char_rq_init(&rx->rx_rq, framer_io_rx_done);
    rx->rx_busy = 0;
    rx->rx_data = NULL;
    rx->rx_size = 0;
  }

  return 0;

 err_mem:
  mem_free(pv);
  return err;
}

static
DEV_CLEANUP(char_framer_cleanup)
{
  struct framer_context_s *pv = dev->drv_pv;

  if (!dev_rq_queue_isempty(&pv->tx_q))
    return -EBUSY;

  if (!dev_rq_queue_isempty(&pv->rx_q))
    return -EBUSY;

  for (uint8_t off = 0; off < 2; ++off) {
    struct framer_rx_s *rx = pv->rx + off;

    if (rx->rx_busy)
      return -EBUSY;
  }

  if (pv->tx_busy)
    return -EBUSY;

  device_put_accessor(&pv->io.base);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(char_framer_drv, 0, "Char Framer", char_framer,
               DRIVER_CHAR_METHODS(char_framer));

DRIVER_REGISTER(char_framer_drv,
                DEV_ENUM_FDTNAME_ENTRY("char_framer"));


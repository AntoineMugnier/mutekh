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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2015

*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/char.h>

#if CONFIG_DRIVER_CHAR_MUX_READ_SIZE < 4
# error CONFIG_DRIVER_CHAR_MUX_READ_SIZE < 4
#endif

/*
  This driver provide multiples virtual char devices. Data streams from
  virtual devices are multiplexed on a single char device specified in
  the device resource named 'io'.

  The number of channels must be specified with an integer resource
  named 'channels'.

  When the CONFIG_DRIVER_CHAR_MUX_RX_FIFOS token is defined, an
  additional integer array resource must be present which specifies
  the size of the input fifos for each channel.

  Virtual char devices support read/write operations of partial data
  and framed data.

  Here is a sample static char_mux device declaration:

  DEV_DECLARE_STATIC(char_mux_dev, "mux", 0, char_mux_drv,
                     DEV_STATIC_RES_DEV_PARAM("io", "/uart"),
                     DEV_STATIC_RES_UINT_PARAM("channels", 4),
                     DEV_STATIC_RES_UINT_ARRAY_PARAM("rx_fifos", 32, 32, 16, 8)
                    );

  Here is the format of packets used to multiplex data on the io device:
  __________________________________________________________
  |  8   |  8  |   1   .  1  .  6   |     len      |  32   |
  | sync | byte|     --- ctrl ---   |   payload    |  hash |
  | 0x67 | len | start . end . chan | ... data ... |  fnv  |

  The fnv hash state is per channel and only reset when the start bit
  is set.
*/

enum char_mux_rx_state_e
{
  CHAR_MUX_RX_IDLE,
  CHAR_MUX_RX_OOSYNC,
  CHAR_MUX_RX_SYNC,
  CHAR_MUX_RX_LEN,
  CHAR_MUX_RX_PAYLOAD,
  CHAR_MUX_RX_PAYLOAD_END,
  CHAR_MUX_RX_CHKSUM,
};

enum char_mux_tx_state_e
{
  CHAR_MUX_TX_IDLE,
  CHAR_MUX_TX_HEADER,
  CHAR_MUX_TX_PAYLOAD,
  CHAR_MUX_TX_CHKSUM,
};

#define CTRL_START 0x80
#define CTRL_END   0x40
#define SYNC_BYTE  0x67

#define FNV_SIZE 4

#define CHAR_MUX_DEBUG(...) /* printk(__VA_ARGS__) */

struct char_mux_channel_s
{
  dev_request_queue_root_t read_q;
#ifdef CONFIG_DRIVER_CHAR_MUX_RX_FIFOS
  uint8_t *fifo;
  uint16_t fifo_size;
  uint16_t fifo_max;
  uint16_t fifo_frame;
#endif
  uint16_t rq_done;
  uint32_t rx_fnv;
  error_t error;
  uint8_t BITFIELD(started,1);
};

DRIVER_PV(struct char_mux_context_s
{
  struct device_char_s io;

  struct dev_char_rq_s write_rq;
  struct dev_char_rq_s read_rq;

  dev_request_queue_root_t write_q;
  uint32_t tx_fnv;
  uint16_t rx_pk_len;
  uint16_t rx_done;
  uint16_t tx_rq_done;
  uint8_t chan_count;
  uint8_t rx_ctrl;
  uint8_t rx_buf[CONFIG_DRIVER_CHAR_MUX_READ_SIZE];

  uint8_t tx_buf[4];
  uint8_t tx_pk_len;

  enum char_mux_tx_state_e BITFIELD(tx_state,4);
  enum char_mux_rx_state_e BITFIELD(rx_state,4);

  struct char_mux_channel_s chans[0];
});

static bool_t char_mux_start_rx(struct device_s *dev);
static bool_t char_mux_start_tx(struct device_s *dev);

static void char_mux_fnv_init(uint32_t *ck)
{
  CHAR_MUX_DEBUG("fnv init %p\n", ck);
  *ck = 0x811c9dc5;
}

static void char_mux_fnv_byte_update(uint32_t *ck, const uint8_t data)
{
  CHAR_MUX_DEBUG("fnv update %p %02x\n", ck, data);
  uint32_t c = *ck;
  c ^= data;
  c *= 0x01000193;
  *ck = c;
}

static void char_mux_fnv_update(uint32_t *ck, const uint8_t *data, size_t l)
{
  size_t i;
  for (i = 0; i < l; i++)
    char_mux_fnv_byte_update(ck, data[i]);
}

static void char_mux_chan_rx_error(struct device_s *dev, struct char_mux_channel_s *chan,
                                     error_t err)
{
  CHAR_MUX_DEBUG(">>> %p %s %i\n", dev, __func__, err);
  struct dev_char_rq_s * __restrict__ rq;

  if ((rq = dev_char_rq_head(&chan->read_q)))
    {
      rq->error = err;
      chan->rq_done = 0;
      dev_char_rq_pop(&chan->read_q);
      rq->base.drvdata = NULL;

      dev_char_rq_done(rq);
    }
  else
    {
      chan->error = err;
    }
  chan->started = 0;
#ifdef CONFIG_DRIVER_CHAR_MUX_RX_FIFOS
  chan->fifo_size = chan->fifo_frame = 0;
#endif
}

static void char_mux_rx_error(struct device_s *dev, error_t err)
{
  CHAR_MUX_DEBUG(">>> %p %s\n", dev, __func__);
  struct char_mux_context_s *pv = dev->drv_pv;
  uint_fast8_t i;

  /* abort next rx requests on all channels */
  for (i = 0; i < pv->chan_count; i++)
    char_mux_chan_rx_error(dev, pv->chans + i, err);
}

static error_t char_mux_try_read(struct device_s *dev, struct char_mux_channel_s * __restrict__ chan,
                                 const uint8_t *data, size_t *size, uint8_t end)
{
  struct dev_char_rq_s * __restrict__ rq;
  size_t l = *size, i, j = 0;
  CHAR_MUX_DEBUG(">>> %p %s\n", dev, __func__);

  while (1)
    {
      rq = dev_char_rq_head(&chan->read_q);

      if (rq == NULL)
        break;

      if ((rq->type & _DEV_CHAR_POLL) && j < l)
        goto done;

      /* copy data */
      for (i = chan->rq_done; i < rq->size && j < l; i++)
        rq->data[i] = data[j++];
      chan->rq_done = i;

#ifdef CONFIG_DRIVER_CHAR_MUX_RX_FIFOS
      if (!(rq->type & _DEV_CHAR_NONBLOCK))
#endif
        {
          if (!i)
            break;

          if ((rq->type & _DEV_CHAR_ALL) && i < rq->size)
            break;

          if (rq->type & _DEV_CHAR_FRAME)
            {
              if (j < l)
                return -ENOSPC;
              else if (!end)
                break;
            }
        }

      rq->data += i;
      rq->size -= i;

      /* end of read request on virtual char device */
      chan->rq_done = 0;
    done:
      dev_char_rq_pop(&chan->read_q);
      rq->base.drvdata = NULL;
      dev_char_rq_done(rq);
    }

  *size = j;
  return 0;
}

static DEV_CHAR_CANCEL(char_mux_cancel)
{
  struct device_s *dev = accessor->dev;
  CHAR_MUX_DEBUG(">>> %p %s\n", dev, __func__);
  struct char_mux_context_s *pv = dev->drv_pv;
  uint_fast8_t num = accessor->number;
  struct char_mux_channel_s *chan = pv->chans + num;
  error_t err = -ENOTSUP;

  LOCK_SPIN_IRQ(&dev->lock);

  switch (rq->type)
    {
    case DEV_CHAR_READ_FRAME:
    case DEV_CHAR_READ_PARTIAL:
    case DEV_CHAR_READ:
      err = -EBUSY;
      if (rq->base.drvdata == chan)
        {
          chan->rq_done = 0;
          dev_char_rq_remove(&chan->read_q, rq);
          rq->base.drvdata = NULL;
          err = 0;
        }
    default:
      break;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_CHAR_REQUEST(char_mux_request)
{
  struct device_s *dev = accessor->dev;
  struct char_mux_context_s *pv = dev->drv_pv;
  uint_fast8_t num = accessor->number;
  error_t err = 0;
  bool_t do_tx = 0, do_rx = 0;
  CHAR_MUX_DEBUG(">>> %p %s\n", dev, __func__);

  assert(rq->size);

  LOCK_SPIN_IRQ(&dev->lock);

  switch (rq->type & ~_DEV_CHAR_FLUSH)
    {
    case DEV_CHAR_READ_POLL:
      if (rq->size > 1)
        {
          err = -ENOTSUP;
          break;
        }
    case DEV_CHAR_READ_FRAME:
    case DEV_CHAR_READ_PARTIAL:
#ifdef CONFIG_DRIVER_CHAR_MUX_RX_FIFOS
    case DEV_CHAR_READ_NONBLOCK:
#endif
    case DEV_CHAR_READ: {
      /* restart RX on IO device if needed */
      do_rx = char_mux_start_rx(dev);

      struct char_mux_channel_s *chan = pv->chans + num;

      /* report channel pending error once */
      if ((err = chan->error))
        {
          chan->error = 0;
          break;
        }

#ifdef CONFIG_DRIVER_CHAR_MUX_RX_FIFOS
      size_t l = chan->fifo_size;
      /* there should be no pending request when the fifo contains data. */
      assert(l == 0 || dev_rq_queue_isempty(&chan->read_q));

      if ((rq->type & _DEV_CHAR_FRAME) && chan->fifo_frame != l)
        {
          /* return an error if more than one frame is stored in the
             fifo because the frame boundary can not be determined. */
          err = -EPIPE;
          l -= chan->fifo_frame;
          memmove(chan->fifo, chan->fifo + l, chan->fifo_frame);
          chan->fifo_size = chan->fifo_frame;
          break;
        }
#endif

      rq->error = 0;
      rq->base.drvdata = chan;
      dev_char_rq_pushback(&chan->read_q, rq);

#ifdef CONFIG_DRIVER_CHAR_MUX_RX_FIFOS
      /* try to read from channel fifo */
      if ((err = char_mux_try_read(dev, chan, chan->fifo, &l, !chan->started)))
        {
          chan->rq_done = 0;
          dev_char_rq_pop(&chan->read_q);
          rq->base.drvdata = NULL;
          break;
        }
      chan->fifo_size -= l;
      chan->fifo_frame = 0;
      memmove(chan->fifo, chan->fifo + l, chan->fifo_size);
#endif
      break;
    }

    case DEV_CHAR_WRITE_FRAME:
    case DEV_CHAR_WRITE_PARTIAL:
    case DEV_CHAR_WRITE: {
      rq->base.drvuint = num;
      dev_char_rq_pushback(&pv->write_q, rq);
      do_tx = char_mux_start_tx(dev);
      break;
    }
    case DEV_CHAR_WRITE_POLL:
      err = -ENOTSUP;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  if (do_rx)
    DEVICE_OP(&pv->io, request, &pv->read_rq);

  if (do_tx)
    DEVICE_OP(&pv->io, request, &pv->write_rq);

  if (err)
    {
      rq->error = err;
      dev_char_rq_done(rq);
    }
}

static void char_mux_rx_data(struct device_s *dev, size_t l)
{
  struct char_mux_context_s *pv = dev->drv_pv;
  CHAR_MUX_DEBUG(">>> %p %s\n", dev, __func__);
  const uint8_t *in = pv->rx_buf;
  bool_t begin = !pv->rx_done;
  uint8_t ctrl;
  error_t err = 0;

  if (begin)
    {
      assert(l != 0);
      pv->rx_ctrl = ctrl = pv->rx_buf[0];
      l--;
      in++;
    }
  else
    {
      ctrl = pv->rx_ctrl;
    }

  uint8_t ch = ctrl & 0x3f;

  if (ch >= pv->chan_count)
    return;

  struct char_mux_channel_s * __restrict__ chan = pv->chans + ch;

  if (l == 0)                  /* packet last chunk */
    {
      if (chan->rx_fnv != endian_le32_na_load(in))
        {
          err = -EBADDATA;
          goto err;
        }

      CHAR_MUX_DEBUG("%p read end: ch:%u l:%u err:%i\n", dev, ch, l, chan->error);
    }
  else
    {
      if (begin)                /* packet first chunk */
        {
          if (ctrl & CTRL_START)
            {
              chan->started = 1;
#ifdef CONFIG_DRIVER_CHAR_MUX_RX_FIFOS
              chan->fifo_frame = 0;
#endif
              char_mux_fnv_init(&chan->rx_fnv);
            }
          char_mux_fnv_byte_update(&chan->rx_fnv, pv->rx_pk_len - 1);
          char_mux_fnv_byte_update(&chan->rx_fnv, ctrl);
          CHAR_MUX_DEBUG("%p read begin: ch:%u s:%u\n", dev, ch, chan->started);
        }

      char_mux_fnv_update(&chan->rx_fnv, in, l);

      ctrl &= ~CTRL_END;
    }

  if (chan->started)
    {
      /* write as much data as possible to pending requests */
      size_t i = l;
      err = char_mux_try_read(dev, chan, in, &i, ctrl & CTRL_END);

      if (err)
        goto err;

#ifdef CONFIG_DRIVER_CHAR_MUX_RX_FIFOS
      /* write remaining data in channel rx fifo */
      size_t s = chan->fifo_size, j = s;
      for (; i < l && j < chan->fifo_max; i++)
        chan->fifo[j++] = in[i];
      chan->fifo_frame += j - s;
      chan->fifo_size = j;
#endif

      if (ctrl & CTRL_END)
        chan->started = 0;

      if (i < l)            /* channel overflow */
        {
          err = -EPIPE;
          goto err;
        }
    }
  return;

 err:
  char_mux_chan_rx_error(dev, chan, err);
}

static bool_t char_mux_start_rx(struct device_s *dev)
{
  struct char_mux_context_s *pv = dev->drv_pv;
  CHAR_MUX_DEBUG(">>> %p %s %u %u\n", dev, __func__, pv->tx_state, pv->rx_state);

  if (pv->rx_state == CHAR_MUX_RX_IDLE)
    {
      pv->read_rq.size = 2;
      pv->read_rq.data = pv->rx_buf;
      pv->rx_state = CHAR_MUX_RX_OOSYNC;
      return 1;
    }

  return 0;
}

static KROUTINE_EXEC(char_mux_io_read_done)
{
  struct dev_char_rq_s *rq = KROUTINE_CONTAINER(kr, *rq, base.kr);
  struct device_s *dev = rq->base.pvdata;
  struct char_mux_context_s *pv = dev->drv_pv;
  bool_t do_rx = 1;

  CHAR_MUX_DEBUG(">>> %p %s %u %u err:%u\n", dev, __func__, pv->tx_state, pv->rx_state, rq->error);

  LOCK_SPIN_IRQ(&dev->lock);

  switch (rq->error)
    {
    case -EIO:
    case -ENOENT:
    case -ENOTSUP: {
      uint_fast8_t i;
      char_mux_rx_error(dev, rq->error);

      /* do not start an io rx on permanent error when there is no
         more channel requests */
      for (i = 0; i < pv->chan_count; i++)
        if (!dev_rq_queue_isempty(&(pv->chans + i)->read_q))
          goto restart;
      pv->rx_state = CHAR_MUX_RX_IDLE;
      do_rx = 0;
      break;
    }

    default:
      char_mux_rx_error(dev, rq->error);
    restart:
      pv->read_rq.size = 2;
      pv->read_rq.data = pv->rx_buf;
      pv->rx_state = CHAR_MUX_RX_OOSYNC;
      break;

    case 0:
      switch (pv->rx_state)
        {
        case CHAR_MUX_RX_OOSYNC:
        case CHAR_MUX_RX_SYNC:
          if (pv->rx_buf[0] == SYNC_BYTE)
            {
            case CHAR_MUX_RX_LEN:
              {
                uint8_t len = pv->rx_buf[1];
                if (len > 0 && len <= CONFIG_DRIVER_CHAR_MUX_PACKET_SIZE)
                  {
                    CHAR_MUX_DEBUG("%p frame head : %P\n", dev, pv->rx_buf, 2);
                    pv->rx_state = CHAR_MUX_RX_PAYLOAD;
                    pv->rx_pk_len = 1 + len;
                    pv->rx_done = 0;
                    goto next_payload;
                  }
              }
            }

          /* out of sync */
          if (pv->rx_state == CHAR_MUX_RX_SYNC)
            char_mux_rx_error(dev, -EPIPE);

          pv->rx_state = CHAR_MUX_RX_OOSYNC;
          if (pv->rx_buf[1] == SYNC_BYTE)
            {
              pv->rx_state = CHAR_MUX_RX_LEN;
              pv->read_rq.size = 1;
              pv->read_rq.data = pv->rx_buf + 1;
            }
          else
            {
              pv->read_rq.size = 2;
              pv->read_rq.data = pv->rx_buf;
            }
          break;

        case CHAR_MUX_RX_PAYLOAD:
          char_mux_rx_data(dev, CONFIG_DRIVER_CHAR_MUX_READ_SIZE);
          pv->rx_done += CONFIG_DRIVER_CHAR_MUX_READ_SIZE;
        next_payload: {
            size_t l = pv->rx_pk_len - pv->rx_done;
            pv->read_rq.data = pv->rx_buf;
            if (l > CONFIG_DRIVER_CHAR_MUX_READ_SIZE)
              {
                pv->read_rq.size = CONFIG_DRIVER_CHAR_MUX_READ_SIZE;
              }
            else
              {
                pv->read_rq.size = l;
                pv->rx_state = CHAR_MUX_RX_PAYLOAD_END;
              }
          }
          break;

        case CHAR_MUX_RX_PAYLOAD_END: {
          size_t l = pv->rx_pk_len - pv->rx_done;
          CHAR_MUX_DEBUG("%p frame read : %P\n", dev, pv->rx_buf, l);
          char_mux_rx_data(dev, l);
          pv->rx_done = pv->rx_pk_len;
          pv->rx_state = CHAR_MUX_RX_CHKSUM;
          pv->read_rq.size = 4;
          pv->read_rq.data = pv->rx_buf;
          break;
        }

        case CHAR_MUX_RX_CHKSUM: {
          CHAR_MUX_DEBUG("%p chksum read : %P\n", dev, pv->rx_buf, 4);
          char_mux_rx_data(dev, 0);
          pv->rx_state = CHAR_MUX_RX_SYNC;
          pv->read_rq.size = 2;
          pv->read_rq.data = pv->rx_buf;
          break;
        }

        default:
          abort();
        }
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  if (do_rx)
    DEVICE_OP(&pv->io, request, &pv->read_rq);
}

static bool_t char_mux_start_tx(struct device_s *dev)
{
  struct char_mux_context_s *pv = dev->drv_pv;
  struct dev_char_rq_s *trq = dev_char_rq_head(&pv->write_q);

  CHAR_MUX_DEBUG(">>> %p %s %u %u\n", dev, __func__, pv->tx_state, pv->rx_state);

  if (pv->tx_state == CHAR_MUX_TX_IDLE && trq != NULL)
    {
      pv->write_rq.type = DEV_CHAR_WRITE | (trq->type & _DEV_CHAR_FLUSH);
      assert(trq->size);

      /* send header */
      pv->write_rq.data = pv->tx_buf;
      pv->tx_buf[0] = SYNC_BYTE;
      uint8_t ctrl = trq->base.drvuint;
      if (!pv->tx_rq_done || trq->type != DEV_CHAR_WRITE_FRAME)
        {
          char_mux_fnv_init(&pv->tx_fnv);
          ctrl |= CTRL_START;
        }
      uint8_t len = CONFIG_DRIVER_CHAR_MUX_PACKET_SIZE;
      if (trq->size - pv->tx_rq_done <= CONFIG_DRIVER_CHAR_MUX_PACKET_SIZE)
        {
          len = trq->size - pv->tx_rq_done;
          ctrl |= CTRL_END;
        }
      else if (trq->type != DEV_CHAR_WRITE_FRAME)
        ctrl |= CTRL_END;

      pv->tx_buf[1] = pv->tx_pk_len = len;
      char_mux_fnv_byte_update(&pv->tx_fnv, len);
      pv->tx_buf[2] = ctrl;
      char_mux_fnv_byte_update(&pv->tx_fnv, ctrl);
      pv->write_rq.size = 3;
      pv->tx_state = CHAR_MUX_TX_HEADER;

      return 1;
    }

  return 0;
}

static KROUTINE_EXEC(char_mux_io_write_done)
{
  struct dev_char_rq_s *rq = KROUTINE_CONTAINER(kr, *rq, base.kr);
  struct device_s *dev = rq->base.pvdata;
  struct char_mux_context_s *pv = dev->drv_pv;
  struct dev_char_rq_s *trq = dev_char_rq_head(&pv->write_q);
  bool_t do_tx = 1;

  CHAR_MUX_DEBUG(">>> %p %s %u %u err:%u\n", dev, __func__, pv->tx_state, pv->rx_state, rq->error);

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->error)
    {
      trq->error = rq->error;
      goto trq_end;
    }

  switch (pv->tx_state)
    {
    case CHAR_MUX_TX_HEADER:
      /* send payload */
      pv->write_rq.data = trq->data + pv->tx_rq_done;
      pv->write_rq.size = pv->tx_pk_len;
      pv->tx_rq_done += pv->tx_pk_len;
      pv->tx_state = CHAR_MUX_TX_PAYLOAD;
      char_mux_fnv_update(&pv->tx_fnv, pv->write_rq.data, pv->tx_pk_len);
      break;
    case CHAR_MUX_TX_PAYLOAD:
      /* send checksum */
      endian_le32_na_store(pv->tx_buf, pv->tx_fnv);
      pv->write_rq.data = pv->tx_buf;
      pv->write_rq.size = 4;
      pv->tx_state = CHAR_MUX_TX_CHKSUM;
      break;
    case CHAR_MUX_TX_CHKSUM:
      if (pv->tx_rq_done == trq->size ||
          (trq->type & _DEV_CHAR_PARTIAL))
        {
          /* end of write request on virtual char device */
          trq->data += pv->tx_rq_done;
          trq->size -= pv->tx_rq_done;
          trq->error = 0;
        trq_end:
          pv->tx_rq_done = 0;
          dev_char_rq_pop(&pv->write_q);
          dev_char_rq_done(trq);
        }
      pv->tx_state = CHAR_MUX_TX_IDLE;
      do_tx = char_mux_start_tx(dev);
      break;

    default:
      abort();
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  if (do_tx)
    DEVICE_OP(&pv->io, request, &pv->write_rq);
}


static DEV_USE(char_mux_use)
{
  struct device_accessor_s *accessor = param;

  switch (op)
    {
    case DEV_USE_GET_ACCESSOR: {
      struct device_s *dev = accessor->dev;
      struct char_mux_context_s *pv = dev->drv_pv;
      if (accessor->number >= pv->chan_count)
        return -ENOTSUP;
    }
    case DEV_USE_PUT_ACCESSOR:
      return 0;
    case DEV_USE_LAST_NUMBER: {
      struct device_s *dev = accessor->dev;
      struct char_mux_context_s *pv = dev->drv_pv;
      accessor->number = pv->chan_count - 1;
      return 0;
    }
    case DEV_USE_START: {
      struct device_s *dev = accessor->dev;
      struct char_mux_context_s *pv = dev->drv_pv;
      if (dev->start_count == 0)
        return device_start(&pv->io.base);
    }
    case DEV_USE_STOP: {
      struct device_s *dev = accessor->dev;
      struct char_mux_context_s *pv = dev->drv_pv;
      if (dev->start_count == 0)
        device_stop(&pv->io.base);
      return 0;
    }

    default:
      return dev_use_generic(param, op);
    }
}

static DEV_INIT(char_mux_init)
{
  struct char_mux_context_s	*pv;
  uintptr_t num;
  uint_fast8_t i;


  if (device_get_param_uint(dev, "channels", &num) || num < 1)
    return -EINVAL;

  size_t fall = 0;
#ifdef CONFIG_DRIVER_CHAR_MUX_RX_FIFOS
  const uintptr_t *fsizes;
  uint16_t fcnt;
  if (device_get_param_uint_array(dev, "rx_fifos", &fcnt, &fsizes) || fcnt != num)
    return -EINVAL;

  for (i = 0; i < num; i++)
    {
      if (fsizes[i] > 4096)
        return -EINVAL;
      fall += fsizes[i];
    }
#endif

  pv = mem_alloc(sizeof(*pv)
                 + num * sizeof(struct char_mux_channel_s)
                 + num * fall, mem_scope_sys);
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  pv->chan_count = num;

  if (device_get_param_dev_accessor(dev, "io", &pv->io.base, DRIVER_CLASS_CHAR))
    goto err_mem;

  dev_rq_queue_init(&pv->write_q);

  __unused__ uint8_t *fifo = (void*)(pv->chans + num);
  for (i = 0; i < num; i++)
    {
      struct char_mux_channel_s *chan = pv->chans + i;

      dev_rq_queue_init(&chan->read_q);

#ifdef CONFIG_DRIVER_CHAR_MUX_RX_FIFOS
      size_t fifo_size = fsizes[i];
      chan->rq_done = 0;
      chan->fifo = fifo;
      chan->fifo_size = chan->fifo_frame = 0;
      chan->fifo_max = fifo_size;
      chan->error = 0;
      chan->started = 0;
      fifo += fifo_size;
#endif
    }

  pv->read_rq.type = DEV_CHAR_READ;
  pv->read_rq.base.pvdata = dev;
  dev_char_rq_init(&pv->read_rq, char_mux_io_read_done);

  pv->write_rq.base.pvdata = dev;
  dev_char_rq_init(&pv->write_rq, char_mux_io_write_done);

  pv->tx_rq_done = 0;
  pv->tx_state = CHAR_MUX_TX_IDLE;
  pv->rx_state = CHAR_MUX_RX_IDLE;

  return 0;

 err_mem:
  mem_free(pv);
  return -EINVAL;
}

static DEV_CLEANUP(char_mux_cleanup)
{
  struct char_mux_context_s *pv = dev->drv_pv;
  uint_fast8_t i;

  if (pv->tx_state != CHAR_MUX_TX_IDLE ||
      !dev_rq_queue_isempty(&pv->write_q))
    return -EBUSY;

  for (i = 0; i < pv->chan_count; i++)
    if (!dev_rq_queue_isempty(&(pv->chans + i)->read_q))
      return -EBUSY;

  if (pv->rx_state != CHAR_MUX_RX_IDLE &&
      DEVICE_OP(&pv->io, cancel, &pv->read_rq))
    return -EBUSY;

  device_put_accessor(&pv->io.base);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(char_mux_drv, 0, "Char Mux", char_mux,
               DRIVER_CHAR_METHODS(char_mux));

DRIVER_REGISTER(char_mux_drv,
                DEV_ENUM_FDTNAME_ENTRY("char_mux"));


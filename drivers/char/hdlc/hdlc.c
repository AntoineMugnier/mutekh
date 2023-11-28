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

  Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2015
  Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2023
*/

#define LOGK_MODULE_ID "hdlc"

#include <hexo/types.h>
#include <hexo/endian.h>
#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/char.h>

#if CONFIG_DRIVER_CHAR_HDLC_RX_SIZE < 4
# error CONFIG_DRIVER_CHAR_HDLC_RX_SIZE < 4
#endif

#if CONFIG_DRIVER_CHAR_HDLC_TX_SIZE < 4
# error CONFIG_DRIVER_CHAR_HDLC_TX_SIZE < 4
#endif

GCT_CONTAINER_FCNS(dev_request_queue, inline, __hdlc_rq_queue,
                   push);

/*
  This driver provides multiple virtual char devices handling
  frames. Frames from virtual devices are multiplexed on a single char
  stream device specified in the device resource named 'io'.

  Virtual char devices support read/write operations of framed data
  only.  Frames sent and received on the char interface will contain
  the complete HDLC header and payload (address, command and
  payload).

  Device accessor number is used to route incoming data depending on
  address field.  The number of virtual channels and received address
  mapping may be specified with an integer resource named 'rx_map'.

  If no @tt rx_map resource is given, only one device index is
  implemented and it receives all the addresses.  In such case, this
  effectively makes the HDLC device a reliable frame to serial line
  transport.

  On reception, a frame is dropped silently if either:
  @list
  @item CRC check fails
  @item frame is too short (less than two bytes long)
  @item frame is too long (does not fit the read request buffer)
  @end list

  As HDLC frames only contain address of receiver, any accessor may
  send equivalent frames to the HDLC mux. Device index is unused on
  TX. Any transmitted frame must be at least two bytes (address,
  command).

  On transmition, HDLC flag byte is not repeated if two frames are
  sent back-to-back (i.e. another write request is present in queue
  when inter-frame flag is emitted). In other cases, two flag bytes
  are sent, one when first packet ends (to flush it early) and one
  when second packet starts (for out-of-sync receivers).  No periodic
  HDLC flag byte stuffing is performed.

  Here is a sample static HDLC device declaration:

  @code
    #include <drivers/char/hdlc.h>

    // Device accessor [0] receives frames to 0xff
    // Device accessor [1] receives frames to 0x00
    // Device accessor [2] receives frames to 0x02
    // Frames to other addresses are dropped
    static DRIVER_HDLC_RX_MAP(hdlc_dev_rx_map, 0xff, 0x00, 0x02);

    DEV_DECLARE_STATIC(hdlc_dev, "hdlc", 0, hdlc_drv,
                       DEV_STATIC_RES_DEV_PARAM("io", "/uart"),
                       DEV_STATIC_RES_BLOB_PARAM("rx_map", hdlc_dev_rx_map),
                      );
  @end code
*/

enum hdlc_rx_state_e
{
  HDLC_RX_IDLE, // Not reading
  HDLC_RX_DROP, // Reading but unsync or ignoring payload
  HDLC_RX_ADDR,
  HDLC_RX_CMD,
  HDLC_RX_DATA,
};

enum hdlc_tx_state_e
{
  HDLC_TX_IDLE,
  HDLC_TX_PAYLOAD,
  HDLC_TX_CRC,
  HDLC_TX_FLAG,
};

#define HDLC_FLAG   0x7e
#define HDLC_ESCAPE 0x7d
#define HDLC_MANGLE 0x20

static inline bool_t hdlc_is_escaped(uint8_t b)
{
  switch (b) {
  case 0x7e:
  case 0x7d:
  case 0x03:
  case 0x11:
  case 0x13:
  case 0x91:
  case 0x93:
    return 1;
  default:
    return 0;
  }
}

struct hdlc_channel_s
{
  dev_request_queue_root_t rx_q;
};

struct hdlc_rx_context_s
{
  bool_t BITFIELD(busy, 1);
  uint8_t buf[CONFIG_DRIVER_CHAR_HDLC_RX_SIZE];
  struct dev_char_rq_s rq;
};

STRUCT_COMPOSE(hdlc_rx_context_s, rq);

typedef uint16_t hdlc_crc_state_t;
typedef uint16_t hdlc_crc_final_t;

struct hdlc_context_s
{
  struct device_char_s io;

  const uint8_t *rx_map;
  struct hdlc_rx_context_s rx_ctx[CONFIG_DRIVER_CHAR_HDLC_RX_BUFFER_COUNT];

  dev_request_queue_root_t tx_q;
  struct dev_char_rq_s *tx_current;

  struct dev_char_rq_s *rx_current;
  size_t rx_offset;

  hdlc_crc_state_t rx_crc, tx_crc;

  struct dev_char_rq_s tx_rq;
  uint8_t tx_buf[CONFIG_DRIVER_CHAR_HDLC_TX_SIZE];

  enum hdlc_tx_state_e BITFIELD(tx_state,2);
  enum hdlc_rx_state_e BITFIELD(rx_state,3);
  bool_t BITFIELD(rx_escaped, 1);
  bool_t BITFIELD(tx_busy, 1);
  uint8_t chan_count_m1;
  uint8_t rx_shreg[2];

  struct hdlc_channel_s chans[0];
};
  
DRIVER_PV(struct hdlc_context_s);

static const hdlc_crc_state_t hdlc_crc_init = 0x0;
static const hdlc_crc_state_t hdlc_crc_check_state = ~0xf47;

static inline
hdlc_crc_state_t hdlc_crc_insert4(hdlc_crc_state_t state, uint8_t word)
{
    static const hdlc_crc_state_t state_update_table[] = {
        0x0000, 0x1081, 0x2102, 0x3183,
        0x4204, 0x5285, 0x6306, 0x7387,
        0x8408, 0x9489, 0xa50a, 0xb58b,
        0xc60c, 0xd68d, 0xe70e, 0xf78f,
    };

    const uint8_t index = (state ^ word) & 0xf;
    const hdlc_crc_state_t shifted_state = (state >> 4) & 0xffff;
    return shifted_state ^ state_update_table[index];
}

static
hdlc_crc_final_t hdlc_crc_initialize(hdlc_crc_state_t init)
{
    hdlc_crc_state_t state = 0xffff ^ init;
    return state;
}

static
hdlc_crc_state_t hdlc_crc_update(hdlc_crc_state_t state, const uint8_t *data, size_t size)
{
    for (size_t i = 0; i < size; ++i) {
        const uint8_t word = data[i];
        for (int8_t b = 0; b < 8; b += 4)
            state = hdlc_crc_insert4(state, (word >> b) & 0xf);
    }

    return state;
}

static
hdlc_crc_final_t hdlc_crc_finalize(hdlc_crc_state_t state)
{
    state = 0xffff ^ state;
    return state;
}

static
void hdlc_crc_serialize(uint8_t dest[static 2], hdlc_crc_state_t final_value)
{
    for (ssize_t i = 0; i < 2; ++i) {
        dest[i] = final_value & 0xff;
        final_value >>= 8;
    }
}

static
DEV_CHAR_CANCEL(hdlc_cancel)
{
  struct device_s *dev = accessor->dev;
  logk_debug("cancel %p", rq);
  struct hdlc_context_s *pv = dev->drv_pv;
  uint_fast8_t num = accessor->number;
  struct hdlc_channel_s *chan = pv->chans + num;

  if (rq->base.drvdata != chan)
    return -ENOENT;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  switch (rq->type)
    {
    case DEV_CHAR_READ_FRAME:
      if (rq == pv->rx_current)
        return -EBUSY;

      rq->base.drvdata = NULL;
      dev_char_rq_remove(&chan->rx_q, rq);
      return 0;

    case DEV_CHAR_WRITE_FRAME:
      if (rq == pv->tx_current)
        return -EBUSY;

      rq->base.drvdata = NULL;
      dev_char_rq_remove(&pv->tx_q, rq);
      return 0;
      
    default:
      return -ENOTSUP;
    }
}

static
void hdlc_rx_schedule(struct hdlc_context_s *pv)
{
  for (size_t i = 0; i < CONFIG_DRIVER_CHAR_HDLC_RX_BUFFER_COUNT; ++i) {
    struct hdlc_rx_context_s *rx = pv->rx_ctx + i;
    
    if (rx->busy)
      continue;

    rx->rq.size = CONFIG_DRIVER_CHAR_HDLC_RX_SIZE;
    rx->rq.data = rx->buf;
    rx->busy = 1;
    DEVICE_OP(&pv->io, request, &rx->rq);
  }
}

static
size_t hdlc_tx_buf_available(struct hdlc_context_s *pv)
{
  return CONFIG_DRIVER_CHAR_HDLC_TX_SIZE - pv->tx_rq.size;
}

static
void hdlc_tx_buf_pushback(struct hdlc_context_s *pv, uint8_t c)
{
  assert(hdlc_tx_buf_available(pv));

  pv->tx_rq.data[pv->tx_rq.size++] = c;
}

static
void hdlc_tx_prepare(struct hdlc_context_s *pv)
{
  if (pv->tx_busy)
    return;

  pv->tx_rq.size = 0;
  pv->tx_rq.data = pv->tx_buf;

  if (!pv->tx_current) {
    pv->tx_current = dev_char_rq_pop(&pv->tx_q);
    if (!pv->tx_current) {
      pv->tx_state = HDLC_TX_IDLE;
      return;
    }
  }

  switch (pv->tx_state) {
  case HDLC_TX_IDLE:
    hdlc_tx_buf_pushback(pv, HDLC_FLAG);
    pv->tx_state = HDLC_TX_PAYLOAD;
    pv->tx_crc = hdlc_crc_initialize(hdlc_crc_init);

    // fallthrough
  case HDLC_TX_PAYLOAD:
  payload:
    assert(pv->tx_current);

    while (pv->tx_current->size != 0) {
      uint8_t to_tx = *pv->tx_current->data;
      bool_t to_escape = hdlc_is_escaped(to_tx);
      
      if (hdlc_tx_buf_available(pv) < 1 + to_escape)
        break;

      pv->tx_crc = hdlc_crc_update(pv->tx_crc, &to_tx, 1);
      if (to_escape) {
        hdlc_tx_buf_pushback(pv, HDLC_ESCAPE);
        hdlc_tx_buf_pushback(pv, to_tx ^ HDLC_MANGLE);
      } else {
        hdlc_tx_buf_pushback(pv, to_tx);
      }

      pv->tx_current->data++;
      pv->tx_current->size--;
    }

    if (pv->tx_current->size != 0)
      break;

    pv->tx_state = HDLC_TX_CRC;

    // fallthrough
  case HDLC_TX_CRC: {
    uint8_t to_tx[2];

    hdlc_crc_serialize(to_tx, hdlc_crc_finalize(pv->tx_crc));

    bool_t to_escape0 = hdlc_is_escaped(to_tx[0]);
    bool_t to_escape1 = hdlc_is_escaped(to_tx[1]);
    size_t needed = to_escape0 + to_escape1 + 2;
    if (needed > hdlc_tx_buf_available(pv))
      break;

    if (to_escape0) {
      hdlc_tx_buf_pushback(pv, HDLC_ESCAPE);
      hdlc_tx_buf_pushback(pv, to_tx[0] ^ HDLC_MANGLE);
    } else {
      hdlc_tx_buf_pushback(pv, to_tx[0]);
    }

    if (to_escape1) {
      hdlc_tx_buf_pushback(pv, HDLC_ESCAPE);
      hdlc_tx_buf_pushback(pv, to_tx[1] ^ HDLC_MANGLE);
    } else {
      hdlc_tx_buf_pushback(pv, to_tx[1]);
    }

    pv->tx_state = HDLC_TX_FLAG;

    pv->tx_current->base.drvdata = NULL;
    pv->tx_current->base.error = 0;
    logk_debug("tx rq done %p", pv->tx_current);
    dev_char_rq_done(pv->tx_current);
    pv->tx_current = NULL;
    // fallthrough
  }

  case HDLC_TX_FLAG:
    if (hdlc_tx_buf_available(pv) < 1)
      break;

    hdlc_tx_buf_pushback(pv, HDLC_FLAG);

    if (!pv->tx_current) {
      pv->tx_current = dev_char_rq_pop(&pv->tx_q);
      if (!pv->tx_current) {
        pv->tx_state = HDLC_TX_IDLE;
        break;
      }
    }

    pv->tx_state = HDLC_TX_PAYLOAD;
    goto payload;
  }
  
  if (pv->tx_rq.size == 0)
    return;

  pv->tx_busy = 1;
  logk_trace("< %P", pv->tx_rq.data, pv->tx_rq.size);
  DEVICE_OP(&pv->io, request, &pv->tx_rq);
}

static
int_fast16_t hdlc_channel_for_address(struct hdlc_context_s *pv,
                                 uint8_t address)
{
  if (!pv->rx_map)
    return 0;

  for (uint_fast16_t chan = 0; chan <= pv->chan_count_m1; ++chan) {
    if (pv->rx_map[chan] == address)
      return chan;
  }

  return -1;
}
  
static
void hdlc_rx_process_one(struct hdlc_context_s *pv, bool_t flag, uint8_t d)
{
  switch (pv->rx_state) {
  case HDLC_RX_IDLE:
    pv->rx_state = HDLC_RX_DROP;
    // fallthrough

  case HDLC_RX_DROP:
    if (flag) {
      logk_trace("- drop flag");
      pv->rx_state = HDLC_RX_ADDR;
    } else {
      logk_trace("- drop %02x", d);
    }
    return;

  case HDLC_RX_ADDR:
    if (flag) {
      logk_trace("> addr flag");
      return;
    }

    logk_trace("> addr %02x", d);
    pv->rx_crc = hdlc_crc_update(hdlc_crc_initialize(hdlc_crc_init), &d, 1);
    pv->rx_shreg[0] = d;
    pv->rx_state = HDLC_RX_CMD;
    return;

  case HDLC_RX_CMD:
    assert(!pv->rx_current);

    if (flag) {
      // Reset before two useful bytes
      logk_trace("> cmd flag");
      pv->rx_state = HDLC_RX_ADDR;
      return;
    }

    logk_trace("> cmd %02x", d);
    pv->rx_crc = hdlc_crc_update(pv->rx_crc, &d, 1);
    pv->rx_shreg[1] = d;
    pv->rx_state = HDLC_RX_DATA;

    int_fast16_t channel = hdlc_channel_for_address(pv, pv->rx_shreg[0]);
    if (channel < 0) {
      logk_trace("* No channel handler for address %d", pv->rx_shreg[0]);
      // Channel does not exist, ignore frame
      pv->rx_state = HDLC_RX_DROP;
      return;
    }

    struct hdlc_channel_s *chan = &pv->chans[channel];
    pv->rx_current = dev_char_rq_pop(&chan->rx_q);
    if (!pv->rx_current) {
      logk_trace("* No request for address %d channel %d",
                 pv->rx_shreg[0], channel);
      // no RX RQ in queue
      pv->rx_state = HDLC_RX_DROP;
      return;
    }

    pv->rx_offset = 0;
    return;

  case HDLC_RX_DATA:
    assert(pv->rx_current);

    if (flag) {
      pv->rx_state = HDLC_RX_ADDR;

      // Short frames are invalid (header and CRC collide).  By
      // chance, header may be a valid CRC.  User would receive a
      // frame with less than the minimal 2-byte header
      if (pv->rx_offset < 2) {
        logk_trace("* short frame");
        goto rx_drop;
      }

      if (pv->rx_crc != hdlc_crc_check_state) {
        logk_trace("* bad crc %04x expected %04x", pv->rx_crc, hdlc_crc_check_state);
        goto rx_drop;
      }

      logk_trace("> frame OK: %P", pv->rx_current->data, pv->rx_offset);
      goto rx_done;
    }

    if (pv->rx_offset >= pv->rx_current->size) {
      logk_trace("* data %02x overflow", d);
      pv->rx_state = HDLC_RX_DROP;
      goto rx_drop;
    }

    logk_trace("> data %02x <= %02x", pv->rx_shreg[0], d);
    pv->rx_current->data[pv->rx_offset++] = pv->rx_shreg[0];
    pv->rx_crc = hdlc_crc_update(pv->rx_crc, &d, 1);
    pv->rx_shreg[0] = pv->rx_shreg[1];
    pv->rx_shreg[1] = d;
    return;
  }

  return;

 rx_drop: {
    struct hdlc_channel_s *chan = pv->rx_current->base.drvdata;
    logk_trace("rx drop");
    __hdlc_rq_queue_push(&chan->rx_q, &pv->rx_current->base);
    pv->rx_current = NULL;
    return;
  }
  
 rx_done: {
    pv->rx_current->data += pv->rx_offset;
    pv->rx_current->size -= pv->rx_offset;
    pv->rx_current->base.drvdata = NULL;
    pv->rx_current->base.error = 0;

    logk_debug("rx rq done %p", pv->rx_current);
    dev_char_rq_done(pv->rx_current);
    pv->rx_current = NULL;
    return;
  }
}

static
void hdlc_rx_process(struct hdlc_context_s *pv,
                     const uint8_t *data, size_t size)
{
  for (size_t i = 0; i < size; ++i) {
    uint8_t d = data[i];
    // d maybe HDLC_FLAG after escaping, keep standalone flag on its
    // own
    bool_t flag = 0;

    if (d == HDLC_ESCAPE) {
      // If already escaped, this is an error. Dont handle this for
      // now.
      pv->rx_escaped = 1;
      continue;
    } else if (d == HDLC_FLAG) {
      flag = 1;
      // If in escape, this should be considered as bad data. Dont
      // handle this for now.
      pv->rx_escaped = 0;
    } else if (pv->rx_escaped) {
      pv->rx_escaped = 0;
      d ^= HDLC_MANGLE;
      // We should rely on d to test it against FLAG or other
      // escapable value after this
    }

    hdlc_rx_process_one(pv, flag, d);
  }
}

static
DEV_CHAR_REQUEST(hdlc_request)
{
  struct device_s *dev = accessor->dev;
  struct hdlc_context_s *pv = dev->drv_pv;
  uint_fast8_t num = accessor->number;
  logk_debug("rq %p num %d", rq, num);

  if (rq->size < 2) {
    rq->error = -EINVAL;
    logk_debug("Rq too short");
    dev_char_rq_done(rq);
    return;
  }
  
  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  struct hdlc_channel_s *chan = pv->chans + num;

  switch (rq->type & ~_DEV_CHAR_FLUSH) {
  case DEV_CHAR_READ_FRAME: {
    bool_t start = dev_rq_queue_isempty(&chan->rx_q);
    dev_char_rq_pushback(&chan->rx_q, rq);
    rq->base.drvdata = chan;
    rq->base.error = 0;

    logk_debug("Rx");

    if (start)
      hdlc_rx_schedule(pv);
    return;
  }

  case DEV_CHAR_WRITE_FRAME: {
    bool_t start = dev_rq_queue_isempty(&pv->tx_q);
    dev_char_rq_pushback(&pv->tx_q, rq);
    rq->base.drvdata = chan;
    rq->base.error = 0;

    logk_debug("Tx");

    if (start)
      hdlc_tx_prepare(pv);
    return;
  }

  default:
    rq->error = -ENOTSUP;
    dev_char_rq_done(rq);
  }
}

static
KROUTINE_EXEC(hdlc_io_rx_done)
{
  struct dev_char_rq_s *rq = dev_char_rq_from_kr(kr);
  struct hdlc_rx_context_s *rx = hdlc_rx_context_s_from_rq(rq);
  struct device_s *dev = rq->pvdata;
  struct hdlc_context_s *pv = dev->drv_pv;

  logk_debug("rx io done %d err %d", pv->rx_state, rq->error);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  rx->busy = 0;

  switch (rq->error)
    {
    default:
      pv->rx_state = HDLC_RX_DROP;
      goto restart;

    case -EIO:
    case -ENOENT:
    case -ENOTSUP:
      pv->rx_state = HDLC_RX_DROP;
      /* do not start an io rx on permanent error when there is no
         more channel requests */
      for (uint_fast16_t i = 0; i <= pv->chan_count_m1; i++)
        if (!dev_rq_queue_isempty(&(pv->chans + i)->rx_q))
          goto restart;

      pv->rx_state = HDLC_RX_IDLE;

    restart:
      if (pv->rx_current) {
        pv->rx_current->base.error = rq->error;
        pv->rx_current->base.drvdata = NULL;
        dev_char_rq_done(pv->rx_current);
        pv->rx_current = NULL;
        pv->rx_state = HDLC_RX_DROP;
      }
      break;

    case 0:
      hdlc_rx_process(pv, rx->buf, rx->rq.data - rx->buf);
      break;
    }

  if (pv->rx_state != HDLC_RX_IDLE)
    hdlc_rx_schedule(pv);
}

static
KROUTINE_EXEC(hdlc_io_tx_done)
{
  struct dev_char_rq_s *rq = dev_char_rq_from_kr(kr);
  struct device_s *dev = rq->pvdata;
  struct hdlc_context_s *pv = dev->drv_pv;

  logk_debug("tx io done %d err %d", pv->tx_state, rq->error);
  
  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  pv->tx_busy = 0;
  hdlc_tx_prepare(pv);
}


static
DEV_USE(hdlc_use)
{
  struct device_accessor_s *accessor = param;

  switch (op)
    {
    case DEV_USE_GET_ACCESSOR: {
      struct device_s *dev = accessor->dev;
      struct hdlc_context_s *pv = dev->drv_pv;
      if (accessor->number > pv->chan_count_m1)
        return -ENOTSUP;
    }

    case DEV_USE_PUT_ACCESSOR:
      return 0;

    case DEV_USE_LAST_NUMBER: {
      struct device_s *dev = accessor->dev;
      struct hdlc_context_s *pv = dev->drv_pv;
      accessor->number = pv->chan_count_m1;
      return 0;
    }

    case DEV_USE_START: {
      struct device_s *dev = accessor->dev;
      struct hdlc_context_s *pv = dev->drv_pv;
      if (dev->start_count == 0)
        return device_start(&pv->io.base);
    }

    case DEV_USE_STOP: {
      struct device_s *dev = accessor->dev;
      struct hdlc_context_s *pv = dev->drv_pv;
      if (dev->start_count == 0)
        device_stop(&pv->io.base);
      return 0;
    }

    default:
      return dev_use_generic(param, op);
    }
}

static
DEV_INIT(hdlc_init)
{
  struct hdlc_context_s	*pv;
  size_t pv_size, channel_count;
  error_t err;
  const uint8_t *rx_map;

  err = device_get_param_blob(dev, "rx_map", 0, (const void **)&rx_map);
  if (err && rx_map) {
    channel_count = 1;
    rx_map = NULL;
  } else {
    channel_count = rx_map[0] + 1;
    rx_map += 1;
  }

  if (channel_count < 1 || channel_count > 256)
    return -EINVAL;

  pv_size = sizeof(*pv) + channel_count * sizeof(struct hdlc_channel_s);
  pv = mem_alloc(pv_size, mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, pv_size);
  dev->drv_pv = pv;
  pv->chan_count_m1 = channel_count - 1;
  pv->rx_map = rx_map;

  err = device_get_param_dev_accessor(dev, "io", &pv->io.base, DRIVER_CLASS_CHAR);
  if (err)
    goto err_mem;

  dev_rq_queue_init(&pv->tx_q);

  for (uint_fast16_t i = 0; i < channel_count; i++)
    {
      struct hdlc_channel_s *chan = pv->chans + i;

      dev_rq_queue_init(&chan->rx_q);
    }

  for (size_t i = 0; i < CONFIG_DRIVER_CHAR_HDLC_RX_BUFFER_COUNT; ++i) {
    struct hdlc_rx_context_s *rx = pv->rx_ctx + i;
    
    rx->rq.type = DEV_CHAR_READ_PARTIAL;
    rx->rq.pvdata = dev;
    dev_char_rq_init(&rx->rq, hdlc_io_rx_done);
    rx->busy = 0;
  }

  pv->tx_rq.pvdata = dev;
  pv->tx_rq.type = DEV_CHAR_WRITE;
  dev_char_rq_init(&pv->tx_rq, hdlc_io_tx_done);

  pv->tx_busy = 0;
  pv->tx_state = HDLC_TX_IDLE;
  pv->rx_state = HDLC_RX_IDLE;

  return 0;

 err_mem:
  mem_free(pv);
  return err;
}

static
DEV_CLEANUP(hdlc_cleanup)
{
  struct hdlc_context_s *pv = dev->drv_pv;

  if (pv->tx_state != HDLC_TX_IDLE ||
      !dev_rq_queue_isempty(&pv->tx_q))
    return -EBUSY;

  for (uint_fast16_t i = 0; i <= pv->chan_count_m1; i++)
    if (!dev_rq_queue_isempty(&(pv->chans + i)->rx_q))
      return -EBUSY;

  for (size_t i = 0; i < CONFIG_DRIVER_CHAR_HDLC_RX_BUFFER_COUNT; ++i) {
    struct hdlc_rx_context_s *rx = pv->rx_ctx + i;

    if (rx->busy)
      return -EBUSY;
  }

  if (pv->tx_busy)
    return -EBUSY;
  
  device_put_accessor(&pv->io.base);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(hdlc_drv, 0, "HDLC", hdlc,
               DRIVER_CHAR_METHODS(hdlc));

DRIVER_REGISTER(hdlc_drv,
                DEV_ENUM_FDTNAME_ENTRY("hdlc"));


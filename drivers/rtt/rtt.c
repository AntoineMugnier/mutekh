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

#include <mutek/startup.h>

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <drivers/rtt/rtt.h>

struct {
  char id[16];
  uint32_t tx_channel_count;
  uint32_t rx_channel_count;
  struct rtt_channel_s channel[CONFIG_DRIVER_RTT_TX_CHANNEL_COUNT + CONFIG_DRIVER_RTT_RX_CHANNEL_COUNT];
} rtt_control;

uint32_t rtt_channel_write(
  struct rtt_channel_s *chan,
  const uint8_t *buf, uint32_t len)
{
  uint32_t available;
  uint32_t rptr, wptr;
  uint32_t copied = 0;

  while (copied < len) {
    wptr = cpu_mem_read_32((uintptr_t)&chan->write_ptr);
    rptr = cpu_mem_read_32((uintptr_t)&chan->read_ptr);

    if (wptr < rptr)
      available = rptr - wptr - 1;
    else
      available = chan->buffer_size - 1 - wptr + rptr;

    switch (chan->flags & RTT_CHANNEL_MODE_MASK) {
    case RTT_CHANNEL_MODE_BLOCKING:
      if (available == 0)
        continue;
      break;

    case RTT_CHANNEL_MODE_TRIM:
      if (available == 0)
        return copied;
      break;

    case RTT_CHANNEL_MODE_SKIP:
      if (available < len)
        return 0;
      break;

    default:
      break;
    }

    uint32_t to_copy = __MIN(available, len - copied);
    uint32_t to_copy1 = __MIN(chan->buffer_size - wptr, to_copy);
    uint32_t to_copy2 = to_copy - to_copy1;

    memcpy(chan->buffer + wptr, buf + copied, to_copy1);

    if (to_copy2) {
      memcpy(chan->buffer, buf + copied + to_copy1, to_copy2);
      wptr = to_copy2;
    } else {
      wptr += to_copy1;

      if (wptr == chan->buffer_size)
        wptr = 0;
    }

    assert(wptr < chan->buffer_size);

    cpu_mem_write_32((uintptr_t)&chan->write_ptr, wptr);

    copied += to_copy;
  }

  return copied;
}

uint32_t rtt_channel_read(
  struct rtt_channel_s *chan,
  uint8_t *buf, uint32_t len)
{
  uint32_t available;
  uint32_t rptr, wptr;

  wptr = cpu_mem_read_32((uintptr_t)&chan->write_ptr);
  rptr = cpu_mem_read_32((uintptr_t)&chan->read_ptr);

  if (rptr <= wptr)
    available = wptr - rptr;
  else
    available = chan->buffer_size - rptr + wptr;

  if (!available || !len)
    return 0;

  uint32_t to_copy = __MIN(available, len);
  uint32_t to_copy1 = __MIN(chan->buffer_size - rptr, to_copy);
  uint32_t to_copy2 = to_copy - to_copy1;

  memcpy(buf, chan->buffer + rptr, to_copy1);

  if (to_copy2) {
    memcpy(buf + to_copy1, chan->buffer, to_copy2);
    rptr = to_copy2;
  } else {
    rptr += to_copy1;

    if (rptr == chan->buffer_size)
      rptr = 0;
  }

  cpu_mem_write_32((uintptr_t)&chan->read_ptr, rptr);

  return to_copy;
}

void rtt_init(void)
{
  rtt_control.tx_channel_count = CONFIG_DRIVER_RTT_TX_CHANNEL_COUNT;
  rtt_control.rx_channel_count = CONFIG_DRIVER_RTT_RX_CHANNEL_COUNT;
  memset(&rtt_control.channel, 0,
         sizeof(struct rtt_channel_s)
         * (rtt_control.tx_channel_count
            + rtt_control.rx_channel_count));
  memcpy(rtt_control.id, "SEGGER R", 8);
  memcpy(rtt_control.id + 8, "TT", 3);
}

struct rtt_channel_s *rtt_channel_init(uint8_t id,
                                       const char *name,
                                       uint8_t *buf, uint32_t len,
                                       uint32_t flags)
{
  struct rtt_channel_s *chan = &rtt_control.channel[id];

  chan->name = name;
  chan->buffer = buf;
  chan->buffer_size = len;
  chan->read_ptr = chan->write_ptr = 0;
  chan->flags = flags;

  return chan;
}

void rtt_channel_cleanup(struct rtt_channel_s *chan)
{
  memset(chan, 0, sizeof(*chan));
}

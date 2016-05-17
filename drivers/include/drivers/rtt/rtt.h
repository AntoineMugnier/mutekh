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

#ifndef SEGGER_RTT_H_
#define SEGGER_RTT_H_

#include <hexo/types.h>
#include <hexo/iospace.h>

struct rtt_channel_s {
  const char *name;
  uint8_t *buffer;
  uint32_t buffer_size;
  uint32_t write_ptr;
  uint32_t read_ptr;
  uint32_t flags;
};

ALWAYS_INLINE
bool_t rtt_channel_has_data(const struct rtt_channel_s *chan)
{
  return cpu_mem_read_32((uintptr_t)&chan->write_ptr) != cpu_mem_read_32((uintptr_t)&chan->read_ptr);
}

/**
   @this reads as much data as possible from the channel.  Function
   returns as soon as possible.

   @param chan Channel
   @param buf Buffer to read into
   @param len Buffer size to transfer
   @returns actually transferred buffer length
 */
uint32_t rtt_channel_read(
  struct rtt_channel_s *chan,
  uint8_t *buf, uint32_t len);

/**
   @this writes as much data as possible to the channel.  Function
   returns as soon as possible.

   @param chan Channel
   @param buf Buffer to write from
   @param len Buffer size to transfer
   @returns actually transferred buffer length
 */
uint32_t rtt_channel_write(
  struct rtt_channel_s *chan,
  const uint8_t *buf, uint32_t len);

/**
   @this ensures all data written to channel is consumed by peer.

   @param chan Channel
 */
void rtt_channel_flush(
  struct rtt_channel_s *chan);

/**
   @this initializes a channel

   @param id Channel ID
   @param name Channel name. String is not copied, only referenced
   @param buf Buffer to use
   @param len Buffer size
   @param flags Buffer additional flags

   @param chan Channel descriptor
 */
struct rtt_channel_s *rtt_channel_init(uint8_t id,
                                       const char *name,
                                       uint8_t *buf, uint32_t len,
                                       uint32_t flags);

/**
   @this releases a channel

   All associated resources are cleared.

   @param channel Channel
 */
void rtt_channel_cleanup(struct rtt_channel_s *channel);

enum rtt_channel_mode_e {
  RTT_CHANNEL_MODE_SKIP = 0,
  RTT_CHANNEL_MODE_TRIM = 1,
  RTT_CHANNEL_MODE_BLOCKING = 2,
  RTT_CHANNEL_MODE_MASK = 3,
};

#define RTT_CHANNEL_TX_ID(x) (x)
#define RTT_CHANNEL_RX_ID(x) ((CONFIG_DRIVER_RTT_TX_CHANNEL_COUNT) + (x))

#endif

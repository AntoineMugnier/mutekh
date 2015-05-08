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

struct rtt_ringbuffer_s {
  const char *name;
  uint8_t *buffer;
  uint32_t buffer_size;
  uint32_t write_ptr;
  uint32_t read_ptr;
  uint32_t flags;
};

struct rtt_s {
  char id[16];
  uint32_t tx_buffer_count;
  uint32_t rx_buffer_count;
  struct rtt_ringbuffer_s buffer[0];
};

/**
   @this reads as much data as possible from the ringbuffer.  Function
   returns as soon as possible.

   @param ring Ring buffer
   @param buf Buffer to read into
   @param len Buffer size to transfer
   @returns actually transferred buffer length
 */
uint32_t rtt_ringbuffer_read(
  struct rtt_ringbuffer_s *ring,
  uint8_t *buf, uint32_t len);

/**
   @this writes as much data as possible to the ringbuffer.  Function
   returns as soon as possible.

   @param ring Ring buffer
   @param buf Buffer to write from
   @param len Buffer size to transfer
   @returns actually transferred buffer length
 */
uint32_t rtt_ringbuffer_write(
  struct rtt_ringbuffer_s *ring,
  const uint8_t *buf, uint32_t len);

/**
   @this initializes a ring buffer

   @param ring Ring buffer to initialize
   @param name Ring buffer name. String is not copied, only referenced
   @param buf Buffer to use
   @param len Buffer size
   @param flags Buffer additional flags
 */
void rtt_ringbuffer_init(
  struct rtt_ringbuffer_s *ring,
  const char *name,
  uint8_t *buf, uint32_t len,
  uint32_t flags);

#define RTT_RINGBUFFER_MODE_TRIM 1
#define RTT_RINGBUFFER_MODE_BLOCKING 2

/**
   @this is a static initializer for a ring buffer

   @param _name Ring buffer name
   @param _buf Buffer to use
   @param _flags Buffer additional flags
 */
#define RTT_RINGBUFFER_INITIALIZER(_name, _buf, _flags)    \
  {                                                        \
    .name = (_name),                                       \
    .buffer = (_buf),                                      \
    .buffer_size = sizeof(_buf),                           \
    .read_ptr = 0,                                         \
    .write_ptr = 0,                                        \
    .flags = (_flags),                                     \
  }

/**
   @this is a static initializer for a RTT control block

   @param _id Control block identifier
   @param _tx TX buffer count
   @param _rx RX buffer count
   @param ... Buffer initializers
 */
#define RTT_INITIALIZER(_id, _tx, _rx)                     \
  {                                                        \
    .id = (_id),                                           \
    .tx_buffer_count = (_tx),                              \
    .rx_buffer_count = (_rx),                              \
  }

#endif

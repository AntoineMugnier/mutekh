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
#include <hexo/iospace.h>
#include <stdlib.h>
#include <string.h>

#include "segger-rtt.h"

size_t rtt_ringbuffer_write(
  struct rtt_ringbuffer_s *ring,
  const uint8_t *buf, size_t len)
{
  size_t available;
  size_t rptr, wptr;

  wptr = cpu_mem_read_32((uintptr_t)&ring->write_ptr);
  rptr = cpu_mem_read_32((uintptr_t)&ring->read_ptr);

  if (wptr <= rptr)
    available = rptr - wptr - 1;
  else
    available = ring->buffer_size - 1 - wptr + rptr;

  if (!available || !len)
    return 0;

  size_t to_copy = __MIN(available, len);
  size_t to_copy1 = __MIN(ring->buffer_size - wptr, to_copy);
  size_t to_copy2 = to_copy - to_copy1;

  memcpy(ring->buffer + wptr, buf, to_copy1);

  if (to_copy2) {
    memcpy(ring->buffer, buf + to_copy1, to_copy2);
    wptr = to_copy2;
  } else {
    wptr += to_copy1;

    if (wptr == ring->buffer_size)
      wptr = 0;
  }

  cpu_mem_write_32((uintptr_t)&ring->write_ptr, wptr);

  return to_copy;
}

size_t rtt_ringbuffer_read(
  struct rtt_ringbuffer_s *ring,
  uint8_t *buf, size_t len)
{
  size_t available;
  size_t rptr, wptr;

  wptr = cpu_mem_read_32((uintptr_t)&ring->write_ptr);
  rptr = cpu_mem_read_32((uintptr_t)&ring->read_ptr);

  if (rptr <= wptr)
    available = wptr - rptr;
  else
    available = ring->buffer_size - rptr + wptr;

  if (!available || !len)
    return 0;

  size_t to_copy = __MIN(available, len);
  size_t to_copy1 = __MIN(ring->buffer_size - rptr, to_copy);
  size_t to_copy2 = to_copy - to_copy1;

  memcpy(buf, ring->buffer + rptr, to_copy1);

  if (to_copy2) {
    memcpy(buf + to_copy1, ring->buffer, to_copy2);
    rptr = to_copy2;
  } else {
    rptr += to_copy1;

    if (rptr == ring->buffer_size)
      rptr = 0;
  }

  cpu_mem_write_32((uintptr_t)&ring->read_ptr, rptr);

  return to_copy;
}

void rtt_ringbuffer_init(
  struct rtt_ringbuffer_s *ring,
  const char *name,
  uint8_t *buf, size_t len,
  uint32_t flags)
{
  ring->name = name;
  ring->buffer = buf;
  ring->buffer_size = len;
  ring->read_ptr = ring->write_ptr = 0;
  ring->flags = flags;
}

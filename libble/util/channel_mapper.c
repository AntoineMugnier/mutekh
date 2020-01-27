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

#define LOGK_MODULE_ID "cham"

#include <string.h>

#include <mutek/printk.h>

#include <hexo/types.h>
#include <ble/util/channel_mapper.h>

static
void channel_mapping_expand(uint8_t *channel_remap, uint64_t channel_map)
{
  uint8_t enabled_count = 0;
  uint8_t remap[37];
  uint64_t tmp = channel_map;

  for (uint8_t chan = 0; chan < 37; ++chan) {
    if (tmp & 1)
      remap[enabled_count++] = chan;

    tmp >>= 1;
  }

  tmp = channel_map;

  for (uint8_t chan = 0; chan < 37; ++chan) {
    if (tmp & 1)
      channel_remap[chan] = chan;
    else
      channel_remap[chan] = remap[chan % enabled_count];

    tmp >>= 1;
  }
}

static uint8_t mod37(uint32_t x)
{
  uint32_t d = 37 << 10;

  while (d >= 37) {
    if (x >= d)
      x -= d;

    d >>= 1;
  }

  return x;
}

error_t ble_channel_mapper_init(struct ble_channel_mapper_s *mapper,
                             uint64_t chan_map, uint8_t hop)
{
  chan_map &= (1ULL << 37) - 1;

  if (__builtin_popcountll(chan_map) < 2)
    return -EINVAL;

  memset(mapper, 0, sizeof(*mapper));
  channel_mapping_expand(mapper->mapped_channel, chan_map);
  mapper->hop = hop;
  mapper->last_unmapped_channel = hop;
  mapper->last_event = 0;

  return 0;
}

void ble_channel_mapper_event_set(struct ble_channel_mapper_s *mapper,
                                  uint16_t event)
{
  uint16_t delta = event - mapper->last_event;
  uint32_t channel = mapper->last_unmapped_channel + mapper->hop * delta;

  mapper->last_event = event;
  mapper->last_unmapped_channel = mod37(channel);

  if (mapper->update_pending) {
    int16_t after_update = event - mapper->update_instant;

    if (after_update >= 0) {
      mapper->update_pending = 0;
      memcpy(mapper->mapped_channel, mapper->pending_mapped_channel, 37);

      logk_trace("Channel map updated: %P\n", mapper->mapped_channel, 37);
    }
  }
}

uint8_t ble_channel_mapper_chan_get(struct ble_channel_mapper_s *mapper,
                                    uint16_t event)
{
  uint16_t delta = event - mapper->last_event;
  uint8_t unmapped_channel = mod37(mapper->last_unmapped_channel + delta * mapper->hop);

  if (mapper->update_pending) {
    int16_t after_update = event - mapper->update_instant;

    if (after_update >= 0)
      return mapper->pending_mapped_channel[unmapped_channel];
  }

  return mapper->mapped_channel[unmapped_channel];
}

error_t ble_channel_mapper_update_push(struct ble_channel_mapper_s *mapper,
                                       uint16_t instant,
                                       uint64_t chan_map)
{
  int16_t after_update = instant - mapper->last_event;
  if (after_update < 0)
    return -ETIMEDOUT;

  chan_map &= (1ULL << 37) - 1;

  if (__builtin_popcountll(chan_map) < 2)
    return -EINVAL;

  logk_trace("Channel map update at %d: %10llx\n", instant, chan_map);

  mapper->update_pending = 1;
  mapper->update_instant = instant;
  channel_mapping_expand(mapper->pending_mapped_channel, chan_map);

  return 0;
}


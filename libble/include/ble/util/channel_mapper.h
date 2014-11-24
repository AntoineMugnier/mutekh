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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2015
*/

#ifndef BLE_UTIL_CHANNEL_H_
#define BLE_UTIL_CHANNEL_H_

/**
   @file
   @module {Bluetooth Low Energy library}
   @short BLE connection channel mapper utility

   @this contains all declarations for the @ref {ble_channel_mapper_s}
   {Channel Mapper utility} object.
*/

#include <hexo/error.h>

/**
   This utility object tracks channel mapping of a running data
   connection.  It does not decides on channel mapping, but simply
   keeps track of which channel to set connection event on depending
   on current channel map, future channel map, switch instant, and
   last connection channel.

   This can be used either by master of slave side of a connection.
*/
struct ble_channel_mapper_s
{
  uint16_t update_instant;
  uint16_t last_event;
  uint8_t mapped_channel[37];
  uint8_t pending_mapped_channel[37];
  uint8_t last_unmapped_channel;
  uint8_t hop;
  bool_t update_pending : 1;
};

/**
   @this initiailzes a @ref {ble_channel_mapper_s} {Channel Mapper}
   with an initial channel map and hop.
 */
error_t ble_channel_mapper_init(struct ble_channel_mapper_s *mapper,
                                uint64_t chan_map, uint8_t hop);

ALWAYS_INLINE
void ble_channel_mapper_cleanup(struct ble_channel_mapper_s *mapper)
{}

/**
   @this sets last event as @tt event.  This allows to commit next
   channel map if one is pending and instant just got passed.
 */
void ble_channel_mapper_event_set(struct ble_channel_mapper_s *mapper,
                                  uint16_t event);

/**
   @this retrieves channel to run on for given event.  Event may not
   be before last event number passed to last call to @ref
   ble_channel_mapper_event_set.

   Returned channel is the Data Channel Index (See 6.B.1.4.1).
 */
uint8_t ble_channel_mapper_chan_get(struct ble_channel_mapper_s *mapper,
                                    uint16_t event);

/**
   @this pushes a channel map update as pending until @tt instant.  If
   instant passed already, this function may return an error.
 */
error_t ble_channel_mapper_update_push(struct ble_channel_mapper_s *mapper,
                                       uint16_t instant,
                                       uint64_t chan_map);

#endif

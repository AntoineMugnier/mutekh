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

#ifndef BLE_NET_LINK_H_
#define BLE_NET_LINK_H_

/**
   @file
   @module{BLE library}
   @short Low level connection layer (either master or slave)

   @section {Description}

   This handles a connection on low level.  In particular, this layer
   handles cryptography if enabled.  It also demuxes between L2CAP and
   LLCP.

   @end section
*/

#include <hexo/types.h>
#include <hexo/decls.h>

#include <net/layer.h>

#include <ble/protocol/advertise.h>
#include <device/class/crypto.h>

struct buffer_s;
struct ble_link_handler_s;
struct ble_peer_s;

enum ble_link_child_type_e
{
  BLE_LINK_CHILD_LLCP,
  BLE_LINK_CHILD_L2CAP,
};

struct ble_link_param_s
{
  bool_t is_master;
#if defined(CONFIG_BLE_CRYPTO)
  struct device_crypto_s *crypto;
#endif
};

#endif

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

#ifndef BLE_NET_LAYER_H_
#define BLE_NET_LAYER_H_

/**
   @file
   @module {Bluetooth Low Energy library}
   @short Hardware-dependent BLE network layer IDs
*/

/**
   @this defines network layer IDs for BLE layers.

   These IDs should be passed to @ref {device_net_s::f_layer_create}.
 */
enum ble_net_layer_id_e {
  BLE_NET_LAYER_ADV = CONFIG_BLE_NET_LAYER_FIRST,
  BLE_NET_LAYER_SCANNER,
  BLE_NET_LAYER_MASTER,
  BLE_NET_LAYER_SLAVE,
  BLE_NET_LAYER_SNIFFER,
  BLE_NET_LAYER_DTM_TX,
};

#endif

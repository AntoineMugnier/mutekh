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

#ifndef BLE_ATT_H_
#define BLE_ATT_H_

/**
   @file
   @module{BLE library}
   @short Attribute protocol network layer

   @section {Description}

   This implements the Attribute protocol for LE links.  This layer
   actually is a demo for typical Att handling.  Usual ATT
   implementation using a GATT DB backend is in @ref {gatt.h}.

   @end section
*/

#include <hexo/types.h>
#include <hexo/decls.h>
#include <net/task.h>

struct net_scheduler_s;
struct net_layer_s;

#endif

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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2016
*/

#ifndef PSOC4_VARIANT_COMMON_H
#define PSOC4_VARIANT_COMMON_H

#include <arch/psoc4/hsiom_port.h>

/*
  Pin mux
 */

#define PSOC4_IO(port_, pin_) (((port_)<<3) | (pin_))
#define PSOC4_IO_PORT(io_) ((io_) >> 3)
#define PSOC4_IO_PIN(io_) ((io_) & 7)

#endif

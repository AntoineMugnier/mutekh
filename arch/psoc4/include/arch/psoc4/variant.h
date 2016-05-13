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

#if defined(CONFIG_ARCH_PSOC4_PROCBLE)
# include <arch/psoc4/variant/procble.h>
#elif defined(CONFIG_ARCH_PSOC4_PSOC)
# include <arch/psoc4/variant/psoc.h>
#elif defined(CONFIG_ARCH_PSOC4_BLE)
# include <arch/psoc4/variant/psoc4_ble.h>
#else
# error Unsupported architecture variant
#endif

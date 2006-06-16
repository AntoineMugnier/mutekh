/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#ifndef __ENUM_PCI_PRIVATE_H_
#define __ENUM_PCI_PRIVATE_H_

#include <mutek/types.h>
#include <mutek/lock.h>
#include <mutek/iospace.h>

#ifdef __ARCH__ibmpc__

# define PCI_IBMPC_CONF_ADDRIO	0x0cf8
# define PCI_IBMPC_CONF_DATAIO	0x0cfc

static inline uint32_t
pci_confreg_read(uint_fast8_t addr)
{
  cpu_io_write_32(PCI_IBMPC_CONF_ADDRIO, addr);

  return cpu_io_read_32(PCI_IBMPC_CONF_DATAIO);
}

static inline void
pci_confreg_write(uint_fast8_t addr, uint32_t data)
{
  cpu_io_write_32(PCI_IBMPC_CONF_ADDRIO, addr);
  cpu_io_write_32(PCI_IBMPC_CONF_DATAIO, data);
}

#else
# error support missing for PCI enumerator device
#endif

struct enum_pci_context_s
{
  lock_t			lock;
};

#endif


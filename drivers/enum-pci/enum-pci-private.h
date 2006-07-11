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

#include <hexo/types.h>
#include <hexo/lock.h>
#include <hexo/iospace.h>

#define PCI_CONFREG_VENDOR		0x00
#define PCI_CONFREG_DEVID		0x02
#define PCI_CONFREG_CMD			0x04
#define PCI_CONFREG_STATUS		0x06
#define PCI_CONFREG_REVID		0x08
#define PCI_CONFREG_CLASS		0x09

#define PCI_CONFREG_CLINE		0x0c
#define PCI_CONFREG_LATENCY		0x0d
#define PCI_CONFREG_HTYPE		0x0e
# define PCI_CONFREG_HTYPE_MULTI	0x80 /* multi-function device */
#define PCI_CONFREG_BIST		0x0f

#define PCI_CONF_MAXBUS			8
#define PCI_CONF_MAXDEVICE		32
#define PCI_CONF_MAXFCN			8
#define PCI_CONF_MAXREG			256

/************************************************************************/

#ifdef __ARCH__ibmpc__

# define PCI_IBMPC_CONF_ADDRIO	0x0cf8
# define PCI_IBMPC_CONF_DATAIO	0x0cfc

static inline uint32_t
pci_confreg_read(uint_fast8_t bus, uint_fast8_t dv,
		 uint_fast8_t fn, uint_fast8_t reg)
{
  uint32_t	addr = 0x80000000 | (bus << 16) | (dv << 11) | (fn << 8) | (reg & 0xfc);

  cpu_io_write_32(PCI_IBMPC_CONF_ADDRIO, addr);

  return cpu_io_read_32(PCI_IBMPC_CONF_DATAIO) >> (8 * (reg & 3));
}

static inline void
pci_confreg_write(uint_fast8_t bus, uint_fast8_t dv, uint_fast8_t fn,
		  uint_fast8_t reg, uint32_t data)
{
  uint32_t	addr = 0x80000000 | (bus << 16) | (dv << 11) | (fn << 8) | reg;

  cpu_io_write_32(PCI_IBMPC_CONF_ADDRIO, addr);
  cpu_io_write_32(PCI_IBMPC_CONF_DATAIO, data);
}

#else
# error support missing for PCI enumerator device
#endif

/************************************************************************/

struct enum_pci_context_s
{
  lock_t			lock;
};

struct enum_pv_pci_s
{
  uint16_t		vendor;
  uint16_t		devid;
  uint32_t		class;
};

#endif


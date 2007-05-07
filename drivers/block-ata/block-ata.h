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

#ifndef DRIVER_BLOCK_ATA_H_
#define DRIVER_BLOCK_ATA_H_

#include <hexo/device/block.h>
#include <hexo/device.h>

DEV_IRQ(block_ata_irq);
DEV_INIT(block_ata_init);
DEV_CLEANUP(block_ata_cleanup);
DEVBLOCK_READ(block_ata_read);
DEVBLOCK_WRITE(block_ata_write);

#endif


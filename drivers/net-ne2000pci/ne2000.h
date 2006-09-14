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

    Copyright Matthieu Bucchianeri <matthieu.bucchianeri@epita.fr> (c) 2006

*/

#ifndef _NE2000_H
#define _NE2000_H

/* command register bits */
#define NE2000_STP	(1 << 0)
#define NE2000_STA	(1 << 1)
#define NE2000_TXP	(1 << 2)
#define NE2000_RD0	(1 << 3)
#define NE2000_RD1	(1 << 4)
#define NE2000_RD2	(1 << 5)
#define NE2000_PS0	(1 << 6)
#define NE2000_PS1	(1 << 7)

/* shortcuts for DMA transfers */
#define NE2000_DMA_RD	NE2000_RD0
#define NE2000_DMA_WR	NE2000_RD1
#define NE2000_DMA_SEND	(NE2000_RD1 | NE2000_RD0)

/* shortcuts for page select */
#define NE2000_P0	0
#define NE2000_P1	NE2000_PS0
#define NE2000_P2	NE2000_PS1

/* interrupt register bits */
#define NE2000_PRX	(1 << 0)
#define NE2000_PTX	(1 << 1)
#define NE2000_TXE	(1 << 3)
#define NE2000_OVW	(1 << 4)
#define NE2000_RDC	(1 << 6)

/* register addresses */
#define NE2000_TPSR	0x4
#define NE2000_TBCR0	0x5
#define NE2000_TBCR1	0x6

#endif

/*
 *  GRUB  --  GRand Unified Bootloader
 *  Copyright (C) 2001,2002  Free Software Foundation, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/**************************************************************************

Author: Martin Renters
  Date: May/94

 This code is based heavily on David Greenman's if_ed.c driver

 Copyright (C) 1993-1994, David Greenman, Martin Renters.
  This software may be used, modified, copied, distributed, and sold, in
  both source and binary form provided that the above copyright and these
  terms are retained. Under no circumstances are the authors responsible for
  the proper functioning of this software, nor do the authors assume any
  responsibility for damages incurred with its use.

**************************************************************************/

#include <hexo/types.h>
#include <hexo/device.h>
#include <hexo/iospace.h>
#include <hexo/alloc.h>
#include <hexo/lock.h>
#include <hexo/interrupt.h>

#include "net-ns8390.h"

#include "net-ns8390-private.h"

#include "ns8390.h"

/*
 * programmed I/O reading.
 */

void	net_ns8390_pio_read(struct net_ns8390_context_s		*pv,
			    uint_fast16_t			src,
			    uint8_t				*dst,
			    size_t				size)
{
  if (pv->mode_16bits)
    {
      ++size;
      size &= ~1;
    }

  /* configure the transfer */
  cpu_io_write_8(pv->base + D8390_P0_COMMAND,
		 D8390_COMMAND_RD2 | D8390_COMMAND_STA);
  cpu_io_write_8(pv->base + D8390_P0_RBCR0, size);
  cpu_io_write_8(pv->base + D8390_P0_RBCR1, size >> 8);
  cpu_io_write_8(pv->base + D8390_P0_RSAR0, src);
  cpu_io_write_8(pv->base + D8390_P0_RSAR1, src >> 8);
  cpu_io_write_8(pv->base + D8390_P0_COMMAND,
		 D8390_COMMAND_RD0 | D8390_COMMAND_STA);

  if (pv->mode_16bits)
    size >>= 1;

  /* read the data into the buffer */
  while (size--)
    {
      if (pv->mode_16bits)
	{
	  (*(uint16_t*)dst) = cpu_io_read_16(pv->asic + NE_DATA);
	  dst += 2;
	}
      else
	*dst++ = cpu_io_read_8(pv->asic + NE_DATA);
    }
}

/*
 * programmed I/O writing.
 */

void	net_ns8390_pio_write(struct net_ns8390_context_s	*pv,
			     uint8_t				*src,
			     uint_fast16_t			dst,
			     size_t				size)
{
  if (pv->mode_16bits)
    {
      ++size;
      size &= ~1;
    }

  /* configure the transfer */
  cpu_io_write_8(pv->base + D8390_P0_COMMAND,
		 D8390_COMMAND_RD2 | D8390_COMMAND_STA);
  cpu_io_write_8(pv->base + D8390_P0_ISR, D8390_ISR_RDC);
  cpu_io_write_8(pv->base + D8390_P0_RBCR0, size);
  cpu_io_write_8(pv->base + D8390_P0_RBCR1, size >> 8);
  cpu_io_write_8(pv->base + D8390_P0_RSAR0, dst);
  cpu_io_write_8(pv->base + D8390_P0_RSAR1, dst >> 8);
  cpu_io_write_8(pv->base + D8390_P0_COMMAND,
		 D8390_COMMAND_RD1 | D8390_COMMAND_STA);

  if (pv->mode_16bits)
    size >>= 1;

  /* write the buffer */
  while (size--)
    {
      if (pv->mode_16bits)
	{
	  cpu_io_write_16(pv->asic + NE_DATA, (*(uint16_t*)src));
	  src += 2;
	}
      else
	cpu_io_write_8(pv->asic + NE_DATA, *src++);
    }

  /* wait for the transfer to be completed */
  while ((cpu_io_read_8(pv->base + D8390_P0_ISR) & D8390_ISR_RDC) !=
	 D8390_ISR_RDC)
    ;
}

/*
 * reset the device.
 */

void		net_ns8390_reset(struct net_ns8390_context_s	*pv)
{
  uint_fast8_t	i;

  /* setup transmit/receive buffer */
  cpu_io_write_8(pv->base + D8390_P0_COMMAND,
		 D8390_COMMAND_PS0 | D8390_COMMAND_RD2 | D8390_COMMAND_STP);
  if (pv->mode_16bits)
    cpu_io_write_8(pv->base + D8390_P0_DCR, 0x49);
  else
    cpu_io_write_8(pv->base + D8390_P0_DCR, 0x48);
  cpu_io_write_8(pv->base + D8390_P0_RBCR0, 0);
  cpu_io_write_8(pv->base + D8390_P0_RBCR1, 0);
  cpu_io_write_8(pv->base + D8390_P0_RCR, 0x20);
  cpu_io_write_8(pv->base + D8390_P0_TCR, 0x2);
  cpu_io_write_8(pv->base + D8390_P0_TPSR, pv->tx_start);
  cpu_io_write_8(pv->base + D8390_P0_PSTART, pv->rx_start);
  cpu_io_write_8(pv->base + D8390_P0_PSTOP, pv->mem);
  cpu_io_write_8(pv->base + D8390_P0_BOUND, pv->mem - 1);
  cpu_io_write_8(pv->base + D8390_P0_ISR, 0xff);
  cpu_io_write_8(pv->base + D8390_P0_IMR, 0);

  /* setup MAC address */
  cpu_io_write_8(pv->base + D8390_P0_COMMAND,
		 D8390_COMMAND_PS1 | D8390_COMMAND_RD2 | D8390_COMMAND_STP);
  for (i = 0; i < ETH_ALEN; i++)
    cpu_io_write_8(pv->base + D8390_P1_PAR0 + i, pv->mac[i]);
  for (i = 0; i < ETH_ALEN; i++)
    cpu_io_write_8(pv->base + D8390_P1_MAR0 + i, 0xff);

  cpu_io_write_8(pv->base + D8390_P1_CURR, pv->rx_start);
  cpu_io_write_8(pv->base + D8390_P0_COMMAND,
		 D8390_COMMAND_PS0 | D8390_COMMAND_RD2 | D8390_COMMAND_STA);
  cpu_io_write_8(pv->base + D8390_P0_ISR, 0xff);
  cpu_io_write_8(pv->base + D8390_P0_TCR, 0x0);
  cpu_io_write_8(pv->base + D8390_P0_RCR, 0x4);
}

/*
 * probe the device.
 */

error_t			net_ns8390_probe(struct net_ns8390_context_s	*pv,
					 uint_fast16_t			base)
{
  uint_fast8_t		i;
  static char		ref_buff[] = "TEST";
  uint8_t		buff[32];
  uint8_t		rom[32];

  pv->mode_16bits = 1;
  pv->base = base;
  pv->asic = base + NE_ASIC_OFFSET;
  pv->mem = MEM_16384;
  pv->tx_start = 32;
  pv->rx_start = 32 + D8390_TXBUF_SIZE;

  /* reset the controller */
  cpu_io_write_8(pv->asic + NE_RESET, cpu_io_read_8(pv->asic + NE_RESET));

  cpu_io_read_8(0x84);

  /* configure the device for a R/W test */
  cpu_io_write_8(base + D8390_P0_COMMAND,
		 D8390_COMMAND_STP | D8390_COMMAND_RD2);

  cpu_io_write_8(base + D8390_P0_RCR, D8390_RCR_MON);
  cpu_io_write_8(base + D8390_P0_DCR, D8390_DCR_FT1 | D8390_DCR_LS);
  cpu_io_write_8(base + D8390_P0_PSTART, MEM_8192);
  cpu_io_write_8(base + D8390_P0_PSTOP, MEM_16384);

  memset(buff, 0, sizeof (buff));
  net_ns8390_pio_write(pv, (uint8_t*)ref_buff, 8192, sizeof (ref_buff));
  net_ns8390_pio_read(pv, 8192, buff, sizeof (ref_buff));

  if (!memcmp(buff, ref_buff, sizeof (ref_buff)))
    goto ok; /* test succeeded */

  /* try with more memory */
  pv->mode_16bits = 1;
  pv->mem = MEM_32768;
  pv->tx_start = 64;
  pv->rx_start = 64 + D8390_TXBUF_SIZE;

  cpu_io_write_8(base + D8390_P0_DCR,
		 D8390_DCR_FT1 | D8390_DCR_LS | D8390_DCR_WTS);
  cpu_io_write_8(base + D8390_P0_PSTART, MEM_16384);
  cpu_io_write_8(base + D8390_P0_PSTOP, MEM_32768);

  memset(buff, 0, sizeof (buff));
  net_ns8390_pio_write(pv, (uint8_t*)ref_buff, 16384, sizeof (ref_buff));
  net_ns8390_pio_read(pv, 16384, buff, sizeof (ref_buff));

  if (!memcmp(buff, ref_buff, sizeof (ref_buff)))
    goto ok; /* test suceeded */

  return -1; /* no device found */

 ok:
  printf("ns8390: device I/O base 0x%x\n", base);

  /* read MAC address */
  net_ns8390_pio_read(pv, 0, rom, sizeof (rom));

  for (i = 0; i < ETH_ALEN; i++)
    pv->mac[i] = rom[i << 1];

  printf("  MAC address: %P\n", pv->mac, ETH_ALEN);

  return 0;
}

/*
 * device read operation
 */

size_t			net_ns8390_read(struct net_ns8390_context_s	*pv,
					uint8_t				**data)
{
  uint_fast8_t		current;
  uint_fast8_t		next;
  uint_fast16_t		packet;
  uint_fast16_t		fragment;
  uint_fast16_t		total;
  uint_fast16_t		length;
  struct net_ns8380_header_s	header;

  /* get the current state */
  if (!(cpu_io_read_8(pv->base + D8390_P0_RSR) & D8390_RSTAT_PRX))
    return 0;
  /* identifies current and next packets */
  next = cpu_io_read_8(pv->base + D8390_P0_BOUND) + 1;
  if (next >= pv->mem)
    next = pv->rx_start;
  cpu_io_write_8(pv->base + D8390_P0_COMMAND, D8390_COMMAND_PS1);
  current = cpu_io_read_8(pv->base + D8390_P1_CURR);
  cpu_io_write_8(pv->base + D8390_P0_COMMAND, D8390_COMMAND_PS0);
  if (current >= pv->mem)
    current = pv->rx_start;

  if (current == next)
    return 0;

  /* fetch the packet header */
  packet = next << 8;
  net_ns8390_pio_read(pv, packet, (uint8_t*)&header, 4);
  packet += sizeof (struct net_ns8380_header_s);

  length = header.size - 4;
  total = length;

  /* allocate the packet */
  *data = mem_alloc(total, MEM_SCOPE_THREAD);

  if (!(header.status & D8390_RSTAT_PRX) ||
      length < ETH_ZLEN || length > ETH_FRAME_LEN)
    {
      printf("ns8390: bad packet\n");
      return 0;
    }

  /* the packet may be fragmented in two parts */
  fragment = (pv->mem << 8) - packet;

  /* fetch the first part (if packet splitted) */
  if (0 && length > fragment) /* XXX does not work */
    {
      printf("ns8390: fetching %d-%d %d -> %p\n", 0, fragment, packet, *data);
      net_ns8390_pio_read(pv, packet, *data, fragment);
      packet = pv->rx_start << 8;
      *data += fragment;
      length -= fragment;
    }
  /* fetch the second part (the entire packet if no split) */
  printf("ns8390: fetching -%d %d -> %p\n", length, packet, *data);
  net_ns8390_pio_read(pv, packet, *data, length);

  next = header.next;
  if (next == pv->rx_start)
    next = pv->mem;
  cpu_io_write_8(pv->base + D8390_P0_BOUND, next - 1);
  return total;
}

/*
 * device write operation
 */

void			net_ns8390_write(struct net_ns8390_context_s	*pv,
					 uint8_t			*data,
					 size_t				size)
{
  size_t		len;

  /* copy the packet in the network card */
  net_ns8390_pio_write(pv, data, pv->tx_start << 8, size);
  len = size;
  /* adjust the packet size if necessary */
  if (len < ETH_ZLEN)
    len = ETH_ZLEN;
  /* setup the controller to send the packet */
  cpu_io_write_8(pv->base + D8390_P0_COMMAND,
		 D8390_COMMAND_PS0 | D8390_COMMAND_RD2 | D8390_COMMAND_STA);
  cpu_io_write_8(pv->base + D8390_P0_TPSR, pv->tx_start);
  cpu_io_write_8(pv->base + D8390_P0_TBCR0, len);
  cpu_io_write_8(pv->base + D8390_P0_TBCR1, len >> 8);
  cpu_io_write_8(pv->base + D8390_P0_COMMAND, D8390_COMMAND_PS0 |
		 D8390_COMMAND_TXP | D8390_COMMAND_RD2 | D8390_COMMAND_STA);
}

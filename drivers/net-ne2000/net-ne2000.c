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

#include <hexo/types.h>
#include <hexo/device.h>
#include <hexo/iospace.h>
#include <hexo/alloc.h>
#include <hexo/lock.h>
#include <hexo/interrupt.h>
#include <pthread.h>

#include <netinet/if.h>

#include "net-ne2000.h"

#include "net-ne2000-private.h"
#include "ne2000.h"

#ifndef CONFIG_STATIC_DRIVERS

/*
 * PCI identifiers of compatible cards.
 */

static const struct devenum_ident_s	net_ne2000_ids[] =
  {
    { .vendor = 0x10ec, .device = 0x8029 },	/* Realtek 8029 */
    { .vendor = 0x1050, .device = 0x0940 },	/* Winbond 89C940 */
    { .vendor = 0x1050, .device = 0x5a5a },	/* Winbond 89C940F */
    { .vendor = 0x8c4a, .device = 0x1980 },	/* Winbond 89C940 (bad ROM) */
    { .vendor = 0x11f6, .device = 0x1401 },	/* Compex ReadyLink 2000 */
    { .vendor = 0x8e2e, .device = 0x3000 },	/* KTI ET32P2 */
    { .vendor = 0x4a14, .device = 0x5000 },	/* NetVin NV5000SC */
    { .vendor = 0x12c3, .device = 0x0058 },	/* HolTek HT80232 */
    { .vendor = 0x1106, .device = 0x0926 },	/* Via 86C926 */
    { .vendor = 0x10bd, .device = 0x0e34 },	/* SureCom NE34 */
    { 0 }
  };

/*
 * Driver operations vector.
 */

const struct driver_s	net_ne2000_drv =
{
  .id_table		= net_ne2000_ids,

  .f_init		= net_ne2000_init,
  .f_cleanup		= net_ne2000_cleanup,
  .f_irq		= net_ne2000_irq,
  .f.net = {
    .f_preparepkt	= net_ne2000_preparepkt,
    .f_sendpkt		= net_ne2000_sendpkt,
    .f_register_proto	= net_ne2000_register_proto,
  }
};
#endif

/*
 * update the command register.
 */

static inline void	ne2000_command(struct device_s	*dev,
				       uint_fast8_t	cmd)
{
  uint_fast16_t		addr = dev->addr[NET_NE2000_ADDR] + NE2000_CMD;

  cpu_io_write_8(addr, cpu_io_read_8(addr) | cmd);
}

/*
 * update the command register for DMA operations.
 */

static inline void	ne2000_dma(struct device_s	*dev,
				   uint_fast8_t		cmd)
{
  uint_fast16_t		addr = dev->addr[NET_NE2000_ADDR] + NE2000_CMD;

  cpu_io_write_8(addr, (cpu_io_read_8(addr) & ~NE2000_DMA_MASK) | cmd);
}

/*
 * update the command register for page selection.
 */

static inline void	ne2000_page(struct device_s	*dev,
				    uint_fast8_t	cmd)
{
  uint_fast16_t		addr = dev->addr[NET_NE2000_ADDR] + NE2000_CMD;

  cpu_io_write_8(addr, (cpu_io_read_8(addr) & ~NE2000_PG_MASK) | cmd);
}

/*
 * read from the device's memory.
 */

static void		ne2000_mem_read(struct device_s	*dev,
					uint_fast16_t	offs,
					void		*dst,
					uint_fast16_t	size)
{
  struct net_ne2000_context_s	*pv = dev->drv_pv;

  /* select register bank 0 */
  ne2000_page(dev, NE2000_P0);

  /* ensure DMA operations are reset */
  ne2000_dma(dev, NE2000_DMA_ABRT);
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_ISR, NE2000_RDC);

  /* setup size */
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_RBCR0, size);
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_RBCR1, size >> 8);

  /* setup position */
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_RSAR0, offs);
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_RSAR1, offs >> 8);

  /* start read operation */
  ne2000_dma(dev, NE2000_DMA_RD);

  /* copy the whole packet */
  if (pv->io_16)
    {
      uint16_t	*d = (uint16_t *)dst;

      size >>= 1;
      while (size--)
	*d++ = cpu_io_read_16(dev->addr[NET_NE2000_ADDR] + NE2000_DATA);
    }
  else
    {
      uint8_t	*d = dst;

      while (size--)
	*d++ = cpu_io_read_8(dev->addr[NET_NE2000_ADDR] + NE2000_DATA);
    }

  /* XXX better use insb/insw */
}

/*
 * init a remote DMA writing transfer.
 */

static void		ne2000_dma_init_write(struct device_s	*dev,
					      uint_fast16_t	offs,
					      uint_fast16_t	size)
{
  /* select register bank 0 */
  ne2000_page(dev, NE2000_P0);

  /* ensure DMA operations are reset */
  ne2000_dma(dev, NE2000_DMA_ABRT);
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_ISR, NE2000_RDC);

  /* setup size */
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_RBCR0, size);
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_RBCR1, size >> 8);

  /* setup position */
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_RSAR0, offs);
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_RSAR1, offs >> 8);

  /* start write operation */
  ne2000_dma(dev, NE2000_DMA_WR);
}

/*
 * output data for remote DMA.
 */

static void		ne2000_dma_do_write(struct device_s	*dev,
					 void			*src,
					 uint_fast16_t		size)
{
  struct net_ne2000_context_s	*pv = dev->drv_pv;

  /* copy the whole packet */
  if (pv->io_16)
    {
      uint16_t	*d = (uint16_t *)src;

      size >>= 1;
      while (size--)
	cpu_io_write_16(dev->addr[NET_NE2000_ADDR] + NE2000_DATA, *d++);
    }
  else
    {
      uint8_t	*d = src;

      while (size--)
	cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_DATA, *d++);
    }

  /* XXX better use outsb/outsw */
}

/*
 * write to the device's memory.
 */

static inline void	ne2000_mem_write(struct device_s	*dev,
					 uint_fast16_t		offs,
					 void			*src,
					 uint_fast16_t		size)
{
  ne2000_dma_init_write(dev, offs, size);

  ne2000_dma_do_write(dev, src, size);
}

/*
 * try reading and writing a stupid sentence to memory.
 */

static uint_fast8_t	ne2000_rw_test(struct device_s	*dev)
{
  struct net_ne2000_context_s	*pv = dev->drv_pv;
  uint_fast8_t			endian;
  uint_fast16_t			timeout = 2000;
  char				ref[] = "MutekH 42 NE2000 Driver";
  char				buf[sizeof (ref)];

  /* configure the device for the test */
  /* send a reset signal */
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_RESET,
		 cpu_io_read_8(dev->addr[NET_NE2000_ADDR] + NE2000_RESET));

  /* wait a moment */
  while (timeout)
    timeout--;
  timeout = 200000;

  /* stop completely the device */
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_CMD,
		 NE2000_P0 | NE2000_DMA_ABRT | NE2000_STP);

#ifdef CPU_ENDIAN_ISBIG
  endian = NE2000_BE;
#else
  endian = NE2000_LE;
#endif

  /* setup data configuration registers */
  if (pv->io_16)
    cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_DCR,
		   NE2000_8BITS | endian | NE2000_NORMAL | NE2000_FIFO4);
  else
    cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_DCR,
		   NE2000_16BITS | endian | NE2000_NORMAL | NE2000_FIFO4);
  /* setup transmit and receive in loopback */
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_RCR, NE2000_MONITOR);
  /* setup RX ring */
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_PSTART, pv->tx_buf);
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_PSTOP, pv->mem);

  /* start the device */
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_CMD,
		 NE2000_P0 | NE2000_DMA_ABRT | NE2000_STA);

  /* do the R/W test */
  ne2000_mem_write(dev, pv->tx_buf << 8, ref, sizeof (ref));
  while (timeout && !(cpu_io_read_8(dev->addr[NET_NE2000_ADDR] + NE2000_ISR) & NE2000_RDC))
    timeout--;
  memset(buf, 0, sizeof (ref));
  ne2000_mem_read(dev, pv->tx_buf << 8, buf, sizeof (ref));

  return !memcmp(ref, buf, sizeof (ref));
}

/*
 * probe device capabilities.
 */

static uint_fast8_t	ne2000_probe(struct device_s	*dev)
{
  struct net_ne2000_context_s	*pv = dev->drv_pv;
  uint8_t			buf[ETH_ALEN * 2];
  uint_fast8_t			i;

  /* try 8 bits mode with 16k */
  pv->io_16 = 0;
  pv->mem = NE2000_MEM_16K;
  pv->tx_buf = NE2000_MEM_8K;
  pv->rx_buf = NE2000_MEM_8K + NE2000_TX_BUFSZ;

  if (ne2000_rw_test(dev))
    goto ok;

  /* try 16 bits mode with 16k */
  pv->io_16 = 0;
  pv->mem = NE2000_MEM_32K;
  pv->tx_buf = NE2000_MEM_16K;
  pv->rx_buf = NE2000_MEM_16K + NE2000_TX_BUFSZ;

  if (ne2000_rw_test(dev))
    goto ok;

  /* try 8 bits mode with 32k */
  pv->io_16 = 1;
  pv->mem = NE2000_MEM_16K;
  pv->tx_buf = NE2000_MEM_8K;
  pv->rx_buf = NE2000_MEM_8K + NE2000_TX_BUFSZ;

  if (ne2000_rw_test(dev))
    goto ok;

  /* try 16 bits mode with 32k */
  pv->io_16 = 1;
  pv->mem = NE2000_MEM_32K;
  pv->tx_buf = NE2000_MEM_16K;
  pv->rx_buf = NE2000_MEM_16K + NE2000_TX_BUFSZ;

  if (ne2000_rw_test(dev))
    goto ok;

  /* all configuration failed */
  return 0;

 ok:
  /* everything ok */

  /* determine MAC address, the first 6 bytes/words of the PROM */
  ne2000_mem_read(dev, 0, buf, ETH_ALEN * 2);

  if (pv->io_16)
    for (i = 0; i < ETH_ALEN; i++)
      pv->mac[i] = buf[i << 1];
  else
    for (i = 0; i < ETH_ALEN; i++)
      pv->mac[i] = buf[i];

  return 1;
}

/*
 * init device. refer to the 8390 documentation for this sequence.
 */

static void		ne2000_init(struct device_s	*dev)
{
  struct net_ne2000_context_s	*pv = dev->drv_pv;
  uint_fast8_t			endian;
  uint_fast8_t			i;

  /* stop completely the device */
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_CMD,
		 NE2000_P0 | NE2000_DMA_ABRT | NE2000_STP);

#ifdef CPU_ENDIAN_ISBIG
  endian = NE2000_BE;
#else
  endian = NE2000_LE;
#endif

  /* setup data configuration registers */
  if (pv->io_16)
    cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_DCR,
		   NE2000_8BITS | endian | NE2000_NORMAL | NE2000_FIFO4);
  else
    cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_DCR,
		   NE2000_16BITS | endian | NE2000_NORMAL | NE2000_FIFO4);
  /* clear dma state */
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_RBCR0, 0);
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_RBCR1, 0);
  /* setup receive configuration register */
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_RCR,
		 NE2000_REJECT_ON_ERROR | NE2000_ACCEPT_BCAST |
		 NE2000_REJECT_MCAST);
  /* enter loopback mode */
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_TCR, 0x2);
  /* initialize TX and RX buffers */
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_TPSR, pv->tx_buf);
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_PSTART, pv->rx_buf);
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_PSTOP, pv->mem);
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_BOUND, pv->rx_buf);
  /* clear ISR */
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_ISR, 0xff);
  /* activate interrupts */
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_IMR,
		 NE2000_PRXE | NE2000_PTXE | NE2000_TXEE | NE2000_OVWE |
		 NE2000_RDCE);
  /* init MAC */
  ne2000_page(dev, NE2000_P1);
  for (i = 0; i < ETH_ALEN; i++)
    cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_PAR + i, pv->mac[i]);
  for (i = 0; i < ETH_ALEN; i++)
    cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_MAR + i, 0xff);
  /* init current receive buffer pointer */
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_CURR, pv->rx_buf + 1);
  /* bring the device up */
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_CMD,
		 NE2000_P0 | NE2000_DMA_ABRT | NE2000_STA);
  /* setup transmit configuration register */
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_TCR, NE2000_AUTOCRC);
  /* clear ISR */
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_ISR, 0xff);
}

/*
 * prepare sending a packet.
 */

static void	ne2000_send(struct device_s	*dev)
{
  struct net_ne2000_context_s	*pv = dev->drv_pv;
  struct net_header_s		*nethdr;
  uint_fast16_t			size;

  nethdr = &pv->current->header[0];

  /* copy in one time */
  size = (pv->io_16 ? ALIGN_VALUE(nethdr->size, 2) : nethdr->size);

  /* reset the tries counter */
  pv->send_tries = 0;

#if defined(CONFIG_NE2000_FRAGMENT) || defined(CONFIG_NETWORK_AUTOALIGN)
  uint_fast8_t	i;

  /* initialize writing */
  ne2000_dma_init_write(dev, pv->tx_buf << 8, size);

  for (i = 0; nethdr[i].data; i++)
    {
      uint_fast16_t	fragsz;

      printf("%u: data %p, size %u\n", i, nethdr[i].data, nethdr[i].size);
      printf("%u: data %p, size %u\n", i + 1, nethdr[i + 1].data, nethdr[i + 1].size);

      if (!nethdr[i + 1].data)
	fragsz = nethdr[i].size;
      else
	fragsz = nethdr[i].size - nethdr[i + 1].size;
      net_debug("chunk size %u\n", fragsz);
      /* write each chunk after the previous one */
      ne2000_dma_do_write(dev, nethdr[i].data, fragsz);
    }
#else
  ne2000_mem_write(dev, pv->tx_buf << 8, nethdr->data, size);
#endif

  /* the data being written, we wait for remote DMA to be completed (IRQ) */
}

/*
 * push a packet.
 */

static void	ne2000_push(struct device_s	*dev,
			    uint8_t		*data,
			    uint_fast16_t	size)
{
  struct net_ne2000_context_s	*pv = dev->drv_pv;
  struct ether_header		*hdr;
  struct net_packet_s		*packet;
  struct net_header_s		*nethdr;

  /* create a new packet object */
  packet = packet_obj_new(NULL);
  packet->packet = data;

  nethdr = &packet->header[0];
  nethdr->data = data;
  nethdr->size = size;

  /* get the good header */
  hdr = (struct ether_header*)nethdr->data;

  /* fill some info */
  packet->MAClen = sizeof(struct ether_addr);
  packet->sMAC = hdr->ether_shost;
  packet->tMAC = hdr->ether_dhost;
  packet->proto = net_be16_load(hdr->ether_type);

  /* prepare packet for next stage */
#ifdef CONFIG_NE2000_FRAGMENT
  nethdr[1].data = data + sizeof(struct ether_header) + 2;
  nethdr[1].size = size - sizeof(struct ether_header);
#else
  nethdr[1].data = data + sizeof(struct ether_header);
  nethdr[1].size = size - sizeof(struct ether_header);
#endif

  packet->stage++;

  packet_queue_lock_pushback(&pv->rcvqueue, packet);
}

/*
 * device IRQ handling.
 */

DEV_IRQ(net_ne2000_irq)
{
  struct net_ne2000_context_s	*pv = dev->drv_pv;
  uint_fast8_t			isr;

  /* select register bank 0 */
  ne2000_page(dev, NE2000_P0);

  assert(!cpu_interrupt_getstate());

  isr = cpu_io_read_8(dev->addr[NET_NE2000_ADDR] + NE2000_ISR);

  /* remote DMA completed */
  if (isr & NE2000_RDC)
    {
      uint_fast16_t	length = pv->current->header[0].size;

      if (pv->current)
	{
	  /* the packet is in the device memory, we can send it */
	  ne2000_dma(dev, NE2000_DMA_ABRT);

	  /* set page start */
	  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_TPSR, pv->tx_buf);

	  /* set packet size */
	  if (length < ETH_ZLEN)
	    length = ETH_ZLEN;
	  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_TBCR0, length);
	  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_TBCR1, length >> 8);

	  /* send it ! */
	  ne2000_command(dev, NE2000_TXP);

	  /* XXX timeout here */
	}

      /* acknowledge interrupt */
      cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_ISR, NE2000_RDC);
    }

  /* transmit error */
  if (isr & NE2000_TXE)
    {
      net_debug("ne2000: TXE\n");

      /* increment the retry counter */
      pv->send_tries++;

      /* too many retries, abording */
      if (pv->send_tries > 3)
	{
	  /* set the interrupt flags as "transmitted without error",
	     so the next queued packet can be sent */
	  isr &= NE2000_PTX;
	}
      else
	{
	  /* resend the transmit command */
	  ne2000_command(dev, NE2000_TXP);
	}

      /* acknowledge interrupt */
      cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_ISR, NE2000_TXE);
    }

  /* packet transmitted */
  if (isr & NE2000_PTX)
    {
      struct net_packet_s	*wait;

      /* packet sent successfully, drop it */
      packet_obj_refdrop(pv->current);

      /* one or more packets in the wait queue ? */
      if ((wait = packet_queue_pop(&pv->sendqueue)))
	{
	  /* take lock */
	  lock_spin_irq(&pv->lock);

	  pv->current = wait;

	  /* release lock */
	  lock_release_irq(&pv->lock);

	  ne2000_send(dev);
	}
      else
	{
	  /* take lock */
	  lock_spin_irq(&pv->lock);

	  pv->current = NULL;

	  /* release lock */
	  lock_release_irq(&pv->lock);
	}

      /* acknowledge interrupt */
      cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_ISR, NE2000_PTX);
    }

  /* buffer full */
  if (isr & NE2000_OVW)
    {
      net_debug("ne2000: OVW\n");
      /* XXX */

      /* acknowledge interrupt */
      cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_ISR, NE2000_OVW);
    }

  /* packet received */
  if (isr & NE2000_PRX)
    {
      /* no errors */
      if (cpu_io_read_8(dev->addr[NET_NE2000_ADDR] + NE2000_RSR) & NE2000_SPRX)
	{
	  struct ne2000_header_s	hdr;
	  uint_fast16_t			next;
	  uint_fast16_t			dma;
	  uint_fast16_t			size;
	  uint8_t			*data;

	  /* get next packet pointer */
	  next = cpu_io_read_8(dev->addr[NET_NE2000_ADDR] + NE2000_BOUND) + 1;
	  if (next > pv->mem)
	    next = pv->rx_buf;

	  /* read the packet header (automatically appended by the chip) */
	  dma = next << 8;
	  ne2000_mem_read(dev, dma, (struct ne2000_header_s *)&hdr,
			  sizeof (struct ne2000_header_s));

	  /* read the packet itself */
	  size = hdr.size - sizeof (struct ne2000_header_s);

#ifdef CONFIG_NE2000_FRAGMENT
	  data = mem_alloc(size + 2, MEM_SCOPE_CONTEXT);
	  ne2000_mem_read(dev, dma + sizeof (struct ne2000_header_s),
			  data, sizeof (struct ether_header));
	  ne2000_mem_read(dev, dma + sizeof (struct ne2000_header_s) +
			  sizeof (struct ether_header),
			  data + sizeof (struct ether_header) + 2,
			  size - sizeof (struct ether_header));
#else
	  data = mem_alloc(size, MEM_SCOPE_CONTEXT);
	  ne2000_mem_read(dev, dma + sizeof (struct ne2000_header_s),
			  data, size);
#endif

	  /* update the next packet pointer */
	  next = hdr.next;
	  if (next > pv->mem)
	    next = pv->rx_buf;
	  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_BOUND, next - 1);

	  /* push the packet into the network stack */
	  ne2000_push(dev, data, size);
	}

      /* acknowledge interrupt */
      cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_ISR, NE2000_PRX);
    }

  return 1;
}

/*
 * initializing the driver and the device.
 */

DEV_INIT(net_ne2000_init)
{
  struct net_ne2000_context_s	*pv;
  struct net_dispatch_s		*dispatch;

#ifndef CONFIG_STATIC_DRIVERS
  dev->drv = &net_ne2000_drv;
#endif

  printf("ne2000 driver init on device %p\n", dev);

  /* driver private data */
  pv = mem_alloc(sizeof(*pv), MEM_SCOPE_SYS);

  if (!pv)
    return -1;

  lock_init(&pv->lock);

  dev->drv_pv = pv;

  pv->current = NULL;

  /* probe and init device */
  if (!ne2000_probe(dev))
    {
      mem_free(pv);

      return -1;
    }
  printf("ne2000: detected a %s ne2000 device with %d kb\n",
	 pv->io_16 ? "16 bits" : "8 bits", pv->mem << 8);
  printf("ne2000: MAC is %P\n", pv->mac, ETH_ALEN);
  ne2000_init(dev);

  /* setup some containers */
  packet_queue_init(&pv->sendqueue);
  packet_queue_lock_init(&pv->rcvqueue);
  net_protos_init(&pv->protocols);

  /* bind to ICU */
  DEV_ICU_BIND(icudev, dev);

  /* start dispatch thread */
  dispatch = mem_alloc(sizeof (struct net_dispatch_s), MEM_SCOPE_SYS);
  dispatch->device = dev;
  dispatch->packets = &pv->rcvqueue;
  dispatch->protocols = &pv->protocols;
  if (pthread_create(&pv->dispatch, NULL, packet_dispatch, (void *)dispatch))
    {
      printf("ne2000: cannot start dispatch thread\n");

      net_ne2000_cleanup(dev);

      return -1;
    }

  /* register as a net device */
  if_register(dev);

  return 0;
}

/*
 * cleaning the driver.
 */

DEV_CLEANUP(net_ne2000_cleanup)
{
  struct net_ne2000_context_s	*pv = dev->drv_pv;
  struct net_packet_s		*wait;

  /* unregister the device */
  if_unregister(dev);

  /* destroy the packet scheduled for sending if necessary */
  if (pv->current)
    packet_obj_refdrop(pv->current);

  /* empty the sendqueue */
  while ((wait = packet_queue_pop(&pv->sendqueue)))
    {
      packet_obj_refdrop(wait);
    }

  /* terminate the dispatch thread */
  /* XXX */

  /* empty the receive queue */
  while ((wait = packet_queue_lock_pop(&pv->rcvqueue)))
    {
      packet_obj_refdrop(wait);
    }

  /* destroy some containers */
  packet_queue_destroy(&pv->sendqueue);
  packet_queue_lock_destroy(&pv->rcvqueue);
  net_protos_destroy(&pv->protocols);

  /* turn the device off */
  ne2000_dma(dev, NE2000_DMA_ABRT);
  ne2000_command(dev, NE2000_STP);

  /* free private data */
  mem_free(pv);
}

/*
 * preparing a packet.
 */

DEVNET_PREPAREPKT(net_ne2000_preparepkt)
{
  struct net_ne2000_context_s	*pv = dev->drv_pv;
  struct net_header_s		*nethdr;
  uint_fast16_t			total = 0;
  uint8_t			*buff;

#ifdef CONFIG_NE2000_FRAGMENT
  total = sizeof (struct ether_header) + size + 2;
#else
  total = sizeof (struct ether_header) + size;
#endif

  buff = packet->packet = mem_alloc(total, MEM_SCOPE_CONTEXT);

  nethdr = &packet->header[0];
  nethdr->data = buff;
  nethdr->size = total;

  packet->stage = 1;

  packet->sMAC = pv->mac;
  packet->MAClen = ETH_ALEN;

#ifdef CONFIG_NE2000_FRAGMENT
  /* when we use fragmentation, the next packet will be aligned on 4
     bytes boundary */
  return buff + sizeof (struct ether_header) + 2;
#else
  return buff + sizeof (struct ether_header);
#endif
}

/*
 * sending a packet.
 */

DEVNET_SENDPKT(net_ne2000_sendpkt)
{
  struct net_ne2000_context_s	*pv = dev->drv_pv;
  struct ether_header		*hdr;
  struct net_header_s		*nethdr;

  /* get a pointer to the header */
  nethdr = &packet->header[0];
  hdr = (struct ether_header*)nethdr->data;

  /* fill the header */
  memcpy(hdr->ether_shost, packet->sMAC, packet->MAClen);
  memcpy(hdr->ether_dhost, packet->tMAC, packet->MAClen);
  net_be16_store(hdr->ether_type, proto);

  /* take lock */
  lock_spin_irq(&pv->lock);

  /* if the device is busy, queue the packet */
  if (cpu_io_read_8(dev->addr[NET_NE2000_ADDR] + NE2000_CMD) & NE2000_TXP ||
      pv->current)
    {
      packet_queue_push(&pv->sendqueue, packet);

      return;
    }

  /* otherwise, send the datagram immediately */
  pv->current = packet;

  /* release lock */
  lock_release_irq(&pv->lock);

  ne2000_send(dev);
}

/*
 * registering a new protocol.
 */

DEVNET_REGISTER_PROTO(net_ne2000_register_proto)
{
  struct net_ne2000_context_s	*pv = dev->drv_pv;
  va_list			va;

  va_start(va, proto);

  /* call the protocol constructor */
  if (proto->desc->initproto)
    proto->desc->initproto(dev, proto, va);

  /* insert in the protocol list */
  net_protos_push(&pv->protocols, proto);

  va_end(va);
}

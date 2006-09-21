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

#include <hexo/device/icu.h>
#include <hexo/device/net.h>
#include <hexo/device.h>
#include <hexo/driver.h>

#include <hexo/iospace.h>
#include <hexo/alloc.h>
#include <hexo/lock.h>
#include <hexo/interrupt.h>
#include <hexo/endian.h>

#include <pthread.h>
#include <semaphore.h>

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
  }
};
#endif


/*
 * prepare sending a packet.
 */

static void	ne2000_send(struct device_s	*dev)
{
  struct net_ne2000_context_s	*pv = dev->drv_pv;
  struct net_header_s		*nethdr;
  uint_fast16_t			size;
  uint_fast8_t			i;

  nethdr = &pv->current->header[0];

  /* copy in one time */
  size = (pv->io_16 ? ALIGN_VALUE(nethdr->size, 2) : nethdr->size);

  /* reset the tries counter */
  pv->send_tries = 0;

  /* initialize writing */
  ne2000_dma_init_write(dev, pv->tx_buf << 8, size);

  for (i = 0; nethdr[i].data; i++)
    {
      uint_fast16_t	fragsz;

      net_debug("%u: data %p, size %u\n", i, nethdr[i].data, nethdr[i].size);
      net_debug("%u: data %p, size %u\n", i + 1, nethdr[i + 1].data, nethdr[i + 1].size);

      if (!nethdr[i + 1].data)
	fragsz = nethdr[i].size;
      else
	fragsz = nethdr[i].size - nethdr[i + 1].size;
      net_debug("chunk size %u\n", fragsz);
      /* write each chunk after the previous one */
      ne2000_dma_do_write(dev, nethdr[i].data, fragsz);
    }

  /* the data being written, we wait for remote DMA to be completed (IRQ) */
}

/*
 * read a packet from the NIC ring buffer.
 */

static void			ne2000_rx(struct device_s	*dev,
					  uint8_t		**data,
					  uint_fast16_t		*size)
{
  struct net_ne2000_context_s	*pv = dev->drv_pv;
  struct ne2000_header_s	hdr;
  uint_fast16_t			next;
  uint_fast16_t			dma;
  uint8_t			*buf;

  /* get next packet pointer */
  next = cpu_io_read_8(dev->addr[NET_NE2000_ADDR] + NE2000_BOUND) + 1;
  if (next > pv->mem)
    next = pv->rx_buf;

  /* read the packet header (automatically appended by the chip) */
  dma = next << 8;
  ne2000_mem_read(dev, dma, (struct ne2000_header_s *)&hdr,
		  sizeof (struct ne2000_header_s));

  /* read the packet itself */
  *size = hdr.size - sizeof (struct ne2000_header_s);

#ifdef CONFIG_NE2000_FRAGMENT
  buf = *data = mem_alloc(*size + 2, MEM_SCOPE_CONTEXT);
  ne2000_mem_read(dev, dma + sizeof (struct ne2000_header_s),
		  buf, sizeof (struct ether_header));
  ne2000_mem_read(dev, dma + sizeof (struct ne2000_header_s) +
		  sizeof (struct ether_header),
		  buf + sizeof (struct ether_header) + 2,
		  *size - sizeof (struct ether_header));
#else
  buf = *data = mem_alloc(*size, MEM_SCOPE_CONTEXT);
  ne2000_mem_read(dev, dma + sizeof (struct ne2000_header_s),
		  buf, *size);
#endif

  /* update the next packet pointer */
  next = hdr.next;
  if (next > pv->mem)
    next = pv->rx_buf;
  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_BOUND, next - 1);
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

  sem_post(&pv->rcvsem);
}

/*
 * device IRQ handling.
 */

DEV_IRQ(net_ne2000_irq)
{
  struct net_ne2000_context_s	*pv = dev->drv_pv;
  uint_fast8_t			tx_serviced = 0;
  uint_fast8_t			isr;

  printf("IRQ\n");

  /* lock the device */
  lock_spin(&pv->lock);

  /* select register bank 0 */
  ne2000_page(dev, NE2000_P0);

  /* the loop is needed because some devices do not re emit interrupts
     until all the bits in ISR are reset */
  while ((isr = cpu_io_read_8(dev->addr[NET_NE2000_ADDR] + NE2000_ISR)))
    {
     /* remote DMA completed */
      if (isr & NE2000_RDC)
	{
	  if (pv->current)
	    {
	      uint_fast16_t	length = pv->current->header[0].size;

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
	}

      /* transmit error */
      if (isr & NE2000_TXE)
	{
	  net_debug("%s: TXE\n", pv->interface->name);

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
	}

      /* packet transmitted */
      if (isr & NE2000_PTX)
	{
	  struct net_packet_s	*wait;

	  /* the big dirty test. on some device, the PTX bit of ISR is
	     not correctly acknowleged */
	  if (!tx_serviced && pv->current)
	    {
	      /* packet sent successfully, drop it */
	      packet_obj_refdrop(pv->current);

	      /* one or more packets in the wait queue ? */
	      if ((wait = packet_queue_pop(&pv->sendqueue)))
		{
		  pv->current = wait;

		  ne2000_send(dev);
		}
	      else
		pv->current = NULL;

	      tx_serviced = 1;
	    }
	}

      /* buffer full */
      if (isr & NE2000_OVW)
	{
	  uint_fast8_t	cr;
	  uint_fast8_t	resend = 0;
	  uint_fast8_t	i;
	  uint_fast16_t	total;

	  net_debug("%s: recovery from overflow\n", pv->interface->name);

	  /* save the NIC state */
	  cr = cpu_io_read_8(dev->addr[NET_NE2000_ADDR] + NE2000_CMD);

	  /* stop completely the device */
	  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_CMD,
			 NE2000_P0 | NE2000_DMA_ABRT | NE2000_STP);

	  /* wait for the NIC to complete operations */
	  for (i = 0; i < 10000000; i++)
	    ;
	  /* XXX */
	  //usleep(1600);

	  /* reset remote byte count registers */
	  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_RBCR0, 0);
	  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_RBCR1, 0);

	  /* do we need to resend the outgoing packet ? */
	  resend = (cr & NE2000_TXP) && !((isr & NE2000_PTX) || (isr & NE2000_TXE));

	  /* enter loopback mode */
	  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_TCR, 0x2);

	  /* bring the device up */
	  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_CMD,
			 NE2000_P0 | NE2000_DMA_ABRT | NE2000_STA);

	  /* read some packets until half the memory is free */
	  for (total = 0; total < ((pv->mem - pv->rx_buf) << 8) / 2; )
	    {
	      uint_fast16_t	size;
	      uint8_t	*data;

	      /* read a packet */
	      ne2000_rx(dev, &data, &size);

	      /* push the packet into the network stack */
	      ne2000_push(dev, data, size);

	      total += size;
	    }

	  /* force ISR reset */
	  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_ISR, isr);

	  /* setup transmit configuration register (disable loopback) */
	  cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_TCR, NE2000_AUTOCRC);

	  /* if a transmission was aborted, restart it */
	  if (resend)
	    ne2000_command(dev, NE2000_PTX);
	}

      /* packet received */
      if (isr & NE2000_PRX)
	{
	  /* no errors */
	  if (cpu_io_read_8(dev->addr[NET_NE2000_ADDR] + NE2000_RSR) & NE2000_SPRX)
	    {
	      uint_fast16_t			size;
	      uint8_t			*data;

	      /* read the packet */
	      ne2000_rx(dev, &data, &size);

	      /* push the packet into the network stack */
	      ne2000_push(dev, data, size);
	    }
	}

      /* acknowledge interrupt */
      cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_ISR, isr);
    }

  /* release lock */
  lock_release(&pv->lock);

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

  /* register as a net device */
  pv->interface = if_register(dev, IF_ETHERNET, pv->mac);

  /* start dispatch thread */
  if (sem_init(&pv->rcvsem, 0, 0))
    {
      printf("ne2000: cannot init dispatch semaphore\n");

      net_ne2000_cleanup(dev);

      return -1;
    }
  dispatch = mem_alloc(sizeof (struct net_dispatch_s), MEM_SCOPE_SYS);
  dispatch->interface = pv->interface;
  dispatch->packets = &pv->rcvqueue;
  dispatch->sem = &pv->rcvsem;

  if (pthread_create(&pv->dispatch, NULL, packet_dispatch, (void *)dispatch))
    {
      printf("ne2000: cannot start dispatch thread\n");

      mem_free(dispatch);
      net_ne2000_cleanup(dev);

      return -1;
    }

  /* bind to ICU */
  DEV_ICU_BIND(icudev, dev);

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
  if_unregister(pv->interface);

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

  /* destroy the receive semaphore */
  sem_destroy(&pv->rcvsem);

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
  total = sizeof (struct ether_header) + size + 2 + max_padding;
#else
  total = sizeof (struct ether_header) + size;
#endif

  buff = packet->packet = mem_alloc(total, MEM_SCOPE_CONTEXT);

  nethdr = &packet->header[0];
  nethdr->data = buff;
  nethdr->size = sizeof (struct ether_header) + size;

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
  if ((cpu_io_read_8(dev->addr[NET_NE2000_ADDR] + NE2000_CMD) & NE2000_TXP) ||
      pv->current != NULL)
    {
      packet_queue_push(&pv->sendqueue, packet);
    }
  else
    {
      /* otherwise, send the datagram immediately */
      pv->current = packet;

      ne2000_send(dev);
    }

  /* release lock */
  lock_release_irq(&pv->lock);
}

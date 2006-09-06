#include <hexo/types.h>
#include <hexo/device.h>
#include <hexo/iospace.h>
#include <hexo/alloc.h>
#include <hexo/lock.h>
#include <hexo/interrupt.h>

#include "net-ns8390.h"

#include "net-ns8390-private.h"

#include "ns8390.h"

/**************************************************************/

/*
 * device open operation
 */

#ifndef CONFIG_STATIC_DRIVERS
const struct driver_s	net_ns8390_drv =
{
  .f_init		= net_ns8390_init,
  .f_cleanup		= net_ns8390_cleanup,
  .f_irq		= NULL, /*net_ns8390_irq,*/
  .f.chr = {
    .f_read		= net_ns8390_read,
    .f_write		= net_ns8390_write,
  }
};
#endif

/*
 * programmed I/O reading.
 */

static void	net_ns8390_pio_read(struct net_ns8390_context_s		*pv,
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

static void	net_ns8390_pio_write(struct net_ns8390_context_s	*pv,
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

static void	net_ns8390_reset(struct net_ns8390_context_s	*pv)
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

static error_t			net_ns8390_probe(struct net_ns8390_context_s	*pv)
{
  static const uint_fast16_t	probe[] = { /*0x280, 0x300, 0x320, 0x340,*/ 0xC100, 0 };
  uint_fast16_t			base;
  uint_fast8_t			i;
  static char			ref_buff[] = "TEST";
  uint8_t			buff[32];
  uint8_t			rom[32];

  /* loop through the different base addresses */
  for (i = 0; ; i++)
    {
      base = probe[i];
      if (!base)
	break;
      printf("Probing base 0x%x\n", base);

      pv->mode_16bits = 0;
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
	break; /* test succeeded */

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
	break; /* test suceeded */
    }

  if (!base)
    return -1; /* no device found */

  printf("Found a NE2000 at I/O base 0x%x\n", base);

  /* read MAC address */
  net_ns8390_pio_read(pv, 0, rom, sizeof (rom));

  for (i = 0; i < ETH_ALEN; i++)
    pv->mac[i] = rom[i << 1];

  printf("MAC address: %2x:%2x:%2x:%2x:%2x:%2x\n",
	 pv->mac[0], pv->mac[1], pv->mac[2],
	 pv->mac[3], pv->mac[4], pv->mac[5]);

  return 0;
}

/*
 * device read operation
 */

DEVCHAR_READ(net_ns8390_read)
{
  struct net_ns8390_context_s	*pv = dev->drv_pv;

  return 0;
}

/*
 * device write operation
 */

DEVCHAR_WRITE(net_ns8390_write)
{
  struct net_ns8390_context_s	*pv = dev->drv_pv;
  size_t			len;

  net_ns8390_pio_write(pv, data, pv->tx_start << 8, size);
  len = size;
  if (len < ETH_ZLEN)
    len = ETH_ZLEN;
  cpu_io_write_8(pv->base + D8390_P0_COMMAND,
		 D8390_COMMAND_PS0 | D8390_COMMAND_RD2 | D8390_COMMAND_STA);
  cpu_io_write_8(pv->base + D8390_P0_TPSR, pv->tx_start);
  cpu_io_write_8(pv->base + D8390_P0_TBCR0, len);
  cpu_io_write_8(pv->base + D8390_P0_TBCR0, len >> 8);
  cpu_io_write_8(pv->base + D8390_P0_COMMAND, D8390_COMMAND_PS0 |
		 D8390_COMMAND_TXP | D8390_COMMAND_RD2 | D8390_COMMAND_STA);

  return size;
}

/*
 * device close operation
 */

DEV_CLEANUP(net_ns8390_cleanup)
{
  struct net_ns8390_context_s	*pv = dev->drv_pv;

  mem_free(pv);
}

/*
 * device open operation
 */

DEV_INIT(net_ns8390_init)
{
  struct net_ns8390_context_s	*pv;

  /* driver private data */
  pv = mem_alloc(sizeof(*pv), MEM_SCOPE_SYS);

  if (!pv)
    return -1;

  lock_init(&pv->lock);

  dev->drv_pv = pv;

  if (net_ns8390_probe(pv))
    printf("No NE2000 device found\n");

  net_ns8390_reset(pv);

  return 0;
}

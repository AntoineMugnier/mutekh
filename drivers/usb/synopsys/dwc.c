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
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright (c) 2016 Sebastien Cerdan <sebcerdan@gmail.com>

*/

#include "dwc_regs.h"
#include "dwc.h"

/*
   This is a driver for the Synopsys DWC OTG usb controler. 
    - Only low/full speed is supported (SYNOPSYS limitation).
    - Only internal DMA mode is implemented.

   This driver support all transfer types.
*/

struct usbdev_endpoint_s * synopsys_usbdev_endpoint(struct synopsys_usbdev_private_s *pv, 
                                                    enum usb_endpoint_dir_e dir,
                                                    uint8_t address)
{
  if (address > CONFIG_DRIVER_USB_SYNOPSYS_EP_COUNT)
    return NULL;

  struct usbdev_endpoint_s * ep;

  if (dir == USB_EP_OUT)
    ep = pv->epo + address - 1;
  else
    ep = pv->epi + address - 1;

  return ep;
}

#if (CONFIG_USBDEV_MAX_ALTERNATE_COUNT > 0)
/* Flush  TX fifo */
static void synopsys_usbdev_flush_tx_fifo(struct synopsys_usbdev_private_s *pv, uint8_t epidx)
{
  uint32_t x = (epidx << SYNOPSYS_USB_GRSTCTL_TXFNUM_SHIFT) | SYNOPSYS_USB_GRSTCTL_TXFFLSH;
  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_GRSTCTL_ADDR, endian_le32(x));
 
  while (1)
    {
      /* Take 8 core cycles to clear */
      x = endian_le32(cpu_mem_read_32(pv->addr + SYNOPSYS_USB_GRSTCTL_ADDR));
      if (!(x & SYNOPSYS_USB_GRSTCTL_TXFFLSH))
        break;
    }
}
#endif

static void synopsys_usbdev_edp_activation(struct synopsys_usbdev_private_s *pv, 
                                        struct dev_usbdev_interface_cfg_s *itf)
{
  uint32_t x;
  /* Get dma interrupt mask */
  uint32_t daint = endian_le32(cpu_mem_read_32(pv->addr + SYNOPSYS_USB_DAINTMSK_ADDR));

  if (itf == NULL)
  /* Enable endpoint 0 */
    {
      uint32_t mps = usbdev_stack_get_ep0_mps(&pv->usbdev_ctx) >> 4;
      mps = (0x1B >> (mps << 1)) & 0x3;

      /* Same value on EPIN and EPOUT */
      x = SYNOPSYS_USB_DIEPCTL_MPS_SHIFT_VAL(mps); 
      SYNOPSYS_USB_DIEPCTL_ETYPE_SETVAL(x, USB_EP_CONTROL);

      cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DIEPCTL_ADDR(0), endian_le32(x));
      cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DOEPCTL_ADDR(0), endian_le32(x));

      daint |= SYNOPSYS_USB_DAINTMSK_OUTEPMSK(0) | SYNOPSYS_USB_DAINTMSK_INEPMSK(0);
    }
  else
  /* Enable endpoints n */
    {
      USBDEV_FOREACH_ENDPOINT(itf->i, itf->epi, itf->epo,
       {
         if (USB_GET_EDP_TYPE(epdesc) == USB_EP_CONTROL)
             pv->ctrl |= 1 << epaddr;

         if (usbdev_is_endpoint_in(epdesc))
           /* TX endpoint */
           {
             /* Clear drop bit */
             x = SYNOPSYS_USB_DIEPINT_PKTDRPSTS;
             cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DIEPINT_ADDR(epaddr), endian_le32(x));

             x = SYNOPSYS_USB_DIEPCTL_MPS_SHIFT_VAL(USB_GET_EDP_MPS(epdesc)) |
                 SYNOPSYS_USB_DIEPCTL_TXFNUM_SHIFT_VAL(epaddr) |
                 SYNOPSYS_USB_DIEPCTL_SETD0PIDEF | 
                 SYNOPSYS_USB_DIEPCTL_USBACTEP; 
             SYNOPSYS_USB_DIEPCTL_ETYPE_SETVAL(x, USB_GET_EDP_TYPE(epdesc));
             
             cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DIEPCTL_ADDR(epaddr), endian_le32(x));

             /* Enable corresponding DMA irq */
             daint |= SYNOPSYS_USB_DAINTMSK_INEPMSK(epaddr);

           }
         if (usbdev_is_endpoint_out(epdesc))
           /* RX endpoint */
           {
             /* Clear drop bit */
             x = SYNOPSYS_USB_DOEPINT_PKTDRPSTS;
             cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DOEPINT_ADDR(epaddr), endian_le32(x));

             x = SYNOPSYS_USB_DOEPCTL_MPS_SHIFT_VAL(USB_GET_EDP_MPS(epdesc)) |
                 SYNOPSYS_USB_DOEPCTL_USBACTEP | 
                 SYNOPSYS_USB_DOEPCTL_SETD0PIDEF; 
             SYNOPSYS_USB_DOEPCTL_ETYPE_SETVAL(x, USB_GET_EDP_TYPE(epdesc)); 
             
             cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DOEPCTL_ADDR(epaddr), endian_le32(x));

             /* Enable corresponding interrupt */
             daint |= SYNOPSYS_USB_DAINTMSK_OUTEPMSK(epaddr);
           }

        });
    }
  /* Apply irq mask */
  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DAINTMSK_ADDR, endian_le32(daint));
}

static error_t synopsys_usbdev_configure(struct synopsys_usbdev_private_s *pv,
                                      struct dev_usbdev_config_s *cfg)
{
  uint16_t rx_ram_size[CONFIG_DRIVER_USB_SYNOPSYS_EP_COUNT] = {0};
  uint16_t tx_ram_size[CONFIG_DRIVER_USB_SYNOPSYS_EP_COUNT] = {0};

  uint16_t mps0 = usbdev_stack_get_ep0_mps(&pv->usbdev_ctx) >> 2;
  uint16_t rx_ram_total_size = 22 + 4 + 1 + 1 + mps0;

  uint16_t tx, rx;

  struct dev_usbdev_interface_cfg_s *icfg;

  if (cfg->itf == NULL)
     synopsys_usbdev_edp_activation(pv, NULL);
  else
    {
      for (uint8_t i = 0; i< CONFIG_USBDEV_MAX_INTERFACE_COUNT; i++)
        {
          icfg = cfg->itf + i;

          if (icfg->i == NULL)
            break;
    
          /* Configure endpoint */
           synopsys_usbdev_edp_activation(pv, icfg);
    
          const struct usbdev_interface_s *itf = icfg->i;
    
          /* Allocate memory */
          USBDEV_FOREACH_INTERFACE(itf,
            {
              USBDEV_FOREACH_ENDPOINT(itf, icfg->epi, icfg->epo,
               {
                 if (epaddr > CONFIG_DRIVER_USB_SYNOPSYS_EP_COUNT)
                   return -ENOTSUP;
                 
                 /* RX and TX memory size to allocate for endpoint */
                 tx = rx = 0;
              
                 if (USB_GET_EDP_TYPE(epdesc) == USB_EP_CONTROL)
                 /* INOUT endpoint */
                   {
                     rx = 4 + (USB_GET_EDP_MPS(epdesc) >> 2) + 1 + 1;
                     tx = USB_GET_EDP_MPS(epdesc) >> 2;
                   }
                 else if (USB_GET_EDP_DIR(epdesc) == USB_EP_IN)
                 /* IN endpoint */{
                   tx = USB_GET_EDP_MPS(epdesc) >> 2;
                   }
                 else
                 /* OUT endpoint */
                   {
                     rx  = (USB_GET_EDP_MPS(epdesc) >> 2);
                     if (USB_GET_EDP_TYPE(epdesc) == USB_EP_ISOCHRONOUS)
                       rx <<= 1;
                     rx += 1;
                   }
            
                 if (tx && tx > tx_ram_size[epaddr - 1])
                   tx_ram_size[epaddr - 1] = tx;
            
                 if (rx && rx > rx_ram_size[epaddr - 1])
                   rx_ram_size[epaddr - 1] = rx;
               });
            });
    
          for (uint8_t i = 0; i < CONFIG_DRIVER_USB_SYNOPSYS_EP_COUNT; i++)
            rx_ram_total_size += rx_ram_size[i];
        }
    }

  uint16_t ptx = rx_ram_total_size;
  uint16_t size;
  uint32_t x;

  /* Set RX fifo size */
  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_GRXFSIZE_ADDR, ptx);

  /* Set each TX fifo size */
  for (uint8_t i = 0; i<CONFIG_DRIVER_USB_SYNOPSYS_EP_COUNT + 1; i++)
    {
      if (i == 0)
        {
          size = usbdev_stack_get_ep0_mps(&pv->usbdev_ctx) >> 2;

          x = SYNOPSYS_USB_GNPTXFSIZE_NPTXFINEPTXF0DEP_SHIFT_VAL(size) | 
            SYNOPSYS_USB_GNPTXFSIZE_NPTXFSTADDR_SHIFT_VAL(ptx);
          cpu_mem_write_32(pv->addr + SYNOPSYS_USB_GNPTXFSIZE_ADDR, endian_le32(x));
        }
      else
        {
          size = tx_ram_size[i - 1];

          if (size == 0)
            continue;

          x = SYNOPSYS_USB_DIEPTXF_INEPNTXFDEP_SHIFT_VAL(size) |
            SYNOPSYS_USB_DIEPTXF_INEPNTXFSTADDR_SHIFT_VAL(ptx);
          cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DIEPTXF_ADDR(i - 1), endian_le32(x));
        }
      ptx += size;
    }

  if (ptx > SYNOPSYS_USB_CTRL_RAM_SIZE)
    return -ENOTSUP;

  x = SYNOPSYS_USB_GRSTCTL_TXFNUM_SHIFT_VAL(ALLFIFO) | SYNOPSYS_USB_GRSTCTL_TXFFLSH |
    SYNOPSYS_USB_GRSTCTL_RXFFLSH;
  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_GRSTCTL_ADDR, endian_le32(x));

  while (1)
    {
      x = endian_le32(cpu_mem_read_32(pv->addr + SYNOPSYS_USB_GRSTCTL_ADDR));
      if (!(x & (SYNOPSYS_USB_GRSTCTL_RXFFLSH | SYNOPSYS_USB_GRSTCTL_TXFFLSH)))
        break;
    }

  return 0;
}

static void synopsys_usbdev_set_flag(struct synopsys_usbdev_private_s *pv, bool_t in,
                                  uint32_t flag, uint8_t idx)
{
  if (in)
    {
      uint32_t x = cpu_mem_read_32(pv->addr + SYNOPSYS_USB_DIEPCTL_ADDR(idx));
      x |= flag;
      cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DIEPCTL_ADDR(idx), x);
      pv->imask |= 1 << idx;
    }
  else
    {
      uint32_t x = cpu_mem_read_32(pv->addr + SYNOPSYS_USB_DOEPCTL_ADDR(idx));
      x |= flag;
      cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DOEPCTL_ADDR(idx), x);
      pv->omask |= 1 << idx;
    }
}

static bool_t synopsys_usbedv_is_ctrl_ep(struct synopsys_usbdev_private_s *pv,
                                      struct dev_usbdev_request_s * tr);

static void synopsys_usbdev_get_in_tsize(struct synopsys_usbdev_private_s *pv,
                                      struct dev_usbdev_request_s *tr,
                                      size_t *size, size_t *pktcnt)
{
  if (tr->type == DEV_USBDEV_CTRL_STATUS_IN)
    {
      *size = 0;
      *pktcnt = 1;
      return;
    }

  uint16_t mps0 = usbdev_stack_get_ep0_mps(&pv->usbdev_ctx);
  uint32_t ctl = endian_le32(cpu_mem_read_32(pv->addr + SYNOPSYS_USB_DIEPCTL_ADDR(tr->ep)));
  uint32_t s = tr->size;
  uint32_t max = tr->ep ? SYNOPSYS_USB_DIEPCTL_MPS_GET(ctl) : mps0;

  if (tr->ep == 0 && s >> 7)
   s = (0x7f/mps0) * mps0;

  uint32_t p = (s + max - 1) / max + !s;

  if (tr->ep == 0 && p >> 2)
   {
     p = 3;
     s = 3 * mps0;
   }

  *size = s;
  *pktcnt = p;
}

static void synopsys_usbdev_send_data(struct synopsys_usbdev_private_s *pv,
                                   struct dev_usbdev_request_s *tr)
{
  uint32_t x, diepctl;
  uint32_t size, pcnt;

  synopsys_usbdev_get_in_tsize(pv, tr, &size, &pcnt);

  x = SYNOPSYS_USB_DIEPTSIZE_XFERSIZE_SHIFT_VAL(size) |
      SYNOPSYS_USB_DIEPTSIZE_PKTCNT_SHIFT_VAL(pcnt);

  /* Set transfer size */
  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DIEPTSIZE_ADDR(tr->ep), x);

  assert(tr->type == DEV_USBDEV_CTRL_STATUS_IN || address_is_aligned(tr->data, 4));

  /* Program DMA address */
  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DIEPDMAADDR_ADDR(tr->ep), endian_le32((uint32_t)tr->data));

  /* Start transfer */
  diepctl = endian_le32(cpu_mem_read_32(pv->addr + SYNOPSYS_USB_DIEPCTL_ADDR(tr->ep)));
  diepctl |= SYNOPSYS_USB_DIEPCTL_CNAK | SYNOPSYS_USB_DIEPCTL_EPENA;
  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DIEPCTL_ADDR(tr->ep), diepctl);
}

static void synopsys_usbdev_get_out_tsize(struct synopsys_usbdev_private_s *pv,
                                       struct dev_usbdev_request_s *tr,
                                       size_t *size, size_t *pktcnt)
{
  *pktcnt = 1;
  *size = 0;

  if (tr->type == DEV_USBDEV_CTRL_STATUS_OUT)
    return;

  uint32_t ctl = endian_le32(cpu_mem_read_32(pv->addr + SYNOPSYS_USB_DOEPCTL_ADDR(tr->ep)));
  uint32_t max = tr->ep ? SYNOPSYS_USB_DOEPCTL_MPS_GET(ctl) : usbdev_stack_get_ep0_mps(&pv->usbdev_ctx);

  /* Size of read buffer must be at least maximum packet size */
  assert(max <= tr->size);
    
  *size = (max % 4) ? max + 4 - (max & 0x3) : max;
}

static void synopsys_usbdev_get_data(struct synopsys_usbdev_private_s *pv,
                                  struct dev_usbdev_request_s *tr)
{
  uint32_t size, pcnt;

  synopsys_usbdev_get_out_tsize(pv, tr, &size, &pcnt);

  uint32_t x = SYNOPSYS_USB_DOEPTSIZE_XFERSIZE_SHIFT_VAL(size) |
               SYNOPSYS_USB_DOEPTSIZE_PKTCNT_SHIFT_VAL(pcnt);

  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DOEPTSIZE_ADDR(tr->ep), x);

  assert(address_is_aligned(tr->data, 4));

  /* Program DMA address */
  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DOEPDMAADDR_ADDR(tr->ep), endian_le32((uint32_t)tr->data));

  /* Start transfer */
  x = endian_le32(cpu_mem_read_32(pv->addr + SYNOPSYS_USB_DOEPCTL_ADDR(tr->ep)));
  x |= SYNOPSYS_USB_DOEPCTL_CNAK | SYNOPSYS_USB_DOEPCTL_EPENA;
  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DOEPCTL_ADDR(tr->ep), x);
}

static bool_t synopsys_usbedv_is_ctrl_ep(struct synopsys_usbdev_private_s *pv,
                                      struct dev_usbdev_request_s * tr)
{
  return ((pv->ctrl & (1 << tr->ep)) != 0);
}

static inline void synopsys_usbdev_end_in_transfer(struct synopsys_usbdev_private_s *pv,
                                                struct dev_usbdev_request_s * tr)
{
  if (tr->type == DEV_USBDEV_CTRL_STATUS_IN)
    goto done;

  uint32_t pcktsize, base, rem, done, x;
  uint32_t size;

  x = cpu_mem_read_32(pv->addr + SYNOPSYS_USB_DIEPCTL_ADDR(tr->ep));
  pcktsize = SYNOPSYS_USB_DIEPCTL_MPS_GET(x);

  /* Retrieve remainning packet to sent on USB bus */
  rem = endian_le32(cpu_mem_read_32(pv->addr + SYNOPSYS_USB_DIEPTSIZE_ADDR(tr->ep)));
  rem = SYNOPSYS_USB_DIEPTSIZE_PKTCNT_GET(rem);

  /* Initial number of packet to send */
  synopsys_usbdev_get_in_tsize(pv, tr, &size, &base);
  done = rem == 0 ? size : (base - rem) * pcktsize;

  /* Udate request */
  tr->data = (uint8_t*)tr->data + done;
  tr->size -= done;

  if (!tr->size)
    goto done;

  if (tr->type == DEV_USBDEV_PARTIAL_DATA_IN || tr->error)
    goto done;

  return synopsys_usbdev_send_data(pv, tr);

done:
  pv->tri[tr->ep] = NULL;
  return usbdev_stack_request_done(&pv->usbdev_ctx, tr);
}

static inline void synopsys_usbdev_end_out_transfer(struct synopsys_usbdev_private_s *pv,
                                                 struct dev_usbdev_request_s * tr)
{
  if (tr->type == DEV_USBDEV_CTRL_SETUP ||
      tr->type == DEV_USBDEV_CTRL_STATUS_OUT)
    goto done;

  uint32_t base, rem, pcnt;

  synopsys_usbdev_get_out_tsize(pv, tr, &base, &pcnt);

  /* Retrieve remaining size */
  rem = endian_le32(cpu_mem_read_32(pv->addr + SYNOPSYS_USB_DOEPTSIZE_ADDR(tr->ep)));
  rem = SYNOPSYS_USB_DOEPTSIZE_XFERSIZE_GET(rem);
  
  uint32_t done = base - rem;

  /* Udate request size and data pointer */

  tr->data = (uint8_t*)tr->data + done;
  tr->size -= done;

  if (synopsys_usbedv_is_ctrl_ep(pv, tr))
    {
      tr->rem -= done;
      /* End of data stage or buffer full */
      if (!tr->rem)
        goto done;
    }
  
  if (tr->size == 0 || rem)
    goto done;

  if (tr->type == DEV_USBDEV_PARTIAL_DATA_OUT ||
      tr->error)
    goto done;

  return synopsys_usbdev_get_data(pv, tr);

done:
  pv->tro[tr->ep] = NULL;
  usbdev_stack_request_done(&pv->usbdev_ctx, tr);
}

void synopsys_usbdev_stack_event(struct synopsys_usbdev_private_s *pv)
{
  struct dev_usbdev_request_s *tr;

  pv->pevent = 0;

  if (pv->tri[0])
    tr = pv->tri[0];
  else
    tr = pv->tro[0];

  if (tr == NULL)
    return;

  tr->event = pv->event;
  tr->error = -EIO;

  pv->event = 0;
  pv->tri[0] = pv->tro[0] = NULL;

  return usbdev_stack_request_done(&pv->usbdev_ctx, tr);
}

static void synopsys_usbdev_end_transfer(struct synopsys_usbdev_private_s *pv, bool_t cb) 
{
  struct dev_usbdev_request_s * tr;

  /* Clear global out NAK interrupt */
  uint32_t x = cpu_mem_read_32(pv->addr + SYNOPSYS_USB_DCTL_ADDR);
  x |= SYNOPSYS_USB_DCTL_CGOUTNAK;
  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DCTL_ADDR, x);
  /* Reenable global OUT NAK interrupt */
  x = endian_le32(cpu_mem_read_32(pv->addr + SYNOPSYS_USB_GINTMSK_ADDR));
  x |= SYNOPSYS_USB_GINTMSK_GOUTNAKEFFMSK;
  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_GINTMSK_ADDR, endian_le32(x));

  if (pv->cfg == NULL)
    return synopsys_usbdev_stack_event(pv);


  if (pv->cfg->type == DEV_USBDEV_UNCONFIGURE)
    {
      for (uint8_t i = 0; i < CONFIG_DRIVER_USB_SYNOPSYS_EP_COUNT + 1; i++)
        {
          tr = pv->tri[i];
          if (tr)
            {
              tr->error = -EPIPE;
              synopsys_usbdev_end_in_transfer(pv, tr);
            }
          tr = pv->tro[i];
          if (tr) 
            {
              tr->error = -EPIPE;
              synopsys_usbdev_end_out_transfer(pv, tr);
            }
        }
      pv->ctrl &= 1;
    }
#if (CONFIG_USBDEV_MAX_ALTERNATE_COUNT > 0)
  else if (pv->cfg->type == DEV_USBDEV_CHANGE_INTERFACE)
    {
      struct dev_usbdev_interface_cfg_s *itf = pv->cfg->itf + DEV_USBDEV_ITF_DISABLE;
      /* Cancel pending transfer */
      USBDEV_FOREACH_ENDPOINT(itf->i, itf->epi, itf->epo,
        {
         if (usbdev_is_endpoint_in(epdesc))
           {
             tr = pv->tri[epaddr];
             /* Terminate transfer */
             tr->error = -EAGAIN;
             synopsys_usbdev_end_in_transfer(pv, tr);
           }
  
         if (usbdev_is_endpoint_out(epdesc)) 
           {
             tr = pv->tro[epaddr];
             /* Terminate transfer */
             tr->error = -EAGAIN;
             synopsys_usbdev_end_out_transfer(pv, tr);
           }
         if (USB_GET_EDP_TYPE(epdesc) == USB_EP_CONTROL)
             pv->ctrl &= ~(1 << epaddr);
        });

        /* Enable new interface */
      itf = pv->cfg->itf + DEV_USBDEV_ITF_ENABLE;
       synopsys_usbdev_edp_activation(pv, itf);
   
      USBDEV_FOREACH_ENDPOINT(itf->i, itf->epi, itf->epo,
        {
          if (usbdev_is_endpoint_in(epdesc))
            {
              assert(pv->tri[epaddr] == NULL);
              synopsys_usbdev_flush_tx_fifo(pv, epaddr);
            }
          if (usbdev_is_endpoint_out(epdesc))
            assert(pv->tro[epaddr] == NULL);
        });
    }
#endif

  if (cb)
    {
      /* Callback */
      usbdev_stack_config_done(&pv->usbdev_ctx);
      pv->cfg = NULL;
    }
}

static bool_t synopsys_usbdev_set_dis(struct synopsys_usbdev_private_s *pv, bool_t cb) 
{
  /* Disable endpoint */
  pv->dstate = SYNOPSYS_USBDEV_WAIT_DIS;

  if (pv->cfg == NULL)
  /* Disable after an event */
    {
      if (pv->tri[0])
        synopsys_usbdev_set_flag(pv, 1, SYNOPSYS_USB_DIEPCTL_EPDIS, 0);
      goto done;
    }

  if (pv->cfg->type == DEV_USBDEV_UNCONFIGURE)
    {
      for (uint8_t i = 0; i < CONFIG_DRIVER_USB_SYNOPSYS_EP_COUNT + 1; i++)
        {
          if (pv->tri[i]) 
            synopsys_usbdev_set_flag(pv, 1, SYNOPSYS_USB_DIEPCTL_EPDIS, i);

          if (pv->tro[i]) 
            synopsys_usbdev_set_flag(pv, 0, SYNOPSYS_USB_DOEPCTL_EPDIS, i);
        }
    }
#if (CONFIG_USBDEV_MAX_ALTERNATE_COUNT > 0)
  else if (pv->cfg->type == DEV_USBDEV_CHANGE_INTERFACE)
    {
      struct dev_usbdev_interface_cfg_s *itf = pv->cfg->itf + DEV_USBDEV_ITF_DISABLE;

      USBDEV_FOREACH_ENDPOINT(itf->i, itf->epi, itf->epo,
        {
          if (usbdev_is_endpoint_in(epdesc) && pv->tri[epaddr]) 
            synopsys_usbdev_set_flag(pv, 1, SYNOPSYS_USB_DIEPCTL_EPDIS, epaddr);
          if (usbdev_is_endpoint_out(epdesc) && pv->tro[epaddr]) 
            synopsys_usbdev_set_flag(pv, 0, SYNOPSYS_USB_DOEPCTL_EPDIS, epaddr);
        });
    }
#endif

done:
  if (pv->imask || pv->omask)
    return 0;
  
  synopsys_usbdev_end_transfer(pv, cb);
  return 1;
}

static void synopsys_usbdev_set_gout_nak(struct synopsys_usbdev_private_s *pv) 
{
  uint32_t x = cpu_mem_read_32(pv->addr + SYNOPSYS_USB_DCTL_ADDR);
  x |= SYNOPSYS_USB_DCTL_SGOUTNAK;
  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DCTL_ADDR, x);
  pv->omask = 1;
}

static bool_t synopsys_usbdev_set_nak(struct synopsys_usbdev_private_s *pv, bool_t cb)
{

  pv->imask = pv->omask = 0;
  pv->dstate = SYNOPSYS_USBDEV_WAIT_SNAK;

  if (pv->cfg == NULL)
  /* Disable after an event */
    {
      pv->pevent = 1;
      if (pv->tri[0])
        synopsys_usbdev_set_flag(pv, 1, SYNOPSYS_USB_DIEPCTL_SNAK, 0);
      if (pv->tro[0])
        synopsys_usbdev_set_gout_nak(pv);
      goto done;
    }

  if (pv->cfg->type == DEV_USBDEV_UNCONFIGURE)
    {
      /* Set IN NAK  */
      for (uint8_t i = 1; i < CONFIG_DRIVER_USB_SYNOPSYS_EP_COUNT + 1; i++)
        {
          if (pv->tri[i]) 
            synopsys_usbdev_set_flag(pv, 1, SYNOPSYS_USB_DIEPCTL_SNAK, i);
        }
      /* Global OUT NAK */
      for (uint8_t i = 1; i < CONFIG_DRIVER_USB_SYNOPSYS_EP_COUNT + 1; i++)
        {
          if (pv->tro[i])
           {
             synopsys_usbdev_set_gout_nak(pv);
             break;
           }
        }
    }
#if (CONFIG_USBDEV_MAX_ALTERNATE_COUNT > 0)
  else if (pv->cfg->type == DEV_USBDEV_CHANGE_INTERFACE)
    {
      struct dev_usbdev_interface_cfg_s *itf = pv->cfg->itf + DEV_USBDEV_ITF_DISABLE;
      /* Set IN NAK  */
      USBDEV_FOREACH_ENDPOINT(itf->i, itf->epi, itf->epo,
       {
         if (usbdev_is_endpoint_in(epdesc) && pv->tri[epaddr]) 
           synopsys_usbdev_set_flag(pv, 1, SYNOPSYS_USB_DIEPCTL_SNAK, epaddr);
       });
      /* Global OUT NAK */
      USBDEV_FOREACH_ENDPOINT(itf->i, itf->epi, itf->epo,
       {
         if (usbdev_is_endpoint_out(epdesc) && pv->tro[epaddr]) 
           {
             synopsys_usbdev_set_gout_nak(pv);
             break;
           }
       });
    }
#endif
  
done:
  if (pv->imask || pv->omask)
    return 0;

  return synopsys_usbdev_set_dis(pv, cb);
}

void synopsys_usbdev_abort_ep0(struct synopsys_usbdev_private_s *pv)
{
  struct dev_usbdev_request_s *tr = pv->tro[0];

  /* Terminate on-going transfer */
  if (tr && tr->type == DEV_USBDEV_EVENT)
    synopsys_usbdev_stack_event(pv);
  else if (pv->tri[0] || tr)
    {
      if (!pv->pevent)
        synopsys_usbdev_set_nak(pv, 0);
    }
}

error_t synopsys_usbdev_config(struct device_s *dev,
                               struct synopsys_usbdev_private_s *pv,
                               struct dev_usbdev_config_s * cfg)
{
  error_t err = 0;

  switch (cfg->type)
    {
    case DEV_USBDEV_SET_ADDRESS:
      {
        uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + SYNOPSYS_USB_DCFG_ADDR));
        SYNOPSYS_USB_DCFG_DEVADDR_SET(x, cfg->addr);
        cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DCFG_ADDR, endian_le32(x));
        break;
      }
    case DEV_USBDEV_UNCONFIGURE:
      pv->cfg = cfg;
      if (dev->start_count == 0)
        {
          /* Unconfiguration after a stop */
          synopsys_usbdev_end_transfer(pv, 0);
          pv->cfg = NULL;
          break;
        }
      /* Start reconfiguration/unconfiguration */
      if (!synopsys_usbdev_set_nak(pv, 0))
        err = -EAGAIN;
      else
        pv->cfg = NULL;
      break;

#if (CONFIG_USBDEV_MAX_ALTERNATE_COUNT > 0)
    case DEV_USBDEV_CHANGE_INTERFACE:
      pv->cfg = cfg;
      /* Start reconfiguration/unconfiguration */
      if (!synopsys_usbdev_set_nak(pv, 0))
        err = -EAGAIN;
      else
        pv->cfg = NULL;
      break;
#endif

    case DEV_USBDEV_CONFIGURE:
      /* Apply configuration */
      err = synopsys_usbdev_configure(pv, cfg);
      break;

    default:
      err = -EINVAL;
      break;
    }
  return err;
}

error_t synopsys_usbdev_transfer(struct synopsys_usbdev_private_s *pv, 
                                 struct dev_usbdev_request_s * tr)
{
//  printk("R%d%d\n", tr->ep, tr->type);
  error_t err = -EAGAIN;

  /* Some event are pending */

  if (tr->ep == 0 && pv->event)
    {
      err = -EIO;
      tr->event = pv->event;
      pv->event = 0;
      goto end;
    }

  /* Start transfer */

  bool_t in = dev_usbdev_get_transfer_dir(tr) == USB_DEVICE_TO_HOST;

  if (in)
    assert(pv->tri[tr->ep] == NULL);
  else
    assert(pv->tro[tr->ep] == NULL);

  switch (tr->type)
    {
    case DEV_USBDEV_CTRL_SETUP:
      {
        uint32_t x;
        /* Initialise setup packet received count */
        x = SYNOPSYS_USB_DOEPTSIZE_RXDPIDSUPCNT_SHIFT_VAL(1);
        cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DOEPTSIZE_ADDR(tr->ep), endian_le32(x));
        /* Program DMA address */
        cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DOEPDMAADDR_ADDR(tr->ep), endian_le32((uint32_t)tr->data));
        /* Enable DMA copy */
        x = endian_le32(cpu_mem_read_32(pv->addr + SYNOPSYS_USB_DOEPCTL_ADDR(tr->ep)));
        x |= SYNOPSYS_USB_DOEPCTL_EPENA;            
        cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DOEPCTL_ADDR(tr->ep), endian_le32(x));
        break;
      }

    case DEV_USBDEV_CTRL_STATUS_IN:
    case DEV_USBDEV_DATA_IN:
    case DEV_USBDEV_PARTIAL_DATA_IN:
      synopsys_usbdev_send_data(pv, tr);
      break;

    case DEV_USBDEV_CTRL_STATUS_IN_STALL:
    case DEV_USBDEV_DATA_IN_STALL:
      synopsys_usbdev_set_flag(pv, 1, SYNOPSYS_USB_DIEPCTL_STALL, tr->ep);
      err = 0;
      goto end;

    case DEV_USBDEV_CTRL_STATUS_OUT:
    case DEV_USBDEV_DATA_OUT:
    case DEV_USBDEV_PARTIAL_DATA_OUT:
      synopsys_usbdev_get_data(pv, tr);
      break;

    case DEV_USBDEV_CTRL_STATUS_OUT_STALL:
    case DEV_USBDEV_DATA_OUT_STALL:
      synopsys_usbdev_set_flag(pv, 0, SYNOPSYS_USB_DOEPCTL_STALL, tr->ep);
      err = 0;
      goto end;

    case DEV_USBDEV_EVENT:
      break;
    }

  if (in)
    pv->tri[tr->ep] = tr;
  else
    pv->tro[tr->ep] = tr;
  
end:
  return err;
}

static void synopsys_usbdev_edpin_irq(struct synopsys_usbdev_private_s *pv, uint8_t idx)
{
  uint32_t irq = endian_le32(cpu_mem_read_32(pv->addr + SYNOPSYS_USB_DIEPINT_ADDR(idx)));

  bool_t drop = irq & SYNOPSYS_USB_DIEPINT_PKTDRPSTS ? 1 : 0;

  irq &= endian_le32(cpu_mem_read_32(pv->addr + SYNOPSYS_USB_DIEPMSK_ADDR));

  if (drop)
    irq |= SYNOPSYS_USB_DIEPINT_PKTDRPSTS;

  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DIEPINT_ADDR(idx), irq);

  if (!irq)
    return;

  if ((irq & SYNOPSYS_USB_DIEPINT_INEPNAKEFF) && 
      (pv->dstate == SYNOPSYS_USBDEV_WAIT_SNAK))
    {
      pv->imask &= ~(1 << idx);
      if (!pv->imask && !pv->omask)
        synopsys_usbdev_set_dis(pv, 1);
    }
    
  if ((irq & SYNOPSYS_USB_DIEPINT_EPDISBLD) &&
      (pv->dstate == SYNOPSYS_USBDEV_WAIT_DIS))
    {
      pv->imask &= ~(1 << idx);
      if (!pv->imask && !pv->omask)
        synopsys_usbdev_end_transfer(pv, 1);
    }

  if (irq & (SYNOPSYS_USB_DIEPINT_TIMEOUT | SYNOPSYS_USB_DIEPINT_XFERCOMPL))
    {
      struct dev_usbdev_request_s *tr;
  
      tr = pv->tri[idx];
  
      if (tr == NULL)
        return;
  
      assert((tr->type == DEV_USBDEV_DATA_IN) ||
             (tr->type == DEV_USBDEV_PARTIAL_DATA_IN) ||
             (tr->type == DEV_USBDEV_CTRL_STATUS_IN));
     
      if (drop)
        tr->error = -ETIMEDOUT;
  
      if (irq & SYNOPSYS_USB_DIEPINT_TIMEOUT)
        return;
  
      /* Terminate transfer */
      synopsys_usbdev_end_in_transfer(pv, tr);
    }
}

static void synopsys_usbdev_edpout_irq(struct synopsys_usbdev_private_s *pv, uint8_t idx)
{
  uint32_t irq = endian_le32(cpu_mem_read_32(pv->addr + SYNOPSYS_USB_DOEPINT_ADDR(idx)));

  bool_t drop = irq & SYNOPSYS_USB_DOEPINT_PKTDRPSTS ? 1 : 0;

  irq &= endian_le32(cpu_mem_read_32(pv->addr + SYNOPSYS_USB_DOEPMSK_ADDR));

  if (drop)
    irq |= SYNOPSYS_USB_DOEPINT_PKTDRPSTS;

  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DOEPINT_ADDR(idx), irq);

  if (!irq)
    return;

  if ((irq & SYNOPSYS_USB_DOEPINT_EPDISBLD) &&
      (pv->dstate == SYNOPSYS_USBDEV_WAIT_DIS))
    {
      pv->omask &= ~(1 << idx);
      if (!pv->imask && !pv->omask)
      /* All endpoint are disabled */
        synopsys_usbdev_end_transfer(pv, 1);
    }

  if (irq & (SYNOPSYS_USB_DOEPINT_XFERCOMPL | SYNOPSYS_USB_DOEPINT_SETUP))
    {
       struct dev_usbdev_request_s *tr;
       
       tr = pv->tro[idx];
       
       if (tr == NULL)
         return;

       if (irq & SYNOPSYS_USB_DOEPINT_SETUP)
         assert(tr->type == DEV_USBDEV_CTRL_SETUP);

       if (irq & SYNOPSYS_USB_DOEPINT_XFERCOMPL)
         assert((tr->type == DEV_USBDEV_DATA_OUT)   ||
                (tr->type == DEV_USBDEV_PARTIAL_DATA_OUT) ||
                (tr->type == DEV_USBDEV_CTRL_STATUS_OUT)); 

      if (drop)
        tr->error = -ETIMEDOUT;
       
       /* Terminate transfer */
       synopsys_usbdev_end_out_transfer(pv, tr);
    }
}

void synopsys_usbdev_event(struct synopsys_usbdev_private_s *pv, uint8_t event)
{
//  printk("e %d %d\n", event, pv->connected);
  switch (event)
    {
    case USBDEV_EVENT_DISCONNECT:
      if (!pv->connected)
        return;
      /* Erase previous event */
      pv->event = event;
      pv->connected = 0;
      break;
    case USBDEV_EVENT_IDLE:
    case USBDEV_EVENT_WAKEUP:
    case USBDEV_EVENT_RESET:
      if (!pv->connected)
        return;
      pv->event |= event;
      break;
    case USBDEV_EVENT_CONNECT:
      if (pv->connected)
        return;
      pv->connected = 1;
      pv->event |= event;
      break;
    case USBDEV_EVENT_STOP:
      pv->event |= event;
      break;
    };
}

bool_t synopsys_usb_irq(struct synopsys_usbdev_private_s *pv)
{
  uint32_t irq, epirq, x;
  uint8_t epidx;

  irq = endian_le32(cpu_mem_read_32(pv->addr + SYNOPSYS_USB_GINTSTS_ADDR));
  irq &= endian_le32(cpu_mem_read_32(pv->addr + SYNOPSYS_USB_GINTMSK_ADDR));

  if (!irq)
    return 0;

  pv->ctrl = 1;

  /* Reset on usb bus */
  if (irq & SYNOPSYS_USB_GINTSTS_USBRST || irq & SYNOPSYS_USB_GINTSTS_RESETDET)
    {
      x = endian_le32(SYNOPSYS_USB_GINTSTS_USBRST | SYNOPSYS_USB_GINTSTS_RESETDET);
      cpu_mem_write_32(pv->addr + SYNOPSYS_USB_GINTSTS_ADDR, x);
      synopsys_usbdev_event(pv, USBDEV_EVENT_RESET);
    }
  /* Controller has entered suspend state */
  if (irq & SYNOPSYS_USB_GINTSTS_USBSUSP)
    {
      x = endian_le32(SYNOPSYS_USB_GINTSTS_USBSUSP);
      cpu_mem_write_32(pv->addr + SYNOPSYS_USB_GINTSTS_ADDR, x);
      synopsys_usbdev_event(pv, USBDEV_EVENT_IDLE);
    }
  /* Wake up during suspended state */
  if (irq & SYNOPSYS_USB_GINTSTS_WKUPINT)
    {
      x = endian_le32(SYNOPSYS_USB_GINTSTS_WKUPINT);
      cpu_mem_write_32(pv->addr + SYNOPSYS_USB_GINTSTS_ADDR, x);
      synopsys_usbdev_event(pv, USBDEV_EVENT_WAKEUP);
    }
  /* End of reset on usb bus */
  if (irq & SYNOPSYS_USB_GINTSTS_ENUMDONE)
    {
      x = endian_le32(SYNOPSYS_USB_GINTSTS_ENUMDONE);
      cpu_mem_write_32(pv->addr + SYNOPSYS_USB_GINTSTS_ADDR, x);
      x = endian_le32(cpu_mem_read_32(pv->addr + SYNOPSYS_USB_DSTS_ADDR));
      x = SYNOPSYS_USB_DSTS_ENUMSPD_GET(x);
      /* Full speed device */
      assert(x == 3);
    }

  /* Global OUT NAK */
  if ((irq & SYNOPSYS_USB_GINTSTS_GOUTNAKEFF) &&
      (pv->dstate == SYNOPSYS_USBDEV_WAIT_SNAK))
    {
      pv->omask = 0;
      /* Disable interrupt */
      x = endian_le32(cpu_mem_read_32(pv->addr + SYNOPSYS_USB_GINTMSK_ADDR));
      x &= ~SYNOPSYS_USB_GINTMSK_GOUTNAKEFFMSK;
      cpu_mem_write_32(pv->addr + SYNOPSYS_USB_GINTMSK_ADDR, endian_le32(x));
      /* Go on disabling endpoints */
      if (!pv->imask && !pv->omask)
        synopsys_usbdev_set_dis(pv, 1);
    }
  

  if (irq & (SYNOPSYS_USB_GINTSTS_IEPINT | SYNOPSYS_USB_GINTSTS_OEPINT))
    {
      epirq = endian_le32(cpu_mem_read_32(pv->addr + SYNOPSYS_USB_DAINT_ADDR));
      epirq &= endian_le32(cpu_mem_read_32(pv->addr + SYNOPSYS_USB_DAINTMSK_ADDR));
      
      /* Endpoint in interrupt */
      uint16_t irqio = epirq & 0xFFFF; 
      epidx = 0;
      while(irqio)
        {
          epidx = __builtin_ctz(irqio);
          synopsys_usbdev_edpin_irq(pv, epidx);
          irqio ^= 1 << epidx; 
        }

      /* Endpoint out interrupt */
      irqio = (epirq >> 16) & 0xFFFF; 
      epidx = 0;
      while(irqio)
        {
          epidx = __builtin_ctz(irqio);
          synopsys_usbdev_edpout_irq(pv, epidx);
          irqio ^= 1 << epidx; 
        }
    }

  return 1;
}

void synopsys_usb_reset_device(struct synopsys_usbdev_private_s *pv)
{
  uint32_t x;

  /* Reset the core */
  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_GRSTCTL_ADDR, endian_le32(SYNOPSYS_USB_GRSTCTL_CSFTRST));
  /* Wait for AHB master state machine to be IDLE. */
  while (1)
    {
      x = cpu_mem_read_32(pv->addr + SYNOPSYS_USB_GRSTCTL_ADDR);
      if (x & SYNOPSYS_USB_GRSTCTL_AHBIDLE)
        break;
    }

  /* Default state is not visible */
  x = SYNOPSYS_USB_DCTL_SFTDISCON;
  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DCTL_ADDR, endian_le32(x));

  /* Reset Device Address */
  x = cpu_mem_read_32(pv->addr + SYNOPSYS_USB_DCFG_ADDR);
  x |= SYNOPSYS_USB_DCFG_DEVSPD_SHIFT_VAL(48MHZ);
  x &= ~SYNOPSYS_USB_DCFG_DEVADDR;
  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DCFG_ADDR, x);
  
  /* Set to device mode */

  x = SYNOPSYS_USB_GUSBCFG_FORCEDEVMODE;
  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_GUSBCFG_ADDR, x);
  /* Wait 25 ms here */

  for (uint8_t i = 0; i <= CONFIG_DRIVER_USB_SYNOPSYS_EP_COUNT; i++)
    cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DIEPTXF_ADDR(i), 0);

  for (uint8_t i = 0; i <= CONFIG_DRIVER_USB_SYNOPSYS_EP_COUNT; i++)
    {
      cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DIEPCTL_ADDR(i), 0);
      cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DIEPINT_ADDR(i), SYNOPSYS_USB_DIEPINT_MASK);
      cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DOEPCTL_ADDR(i), 0);
      cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DOEPINT_ADDR(i), SYNOPSYS_USB_DOEPINT_MASK);
    }

  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DAINTMSK_ADDR, 0);
  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DOEPMSK_ADDR, 0);
  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DIEPMSK_ADDR, 0);
  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_GINTMSK_ADDR, 0);

  /* Enable DMA mode and global interrupt */

  x = SYNOPSYS_USB_GAHBCFG_DMAEN |
      SYNOPSYS_USB_GAHBCFG_GLBLINTRMSK |
      SYNOPSYS_USB_GAHBCFG_PTXFEMPLVL |
      SYNOPSYS_USB_GAHBCFG_HBSTLEN_SHIFT_VAL(INCR);

  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_GAHBCFG_ADDR, endian_le32(x));

  /* Enable global interrupt */

  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_GINTSTS_ADDR, SYNOPSYS_USB_GINTSTS_MASK);

  x = SYNOPSYS_USB_GINTMSK_WKUPINTMSK | SYNOPSYS_USB_GINTMSK_DISCONNINTMSK |
      SYNOPSYS_USB_GINTMSK_RESETDETMSK | SYNOPSYS_USB_GINTMSK_OEPINTMSK |
      SYNOPSYS_USB_GINTMSK_IEPINTMSK | SYNOPSYS_USB_GINTMSK_USBRSTMSK |
//      SYNOPSYS_USB_GINTMSK_USBSUSPMSK | 
      SYNOPSYS_USB_GINTMSK_ENUMDONEMSK |
      SYNOPSYS_USB_GINTMSK_GOUTNAKEFFMSK;

  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_GINTMSK_ADDR, x);

  x = SYNOPSYS_USB_DIEPMSK_XFERCOMPLMSK | SYNOPSYS_USB_DIEPMSK_TIMEOUTMSK |
      SYNOPSYS_USB_DIEPMSK_INEPNAKEFFMSK | SYNOPSYS_USB_DIEPMSK_EPDISBLDMSK;
  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DIEPMSK_ADDR, x);

  x = SYNOPSYS_USB_DOEPMSK_XFERCOMPLMSK | SYNOPSYS_USB_DOEPMSK_SETUPMSK |
      SYNOPSYS_USB_DOEPMSK_EPDISBLDMSK;
  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DOEPMSK_ADDR, x);
}

void synopsys_usbdev_start(struct synopsys_usbdev_private_s *pv)
{
  /* Show device to host */
  uint32_t x = endian_le32(SYNOPSYS_USB_DCTL_CGNPINNAK |
                           SYNOPSYS_USB_DCTL_CGOUTNAK |
                           SYNOPSYS_USB_DCTL_IGNRFRMNUM);

  cpu_mem_write_32(pv->addr + SYNOPSYS_USB_DCTL_ADDR, x);
}

DEV_USBDEV_ALLOC(synopsys_usbdev_alloc)
{
  return mem_alloc_align(align_pow2_up(size, 4), 4, mem_scope_sys);
}

DEV_USBDEV_FREE(synopsys_usbdev_free)
{
  mem_free(ptr);
}


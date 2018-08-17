#include <string.h>
#include <stdio.h>

#include <hexo/types.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/iomux.h>

#include <mutek/printk.h>
#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>

#include <device/class/usbdev.h>
#include <device/usb/usb.h>
#include <arch/pic32/usb.h>

#define PIC32_USB_RAM_SIZE (1 << 12)
#define PIC32_USB_FIFO_EP0_SIZE    64
#define PIC32_USB_DMA_THRESHOLD    8
#define PIC32_USB_DMA_IDLE_MSK     (1 << 7)
#define PIC32_USB_NEW_ADDRESS_MSK  (1 << 7)

static uintptr_t pic32_dma_vir_to_phys(uintptr_t addr)
{
  return (addr & 0x20000000) ? addr - 0xA0000000 : addr - 0x80000000;
}

DRIVER_PV(struct pic32_usbdev_private_s
{
  struct dev_usbdev_context_s usbdev_ctx;
  /* usb address */
  uintptr_t addr;
  /* Interrupt end-point */
  struct dev_irq_src_s irq_eps[2];
  /* Endpoint */
  struct usbdev_endpoint_s epi[CONFIG_DRIVER_PIC32_USBDEV_EP_COUNT];
  struct usbdev_endpoint_s epo[CONFIG_DRIVER_PIC32_USBDEV_EP_COUNT];
  /* Transfers */
  uint16_t sout[CONFIG_DRIVER_PIC32_USBDEV_EP_COUNT + 1];
  uint16_t sin[CONFIG_DRIVER_PIC32_USBDEV_EP_COUNT + 1];
  struct dev_usbdev_rq_s *tr0;
  struct dev_usbdev_rq_s *tri[CONFIG_DRIVER_PIC32_USBDEV_EP_COUNT];
  struct dev_usbdev_rq_s *tro[CONFIG_DRIVER_PIC32_USBDEV_EP_COUNT];
  /* Dma busy mask */
  uint8_t dma_busy;
  uint8_t event;
  /* New address to configure */
  uint8_t dev_addr;
  uint8_t prx;
  bool_t pstatus;
  bool_t connect;
  bool_t status_stage;
});

static DEV_USBDEV_ENDPOINT(pic32_usbdev_endpoint)
{
  struct device_s *dev = ctx->dev;
  struct pic32_usbdev_private_s *pv = dev->drv_pv;

  if (address > CONFIG_DRIVER_PIC32_USBDEV_EP_COUNT)
    return NULL;

  struct usbdev_endpoint_s * ep;

  if (dir == USB_EP_OUT)
    ep = pv->epo + address - 1;
  else
    ep = pv->epi + address - 1;

  return ep;
}

static void pic32_usbdev_select_ep(struct pic32_usbdev_private_s *pv, uint8_t i)
{
  cpu_mem_write_8(pv->addr + PIC32_USB_CS3_ADDR + 2, i);
}

static void pic32_usbdev_enable_irq(struct pic32_usbdev_private_s *pv, uint8_t idx, bool_t rx)
{
  uint32_t addr = pv->addr;
  uint8_t x;

  addr += rx ? PIC32_USB_CS2_ADDR : PIC32_USB_CS1_ADDR + 2;
  x = cpu_mem_read_8(addr);
  x |= 1 << idx;
  cpu_mem_write_8(addr, x);
}

static void pic32_usbdev_set_ep_fifo(struct pic32_usbdev_private_s *pv, uint8_t idx, bool_t rx)
{
  uint32_t addr;
  
  if (idx == 0)
    return;

  /* Each fifo is 256 byte long */
  uint8_t s = 5;
  /* Set fifo size */
  addr = pv->addr + PIC32_USB_OTG_ADDR;
  addr += rx ? 3 : 2;
  cpu_mem_write_8(addr, s);

  /* Endpoint 0 fifo is 64 byte long and is not configurable */
  uint16_t a = 64;
  a += (((idx - 1) << 1) + rx) << 8;
  a >>= 3;
  /* Set fifo address */
  addr = pv->addr + PIC32_USB_FIFOA_ADDR;
  addr += rx ? 2 : 0;
  cpu_mem_write_16(addr, endian_le16(a));
}

static error_t pic32_usbdev_configure_ep(struct pic32_usbdev_private_s *pv,
                                          const struct usb_endpoint_descriptor_s *desc,
                                          dev_usbdev_ep_addr_t addr)
{
  /* Select endpoint */
  pic32_usbdev_select_ep(pv, addr);

  uint32_t iecs0 = endian_le32(cpu_mem_read_32(pv->addr + PIC32_USB_IECS0_ADDR));
  uint32_t iecs1 = endian_le32(cpu_mem_read_32(pv->addr + PIC32_USB_IECS1_ADDR));
  uint32_t iecs2 = endian_le32(cpu_mem_read_32(pv->addr + PIC32_USB_IECS2_ADDR));
  uint32_t iecs3 = endian_le32(cpu_mem_read_32(pv->addr + PIC32_USB_IECS3_ADDR));

  if (usb_ep_type_get(desc) == USB_EP_CONTROL)
    return -ENOTSUP;

  if (usb_ep_dir_get(desc) == USB_EP_IN)
    {
      if (addr > CONFIG_DRIVER_PIC32_USBDEV_EP_COUNT)
        return -ENOTSUP;

      /* Set FIFO address and size */
      pic32_usbdev_set_ep_fifo(pv, addr, 0);

      /* CS0 */

      iecs0 = PIC32_USB_IENCS0_CLRDT;

      if (usb_ep_type_get(desc) == USB_EP_ISOCHRONOUS)
        iecs0 |= PIC32_USB_IENCS0_ISO;
      else
        iecs0 &= ~PIC32_USB_IENCS0_ISO;

      PIC32_USB_IENCS0_TXMAXP_SET(iecs0, usb_ep_mps_get(desc));

      /* Enable TX interrupt */
      pic32_usbdev_enable_irq(pv, addr, 0);
    }
  else
    {
      if (addr > CONFIG_DRIVER_PIC32_USBDEV_EP_COUNT)
        return -ENOTSUP;

      /* CS1 */

      iecs1 = PIC32_USB_IECS1_CLRDT;

      if (usb_ep_type_get(desc) == USB_EP_ISOCHRONOUS)
        iecs1 |= PIC32_USB_IECS1_ISO;
      else
        iecs1 &= ~PIC32_USB_IECS1_ISO;

      PIC32_USB_IECS1_RXMAXP_SET(iecs1, usb_ep_mps_get(desc));

      /* Set FIFO address and size */
      pic32_usbdev_set_ep_fifo(pv, addr, 1);
      /* Enable RX interrupt */
      pic32_usbdev_enable_irq(pv, addr, 1);
    }

  cpu_mem_write_32(pv->addr + PIC32_USB_IECS0_ADDR, endian_le32(iecs0));
  cpu_mem_write_32(pv->addr + PIC32_USB_IECS1_ADDR, endian_le32(iecs1));
  cpu_mem_write_32(pv->addr + PIC32_USB_IECS2_ADDR, endian_le32(iecs2));
  cpu_mem_write_32(pv->addr + PIC32_USB_IECS3_ADDR, endian_le32(iecs3));
  
  return 0;
}

static error_t pic32_usbdev_configure(struct pic32_usbdev_private_s *pv,
                                      struct dev_usbdev_config_s *cfg)
{
  if (cfg->intf == NULL)
  /* Enable interrupt for endpoint 0 */
    {
      pic32_usbdev_enable_irq(pv, 0, 0);
      return 0;
    }

  struct dev_usbdev_interface_cfg_s *icfg;

  for (uint8_t i = 0; i< CONFIG_USBDEV_MAX_INTERFACE_COUNT; i++)
    {
      icfg = cfg->intf + i;

      if (icfg->i == NULL)
        break;

      error_t err;
  
      USBDEV_FOREACH_ENDPOINT(icfg->i, icfg->epi, icfg->epo,
        {
          err = pic32_usbdev_configure_ep(pv, epdesc, epaddr);
          if (err)
           return err;
        });
    }
  return 0;
}

static void pic32_usbdev_transfer_done(struct pic32_usbdev_private_s *pv, uint8_t idx, bool_t in)
{
  struct dev_usbdev_rq_s **tra;
  struct dev_usbdev_rq_s *tr;
  
  tra = idx ? (in ? &pv->tri[idx - 1] : &pv->tro[idx - 1]) : &pv->tr0;
  tr = *tra;
  *tra = NULL;

  usbdev_stack_request_done(&pv->usbdev_ctx, tr);
}

static void pic32_usbdev_unconfigure(struct pic32_usbdev_private_s *pv)
{
  struct dev_usbdev_rq_s * tr;

  for (uint8_t i = 0; i < CONFIG_DRIVER_PIC32_USBDEV_EP_COUNT; i++)
    {
      tr = pv->tro[i];
      if (tr)
        {
          tr->error = -EPIPE;
          pic32_usbdev_transfer_done(pv, i + 1, 0);
        }
    }
  for (uint8_t i = 0; i < CONFIG_DRIVER_PIC32_USBDEV_EP_COUNT; i++)
    {
      tr = pv->tri[i];
      if (tr)
        {
          tr->error = -EPIPE;
          pic32_usbdev_transfer_done(pv, i + 1, 1);
        }
    }
}

static DEV_USBDEV_CONFIG(pic32_usbdev_config)
{
  struct device_s *dev = ctx->dev;
  struct pic32_usbdev_private_s *pv = dev->drv_pv;
  error_t err = 0;

  switch (cfg->type)
    {
    case DEV_USBDEV_SET_ADDRESS:
      /* Set address must be applied after current transaction */
      pv->dev_addr = cfg->addr | PIC32_USB_NEW_ADDRESS_MSK;
      break;

    case DEV_USBDEV_UNCONFIGURE:
      pic32_usbdev_unconfigure(pv);
      break;

    case DEV_USBDEV_CONFIGURE:
      /* Apply configuration */
      err = pic32_usbdev_configure(pv, cfg);
      break;

    default:
      err = -EINVAL;
      break;
    }

  return err;
}

/* This function selects an idle DMA channel among all DMA channel's. Returns
 * 0 if all channel are busy */

static uint8_t pic32_usbdev_select_dma(struct pic32_usbdev_private_s *pv)
{
  uint8_t dma_idle = ~pv->dma_busy;

  assert(dma_idle);

  return (PIC32_USB_DMA_IDLE_MSK | bit_ctz(dma_idle));
}

static void pic32_usbdev_dma_start(struct pic32_usbdev_private_s *pv,
                                   struct dev_usbdev_rq_s *tr,
                                   size_t size, uint8_t dma_idx)
{
  /* Retrieve physical address */
  uintptr_t addr = pic32_dma_vir_to_phys((uintptr_t)tr->data);

  /* Retrieve DMA index */
  uint8_t idx = dma_idx ^ PIC32_USB_DMA_IDLE_MSK;

  cpu_mem_write_32(pv->addr + PIC32_USB_DMAA_ADDR(idx), endian_le32((uint32_t)addr));
  cpu_mem_write_32(pv->addr + PIC32_USB_DMAN_ADDR(idx), endian_le32(size));

  uint16_t dmac = PIC32_USB_DMAC_EN |
                  PIC32_USB_DMAC_IE |
                  PIC32_USB_DMAC_EP(tr->ep) |
                  PIC32_USB_DMAC_BRSTM(3);

  if (tr->type == DEV_USBDEV_DATA_IN ||
      tr->type == DEV_USBDEV_PARTIAL_DATA_IN)
    dmac |= PIC32_USB_DMAC_DIR;

  cpu_mem_write_16(pv->addr + PIC32_USB_DMAC_ADDR(idx), endian_le16(dmac));

  /* Tag DMA as busy */
  pv->dma_busy |= 1 << idx;
}

static error_t pic32_usbdev_read(struct pic32_usbdev_private_s *pv, struct dev_usbdev_rq_s *tr);

static void pic32_usbdev_set_iecs0(struct pic32_usbdev_private_s *pv, uint32_t mask)
{
  uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + PIC32_USB_IECS0_ADDR));
  x |= mask;
  cpu_mem_write_32(pv->addr + PIC32_USB_IECS0_ADDR, endian_le32(x));
}

static error_t pic32_usbdev_read_fifo_done(struct pic32_usbdev_private_s *pv, struct dev_usbdev_rq_s *tr,
                                           uint32_t size)
{
  uint32_t x;

  tr->data += size;
  tr->size -= size;

  if (tr->ep == 0)
    {
      switch (tr->type)
        {
        case DEV_USBDEV_CTRL_SETUP:
          assert(tr->size == 0);
          if (usb_setup_length_get(tr->data) == 0)
            pv->status_stage = 1;
          return 0;
        case DEV_USBDEV_PARTIAL_DATA_OUT:
        case DEV_USBDEV_DATA_OUT:
          tr->rem -= size;
          if (!tr->rem)
            pv->status_stage = 1;
          if (!tr->rem || !tr->size ||
              tr->type == DEV_USBDEV_PARTIAL_DATA_OUT)
            return 0;
          /* Wait next packet */
          pic32_usbdev_set_iecs0(pv, PIC32_USB_IECS0_SVCRPR);
          break;
        default:
          assert(0);
        }
    }
  else
    {
      assert(tr->type == DEV_USBDEV_PARTIAL_DATA_OUT ||
             tr->type == DEV_USBDEV_DATA_OUT);

      /* Clear RX packet ready flag */
      x = endian_le32(cpu_mem_read_32(pv->addr + PIC32_USB_IECS1_ADDR));
      x &= ~PIC32_USB_IECS1_RXPKTRDY;
      cpu_mem_write_32(pv->addr + PIC32_USB_IECS1_ADDR, endian_le32(x));

      x = endian_le32(cpu_mem_read_32(pv->addr + PIC32_USB_IECS1_ADDR));
      x = PIC32_USB_IECS1_RXMAXP_GET(x);

      if (tr->size == 0 || size % x ||
          tr->type == DEV_USBDEV_PARTIAL_DATA_OUT)
        return 0;
    }
  return -EAGAIN;
}

static error_t pic32_usbdev_read(struct pic32_usbdev_private_s *pv, struct dev_usbdev_rq_s *tr)
{
  uint32_t size = endian_le32(cpu_mem_read_32(pv->addr + PIC32_USB_IECS2_ADDR));
  size = PIC32_USB_IECS2_RXCNT_GET(size);

  pv->prx &= ~(1 << tr->ep);

  if (size > tr->size)
    size = tr->size;

  pv->sout[tr->ep] = size;

  /* Retrieve an idle DMA */
  uint8_t idx = pic32_usbdev_select_dma(pv);
  if (size > PIC32_USB_DMA_THRESHOLD && tr->ep)
    /* Use DMA to transfer */
    {
      pic32_usbdev_dma_start(pv, tr, size, idx);
      return -EAGAIN;
    }

  /* No use of DMA */
  for (uint16_t i = 0; i<size; i++)
    ((uint8_t *)tr->data)[i] = cpu_mem_read_8(pv->addr + PIC32_USB_FIFO_ADDR(tr->ep) + i%4);

  return pic32_usbdev_read_fifo_done(pv, tr, size);
}

static bool_t pic32_usbdev_data_in_done(struct pic32_usbdev_private_s *pv,
                                        struct dev_usbdev_rq_s *tr)
{
  bool_t ret = 0;

  if (pv->sin[0] % usbdev_stack_get_ep0_mps(&pv->usbdev_ctx))
    ret = 1;

  if (!pv->sin[0])
  /* Zero length packet */
    ret = 1;

  if (!(tr->rem - pv->sin[0]))
    ret = 1;

  return ret;
}

static void pic32_usbdev_write_fifo_done(struct pic32_usbdev_private_s *pv,
                                         struct dev_usbdev_rq_s *tr)
{
  assert(tr != NULL);
  assert(tr->type == DEV_USBDEV_PARTIAL_DATA_IN ||
         tr->type == DEV_USBDEV_DATA_IN);

  /* Packet is ready */
  uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + PIC32_USB_IECS0_ADDR));

  if (tr->ep == 0)
    {
      x |= PIC32_USB_IECS0_TXPKTRDY;
      if (pic32_usbdev_data_in_done(pv, tr))
        x |= PIC32_USB_IECS0_DATAEND;
    }
  else
    x |= PIC32_USB_IENCS0_TXPKTRDY;

  cpu_mem_write_32(pv->addr + PIC32_USB_IECS0_ADDR, endian_le32(x));
}

static DEV_IRQ_SRC_PROCESS(pic32_usb_dma_irq)
{
  struct device_s *dev = ep->base.dev;
  struct pic32_usbdev_private_s *pv = dev->drv_pv;
  struct dev_usbdev_rq_s *tr;

  lock_spin(&dev->lock);

  uint32_t irq = endian_le32(cpu_mem_read_32(pv->addr + PIC32_USB_DMAINT_ADDR));

  uint8_t dmaidx = 0;

  while(irq)
    {
      dmaidx = bit_ctz(irq);
      irq ^= 1 << dmaidx;

      /* Release DMA channel */
      pv->dma_busy ^= 1 << dmaidx;

      /* Get DMA status */
      uint32_t c = endian_le32(cpu_mem_read_32(pv->addr + PIC32_USB_DMAC_ADDR(dmaidx)));
      uint32_t n = endian_le32(cpu_mem_read_32(pv->addr + PIC32_USB_DMAN_ADDR(dmaidx)));

      if (c & PIC32_USB_DMAC_ERR || n)
        {
          cpu_mem_write_32(pv->addr + PIC32_USB_DMAC_ADDR(dmaidx), endian_le32(PIC32_USB_DMAC_ERR));
          printk("USB DMA irq error \n");
        }

      /* Retrieve endpoint number and select it */
      uint8_t epidx = PIC32_USB_DMAC_EP_GET(c); 
      pic32_usbdev_select_ep(pv, epidx);

      if (c & PIC32_USB_DMAC_DIR)
        {
          tr = epidx ? pv->tri[epidx - 1] : pv->tr0;
          if (tr == NULL)
            continue;

          pic32_usbdev_write_fifo_done(pv, tr);
        }
      else
        {
          tr = epidx ? pv->tro[epidx - 1] : pv->tr0;
          if (tr == NULL)
            continue;

          if (pic32_usbdev_read_fifo_done(pv, tr, pv->sout[tr->ep]) == 0)
            pic32_usbdev_transfer_done(pv, epidx, 0); 
        }
    }
  lock_release(&dev->lock);
}

static void pic32_usbdev_send_data(struct pic32_usbdev_private_s *pv, struct dev_usbdev_rq_s *tr)
{
  uint32_t size, trsize, x;

  trsize = tr->size;

  if (tr->ep)
    {
      x = endian_le32(cpu_mem_read_32(pv->addr + PIC32_USB_IECS0_ADDR));
      size = PIC32_USB_IENCS0_TXMAXP_GET(x);
    }
  else
    size = PIC32_USB_FIFO_EP0_SIZE;

  if (size > trsize)
    size = trsize;

  pv->sin[tr->ep] = size;

  /* Retrieve an idle DMA */
  uint8_t idx = pic32_usbdev_select_dma(pv);

  if (size > PIC32_USB_DMA_THRESHOLD)
  /* Use DMA to transfer */
    pic32_usbdev_dma_start(pv, tr, size, idx);
  else
  /* No use of DMA or zero length packet */
    {
      for (uint16_t i = 0; i<size; i++)
        cpu_mem_write_8(pv->addr + PIC32_USB_FIFO_ADDR(tr->ep) + i%4, ((uint8_t *)tr->data)[i]);

      pic32_usbdev_write_fifo_done(pv, tr);
    }
}

static void pic32_usbdev_stall_ep(struct pic32_usbdev_private_s *pv, uint8_t idx)
{
  pv->status_stage = 0;
  pv->pstatus = 0;
  /* Send stall */
  uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + PIC32_USB_IECS0_ADDR));
  x |= (idx ? PIC32_USB_IENCS0_SENDSTALL : PIC32_USB_IECS0_SENDSTALL); 
  cpu_mem_write_32(pv->addr + PIC32_USB_IECS0_ADDR, endian_le32(x));
}


static DEV_USBDEV_REQUEST(pic32_usbdev_transfer)
{
  struct device_s *dev = ctx->dev;
  struct pic32_usbdev_private_s *pv = dev->drv_pv;
  struct dev_usbdev_rq_s **tra;

  error_t err = -EAGAIN;

  /* Some event are pending */

  if (tr->ep == 0 && pv->event)
    {
      err = -EIO;
      tr->event = pv->event;
      pv->event = 0;
      goto end;
    }

  bool_t in = dev_usbdev_get_transfer_dir(tr) == USB_DEVICE_TO_HOST;

  tra = tr->ep ? (in ? &pv->tri[tr->ep - 1] : &pv->tro[tr->ep - 1]) : &pv->tr0;
  /* No transfer must be on-going */
  assert(*tra == NULL);

  /* Select endpoint */
  pic32_usbdev_select_ep(pv, tr->ep);

  switch (tr->type)
    {
    case DEV_USBDEV_PARTIAL_DATA_OUT:
    case DEV_USBDEV_DATA_OUT:
      if (!tr->ep)
        pic32_usbdev_set_iecs0(pv, PIC32_USB_IECS0_SVCRPR);
    case DEV_USBDEV_CTRL_SETUP:
      /* Try read */
      if (pv->prx & (1 << tr->ep))
        err = pic32_usbdev_read(pv, tr);
      break;

    case DEV_USBDEV_PARTIAL_DATA_IN:
    case DEV_USBDEV_DATA_IN:
      if (!tr->ep)
        pic32_usbdev_set_iecs0(pv, PIC32_USB_IECS0_SVCRPR);
      /* Write data */
      pic32_usbdev_send_data(pv, tr);
      break;
  
    case DEV_USBDEV_DATA_IN_STALL:
    case DEV_USBDEV_CTRL_STATUS_IN_STALL:
      if (!tr->ep)
        pic32_usbdev_set_iecs0(pv, PIC32_USB_IECS0_SVCRPR);
    case DEV_USBDEV_DATA_OUT_STALL:
    case DEV_USBDEV_CTRL_STATUS_OUT_STALL:
      pic32_usbdev_stall_ep(pv, tr->ep);
      err = 0;
      break;

    case DEV_USBDEV_CTRL_STATUS_IN:
      assert(pv->pstatus == 0);
      pic32_usbdev_set_iecs0(pv, PIC32_USB_IECS0_SVCRPR |
                                 PIC32_USB_IECS0_DATAEND);
    case DEV_USBDEV_CTRL_STATUS_OUT:
      assert(tr->ep == 0);
      assert(pv->status_stage);
      if (pv->pstatus)
        {
          pv->status_stage = 0;
          pv->pstatus = 0;
          err = 0;
        }
      break;

    default:
      break;
    }

  if (err == -EAGAIN)
    *tra = tr;

end:
  return err;
}

static void pic32_usb_reset_device(struct device_s *dev)
{
  struct pic32_usbdev_private_s *pv = dev->drv_pv;
  uint32_t x;

  /* Reset Core */

  /* Default state is not visible */
  cpu_mem_write_32(pv->addr + PIC32_USB_CS0_ADDR, 0);

  /* Clear pending interrupt */

  cpu_mem_write_32(pv->addr + PIC32_USB_CS1_ADDR, 0);
  cpu_mem_write_32(pv->addr + PIC32_USB_CS2_ADDR, 0);

  x = endian_le32(cpu_mem_read_32(pv->addr + PIC32_USB_CS0_ADDR));
  x = endian_le32(cpu_mem_read_32(pv->addr + PIC32_USB_CS1_ADDR));
  x = endian_le32(cpu_mem_read_32(pv->addr + PIC32_USB_CS2_ADDR));

  /* Enable event interrupts */

  x = PIC32_USB_CS2_SUSPIE |
      PIC32_USB_CS2_RESUMEIE |
      PIC32_USB_CS2_RESETIE |
      PIC32_USB_CS2_SESSRQIE |
      PIC32_USB_CS2_DISCONIE;

  cpu_mem_write_32(pv->addr + PIC32_USB_CS2_ADDR, endian_le32(x));
}

static void pic32_usbdev_data_in_end(struct pic32_usbdev_private_s *pv, uint8_t idx)
{
  struct dev_usbdev_rq_s *tr = idx ? pv->tri[idx - 1] : pv->tr0;

  if (tr == NULL)
    return;

  switch (tr->type)
    {
    case DEV_USBDEV_DATA_IN:
    case DEV_USBDEV_PARTIAL_DATA_IN:
      if (idx == 0)
        {
          if (pic32_usbdev_data_in_done(pv, tr))
            pv->status_stage = 1;
  
          tr->rem -= pv->sin[tr->ep];
        }

      tr->data += pv->sin[idx];
      tr->size -= pv->sin[idx];

      if (tr->size == 0 ||
          tr->type == DEV_USBDEV_PARTIAL_DATA_IN)
        goto done;
        
      pic32_usbdev_send_data(pv, tr);
      break;

    default:
      break;
    }
  return;

done:
  if (idx)
    {
      uint32_t x;
      /* Set drop bit for isochronous transfert */
      x = endian_le32(cpu_mem_read_32(pv->addr + PIC32_USB_IECS0_ADDR));
      if ((x & PIC32_USB_IENCS0_ISO) && (x & PIC32_USB_IENCS0_UNDERRUN))
        {
          tr->error = -ETIMEDOUT;
          x &= ~PIC32_USB_IENCS0_UNDERRUN;
          cpu_mem_write_32(pv->addr + PIC32_USB_IECS0_ADDR, endian_le32(x));
        }
    }
  pic32_usbdev_transfer_done(pv, idx, 1);
}

static void pic32_usbdev_status(struct pic32_usbdev_private_s *pv)
/* Status has been sent */
{
  struct dev_usbdev_rq_s *tr = pv->tr0;

  pv->pstatus = 1;

  if (pv->dev_addr & PIC32_USB_NEW_ADDRESS_MSK)
    {
      pv->dev_addr ^= PIC32_USB_NEW_ADDRESS_MSK;
      cpu_mem_write_8(pv->addr + PIC32_USB_CS0_ADDR, pv->dev_addr);
    }

  if (tr == NULL)
    return;

  if (tr->type == DEV_USBDEV_CTRL_STATUS_IN ||
      tr->type == DEV_USBDEV_CTRL_STATUS_OUT)
    {
      pv->pstatus = 0;
      pv->status_stage = 0;
      pic32_usbdev_transfer_done(pv, 0, 0);
    }
}

static void pic32_usbdev_epin_irq(struct pic32_usbdev_private_s *pv, uint8_t idx)
{
  /* Select endpoint */
  pic32_usbdev_select_ep(pv, idx);

  if (idx)
  /* Endpoint N IN interrupt */
    return pic32_usbdev_data_in_end(pv, idx);

  /* Endpoint 0 IN/OUT interrupt */

  struct dev_usbdev_rq_s *tr = pv->tr0;
  uint32_t irq = endian_le32(cpu_mem_read_32(pv->addr + PIC32_USB_IECS0_ADDR));
  
  if (irq & PIC32_USB_IECS0_SETUPEND) 
    /* Control transfer aborted */
    {
      printk("ABORT\n");
      /* Clear pending flag */
      cpu_mem_write_32(pv->addr + PIC32_USB_IECS0_ADDR, endian_le32(PIC32_USB_IECS0_SVCSETEND));
      if (tr)
        {
          tr->error = -EPIPE;
          goto done;
        }
    }

  if (pv->status_stage)
    pic32_usbdev_status(pv);

  if (irq & PIC32_USB_IECS0_RXPKTRDY)
    /* New packet available */
    {
      pv->prx |= 1;

      if (tr == NULL)
        return;

      switch (tr->type)
        {
        case DEV_USBDEV_CTRL_SETUP:
        case DEV_USBDEV_DATA_OUT:
        case DEV_USBDEV_PARTIAL_DATA_OUT:
          if (pic32_usbdev_read(pv, tr) == 0)
            goto done;
        default:
          break;
        }
    }
  else
    /* TX sent */
    return pic32_usbdev_data_in_end(pv, 0);

  return;

done:
  pic32_usbdev_transfer_done(pv, 0, 1);
}

static void pic32_usbdev_epout_irq(struct pic32_usbdev_private_s *pv, uint8_t idx)
{
  if (idx == 0)
    return;

  pv->prx |= 1 << idx;

  struct dev_usbdev_rq_s *tr = pv->tro[idx - 1];

  if (tr)
    {
      uint32_t x;
      /* Select endpoint */
      pic32_usbdev_select_ep(pv, idx);
      /* Set drop bit for isochronous transfert */
      x = endian_le32(cpu_mem_read_32(pv->addr + PIC32_USB_IECS1_ADDR));
      if ((x & PIC32_USB_IENCS0_ISO) && (x & PIC32_USB_IENCS0_UNDERRUN))
        {
          tr->error = -ETIMEDOUT;
          x &= ~PIC32_USB_IECS1_OVERRUN;
          cpu_mem_write_32(pv->addr + PIC32_USB_IECS1_ADDR, endian_le32(x));
        }
      /* Read received data */
      if (pic32_usbdev_read(pv, tr) == 0)
        pic32_usbdev_transfer_done(pv, idx, 0);
    }
}

static void pic32_usbdev_stack_event(struct pic32_usbdev_private_s *pv)
{
  struct dev_usbdev_rq_s *tr = pv->tr0;

  if (tr == NULL)
    return;

  tr->event = pv->event;
  tr->error = -EIO;
  pv->event = 0;
  pv->prx &= ~1;

  pic32_usbdev_transfer_done(pv, 0, 0);
}

static DEV_IRQ_SRC_PROCESS(pic32_usb_irq)
{
  struct device_s *dev = ep->base.dev;
  struct pic32_usbdev_private_s *pv = dev->drv_pv;

  uint32_t irqe, irqi, irqo;
  uint32_t irq0, irq1, irq2;

  uint8_t epidx;

  lock_spin(&dev->lock);

  while(1)
    {
      irq0 = endian_le32(cpu_mem_read_32(pv->addr + PIC32_USB_CS0_ADDR));
      irq1 = endian_le32(cpu_mem_read_32(pv->addr + PIC32_USB_CS1_ADDR));
      irq2 = endian_le32(cpu_mem_read_32(pv->addr + PIC32_USB_CS2_ADDR));

      irqe = irq2 & (irq2 >> 8) & (0xFF << 16);
      irqi = ((irq0 & irq1) >> 16) & 0xFF;
      irqo = (irq2 & irq1 & 0xFF);

      if(!irqe && !irqi && !irqo)
        break;

      /* Device disconnected */
      if (irqe & PIC32_USB_CS2_DISCONIF) 
        {
          if (pv->connect)
            {
              pv->event |= USBDEV_EVENT_DISCONNECT;
              pv->connect = 0;
            }
        }

      /* Device attached */
      if (irqe & PIC32_USB_CS2_SESSRQIF) 
        {
          pv->event |= USBDEV_EVENT_CONNECT;
          pv->connect = 1;
        }

      /* Reset on bus */
      if (irqe & PIC32_USB_CS2_RESETIF)
        {
          pv->event |= USBDEV_EVENT_RESET;
          if (pv->connect == 0)
            {
              pv->event |= USBDEV_EVENT_CONNECT;
              pv->connect = 1;
            }
        }

      /* Controller has entered suspend state */
//      if (irqe & PIC32_USB_CS2_SUSPIF)
//        pv->event |= USBDEV_EVENT_IDLE;

      /* Wake up during suspended state */
//      if (irqe & PIC32_USB_CS2_RESUMEIF)
//        pv->event |= USBDEV_EVENT_WAKEUP;

      /* Endpoint OUT interrupt */

      epidx = 0;

      while(irqo)
        {
           epidx = bit_ctz(irqo);
           pic32_usbdev_epout_irq(pv, epidx);
           irqo ^= 1 << epidx; 
        }

      /* Endpoint IN interrupt */

      epidx = 0;

      while(irqi)
        {
           epidx = bit_ctz(irqi);
           pic32_usbdev_epin_irq(pv, epidx);
           irqi ^= 1 << epidx; 
        }
    }
      
  if (pv->event)
    pic32_usbdev_stack_event(pv);

  lock_release(&dev->lock);
}


static DEV_USE(pic32_usbdev_use)
{
  struct device_accessor_s *accessor = param;
  uint32_t x;

  struct device_s *dev = accessor->dev;
  struct pic32_usbdev_private_s *pv = dev->drv_pv;

  switch (op)
    {
    case DEV_USE_START:
      if (dev->start_count == 0)
        {
          /* Show device to host */
          x = endian_le32(PIC32_USB_CS0_SOFTCON | PIC32_USB_CS0_HSEN); 
          cpu_mem_write_32(pv->addr + PIC32_USB_CS0_ADDR, x);
        }
      break;
    case DEV_USE_STOP:
      if (dev->start_count == 0)
        {
          pic32_usb_reset_device(dev);
          /* Inform stack and all services */
          pv->event |= USBDEV_EVENT_STOP;
        }
      break;
    default:
      return dev_use_generic(param, op);
    }

  if (pv->event)
    pic32_usbdev_stack_event(pv);

  return 0;
}

static DEV_USBDEV_ALLOC(pic32_usbdev_alloc)
{
  uintptr_t p = (uintptr_t)mem_alloc_align(align_pow2_up(size, 4), 4, mem_scope_sys);
  /* Buffer must be allocated in non cacheable area */
  return (void*)(p | 0x20000000);
}

static DEV_USBDEV_FREE(pic32_usbdev_free)
{
  mem_free(ptr);
}

static const struct dev_usbdev_driver_ops_s pic32_usbdev_ops_s = {
  .f_transfer = pic32_usbdev_transfer,
  .f_config = pic32_usbdev_config,
  .f_alloc = pic32_usbdev_alloc,
  .f_free = pic32_usbdev_free,
  .f_endpoint = pic32_usbdev_endpoint
};

static DEV_INIT(pic32_usbdev_init)
{
  struct pic32_usbdev_private_s  *pv;

  uintptr_t addr;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL))
    return -ENOENT;

  pv = mem_alloc(sizeof(struct pic32_usbdev_private_s), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  pv->addr = addr;
  dev->drv_pv = pv;

  /* Retrieve number of endpoint */
  uint32_t info = endian_le32(cpu_mem_read_32(addr + PIC32_USB_INFO_ADDR));

  if (PIC32_USB_INFO_RXENDPTS_GET(info) > CONFIG_DRIVER_PIC32_USBDEV_EP_COUNT)
    goto err_nep;

  if (PIC32_USB_INFO_TXENDPTS_GET(info) > CONFIG_DRIVER_PIC32_USBDEV_EP_COUNT)
    goto err_nep;

  pic32_usb_reset_device(dev);

  device_irq_source_init(dev, &pv->irq_eps[0], 1, &pic32_usb_irq);
  device_irq_source_init(dev, &pv->irq_eps[1], 1, &pic32_usb_dma_irq);

  if (device_irq_source_link(dev, pv->irq_eps, 2, -1))
    goto err_mem;

  uint16_t msk = ((1 << (CONFIG_DRIVER_PIC32_USBDEV_EP_COUNT + 1)) - 1);

  if (usbdev_stack_init(dev, &pv->usbdev_ctx, msk, msk, &pic32_usbdev_ops_s))
    goto err_irq;

  return 0;

err_nep:
  printk("Error CONFIG_DRIVER_PIC32_USBDEV_EP_COUNT > available endpoint count\n");
  goto err_mem;
err_irq:
  device_irq_source_unlink(dev, pv->irq_eps, 2);
err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(pic32_usbdev_cleanup)
{
  struct pic32_usbdev_private_s *pv = dev->drv_pv;

  if (usbdev_stack_cleanup(&pv->usbdev_ctx))
    return -EBUSY;

  device_irq_source_unlink(dev, pv->irq_eps, 2);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(pic32_usbdev_drv, 0, "pic32 USB", pic32_usbdev,
               DRIVER_USBDEV_METHODS(pic32_usbdev));


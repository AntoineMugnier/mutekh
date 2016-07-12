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

    Copyright Vincent DEFILIPPI <vincentdefilippi@gmail.com> (c) 2015
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/spi.h>
#include <device/class/timer.h>
#include <device/class/iomux.h>
#include <device/clock.h>

#include <arch/cc26xx/memory_map.h>
#include <arch/cc26xx/prcm.h>
#include <arch/cc26xx/ssi.h>

#define CC26XX_SPI_FIFO_SIZE 8

DRIVER_PV(struct cc26xx_spi_context_s
{
  uintptr_t                      addr;
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_src_s           irq_ep;
#endif
  struct dev_spi_ctrl_transfer_s *tr;
  uint_fast8_t                   fifo_lvl;

  struct dev_spi_ctrl_context_s    spi_ctrl_ctx;

  struct dev_freq_s              freq;
  uint32_t                       bit_rate;

#ifdef CONFIG_DEVICE_CLOCK
  struct dev_clock_sink_ep_s     clk_ep;
#endif

});

static void cc26xx_spi_update_rate(struct cc26xx_spi_context_s *pv)
{
  uint32_t div = (pv->freq.num) / (pv->freq.denom * pv->bit_rate);
  cpu_mem_write_32(pv->addr + CC26XX_SSI_CPSR_ADDR, div);
}

static DEV_SPI_CTRL_CONFIG(cc26xx_spi_config)
{
  struct device_s *dev = accessor->dev;
  struct cc26xx_spi_context_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->tr != NULL)
    err = -EBUSY;
  else
    {
      if (cfg->word_width < 4 || cfg->word_width > 16)
        err = -ENOTSUP;
      else if (cfg->miso_pol != DEV_SPI_ACTIVE_LOW
                || cfg->mosi_pol != DEV_SPI_ACTIVE_LOW)
        err = -ENOTSUP;
      else if (cfg->bit_order != DEV_SPI_MSB_FIRST)
        err = -ENOTSUP;
      else
        {
          uint32_t reg = cfg->word_width - 1;

          if (cfg->ck_mode == DEV_SPI_CK_MODE_2 || cfg->ck_mode == DEV_SPI_CK_MODE_3)
            reg |= CC26XX_SSI_CR0_SPO;

          if (cfg->ck_mode == DEV_SPI_CK_MODE_1 || cfg->ck_mode == DEV_SPI_CK_MODE_3)
            reg |= CC26XX_SSI_CR0_SPH;

          reg |= CC26XX_SSI_CR0_FRF(MOTOROLA_SPI);

          cpu_mem_write_32(pv->addr + CC26XX_SSI_CR0_ADDR, reg);

          if (pv->bit_rate != cfg->bit_rate)
            {
              pv->bit_rate = cfg->bit_rate;
              cc26xx_spi_update_rate(pv);
            }
        }
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static bool_t cc26xx_spi_transfer_tx(struct device_s *dev);

static bool_t cc26xx_spi_transfer_rx(struct device_s *dev)
{
  struct cc26xx_spi_context_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_transfer_s *tr = pv->tr;

  while (pv->fifo_lvl > 0)
    {
      /* Check if the rx fifo is empty or not*/
      uint32_t st = cpu_mem_read_32(pv->addr + CC26XX_SSI_SR_ADDR)
                      & CC26XX_SSI_SR_RNE;

      if (!st)
        return 0;           /* wait for more rx irq */

      uint32_t word = cpu_mem_read_32(pv->addr + CC26XX_SSI_DR_ADDR);

      pv->fifo_lvl--;

      if (tr->data.in == NULL)
        continue;

      switch (tr->data.in_width)
        {
        case 1:
          *(uint8_t*)tr->data.in = word;
          break;
        case 2:
          *(uint16_t*)tr->data.in = word;
          break;
        case 4:
          *(uint32_t*)tr->data.in = word;
          break;
        }

      tr->data.in = (void*)((uint8_t*)tr->data.in + tr->data.in_width);
    }

  if (tr->data.count > 0)
    return cc26xx_spi_transfer_tx(dev);

  pv->tr = NULL;

  return 1;
}

static bool_t cc26xx_spi_transfer_tx(struct device_s *dev)
{
  struct cc26xx_spi_context_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_transfer_s *tr = pv->tr;

  while (tr->data.count > 0 && pv->fifo_lvl < CC26XX_SPI_FIFO_SIZE)
    {
      uint32_t word = 0;
      switch (tr->data.out_width)
        {
        case 1:
          word = *(const uint8_t*)tr->data.out;
          break;
        case 2:
          word = *(const uint16_t*)tr->data.out;
          break;
        case 0:
        case 4:
          word = *(const uint32_t*)tr->data.out;
          break;
        }

      cpu_mem_write_32(pv->addr + CC26XX_SSI_DR_ADDR, word);

      tr->data.out = (const void*)((const uint8_t*)tr->data.out + tr->data.out_width);
      tr->data.count--;
      pv->fifo_lvl++;
    }

  return 0;
}


static DEV_IRQ_SRC_PROCESS(cc26xx_spi_irq)
{

  struct device_s *dev = ep->base.dev;
  struct cc26xx_spi_context_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  while (cpu_mem_read_32(pv->addr + CC26XX_SSI_MIS_ADDR) &
          (CC26XX_SSI_MIS_RXMIS | CC26XX_SSI_MIS_RTMIS))
    {
      struct dev_spi_ctrl_transfer_s *tr = pv->tr;

      if (tr != NULL && cc26xx_spi_transfer_rx(dev))
        kroutine_exec(&tr->kr);
    }

  lock_release(&dev->lock);
}

static DEV_SPI_CTRL_TRANSFER(cc26xx_spi_transfer)
{
  struct device_s *dev = accessor->dev;
  struct cc26xx_spi_context_s *pv = dev->drv_pv;
  bool_t done = 1;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->tr != NULL)
    tr->err = -EBUSY;
  else if (tr->cs_op != DEV_SPI_CS_NOP_NOP)
    tr->err = -ENOTSUP;
  else
    {
      assert(tr->data.count > 0);
      tr->err = 0;
      pv->tr = tr;
      pv->fifo_lvl = 0;
      cc26xx_spi_transfer_tx(dev);
      done = 0;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  if (done)
    kroutine_exec(&tr->kr);
}


#define cc26xx_spi_use dev_use_generic

static void power_domain_on(void)
{
  uint32_t reg;

  //peripheral power domain on (SSI1)
  reg = cpu_mem_read_32(CC26XX_PRCM_BASE + CC26XX_PRCM_PDCTL0PERIPH_ADDR);
  reg |= CC26XX_PRCM_PDCTL0PERIPH_ON;
  cpu_mem_write_32(CC26XX_PRCM_BASE + CC26XX_PRCM_PDCTL0PERIPH_ADDR, reg);

  //serial power domain on (SSI0)
  reg = cpu_mem_read_32(CC26XX_PRCM_BASE + CC26XX_PRCM_PDCTL0SERIAL_ADDR);
  reg |= CC26XX_PRCM_PDCTL0SERIAL_ON;
  cpu_mem_write_32(CC26XX_PRCM_BASE + CC26XX_PRCM_PDCTL0SERIAL_ADDR, reg);

  //waiting for power
  while (!(cpu_mem_read_32(CC26XX_PRCM_BASE + CC26XX_PRCM_PDSTAT0_ADDR) &
    CC26XX_PRCM_PDSTAT0_SERIAL_ON));
}

static void clk_enable(void)
{
  uint32_t reg;

  //SSI clock enable
  reg = cpu_mem_read_32(CC26XX_PRCM_BASE + CC26XX_PRCM_SSICLKGR_ADDR);
  reg |= CC26XX_PRCM_SSICLKGR_CLK_EN;
  cpu_mem_write_32(CC26XX_PRCM_BASE + CC26XX_PRCM_SSICLKGR_ADDR, reg);

  //loading clocks modif
  reg = CC26XX_PRCM_CLKLOADCTL_LOAD;
  cpu_mem_write_32(CC26XX_PRCM_NONBUF_BASE + CC26XX_PRCM_CLKLOADCTL_ADDR, reg);

  //waiting for clocks
  while (!(cpu_mem_read_32(CC26XX_PRCM_BASE + CC26XX_PRCM_CLKLOADCTL_ADDR) &
    CC26XX_PRCM_CLKLOADCTL_LOAD_DONE));
}

static DEV_INIT(cc26xx_spi_init)
{
  struct cc26xx_spi_context_s *pv;



  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  /* init irq endpoint */
  device_irq_source_init(dev, &pv->irq_ep, 1, &cc26xx_spi_irq);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, -1))
    goto err_mem;

  if (device_get_res_freq(dev, &pv->freq, 0))
    goto err_mem;

  /* init state */
  pv->tr = NULL;

#ifdef CONFIG_DEVICE_SPI_REQUEST
  if (dev_spi_context_init(dev, &pv->spi_ctrl_ctx))
    goto err_mem;
#endif

  power_domain_on();
  clk_enable();

  /* Disable the module*/
  cpu_mem_write_32(pv->addr + CC26XX_SSI_CR1_ADDR, 0);

  /* Set SPI mode with 8 bits data size */
  cpu_mem_write_32(pv->addr + CC26XX_SSI_CR0_ADDR, CC26XX_SSI_CR0_DSS(8_BIT)
                    | CC26XX_SSI_CR0_FRF(MOTOROLA_SPI));

  /* setup pinmux */
  if (device_iomux_setup(dev, ">clk <miso? >mosi? >cs?", NULL, NULL, NULL))
    goto err_mem;

  /* setup bit rate */
  pv->bit_rate = 1000000;
  cc26xx_spi_update_rate(pv);

  /* Clear interrupt flags */
  cpu_mem_write_32(pv->addr + CC26XX_SSI_ICR_ADDR, -1);

  /* Set interrupt masks */
  cpu_mem_write_32(pv->addr + CC26XX_SSI_IMSC_ADDR, CC26XX_SSI_IMSC_RXIM
                    | CC26XX_SSI_IMSC_RTIM);

  /* Enable the module */
  cpu_mem_write_32(pv->addr + CC26XX_SSI_CR1_ADDR,
                   CC26XX_SSI_CR1_SSE(SSI_ENABLED) | CC26XX_SSI_CR1_MS(MASTER));

  return 0;

 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(cc26xx_spi_cleanup)
{
  struct cc26xx_spi_context_s	*pv = dev->drv_pv;

  if (pv->tr != NULL)
    return -EBUSY;

  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  /* Clear interrupt flags */
  cpu_mem_write_32(pv->addr + CC26XX_SSI_ICR_ADDR, -1);

  /* Clear interrupt masks */
  cpu_mem_write_32(pv->addr + CC26XX_SSI_IMSC_ADDR, 0);

  /* Disable the module */
  cpu_mem_write_32(pv->addr + CC26XX_SSI_CR1_ADDR, 0);

#ifdef CONFIG_DEVICE_SPI_REQUEST
  dev_spi_context_cleanup(&pv->spi_ctrl_ctx);
#endif

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(cc26xx_spi_drv, 0, "CC26XX SPI", cc26xx_spi,
               DRIVER_SPI_CTRL_METHODS(cc26xx_spi));

DRIVER_REGISTER(cc26xx_spi_drv);


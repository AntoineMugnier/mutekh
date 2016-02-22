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
#include <device/class/char.h>
#include <device/class/iomux.h>

#include <arch/cc26xx/memory_map.h>
#include <arch/cc26xx/prcm.h>
#include <arch/cc26xx/ioc.h>
#include <arch/cc26xx/gpio.h>
#include <arch/cc26xx/uart.h>

/**************************************************************/

#if CONFIG_DRIVER_CHAR_CC26XX_UART_SWFIFO > 0
#include <gct_platform.h>
#include <gct/container_ring.h>
#define GCT_CONTAINER_ALGO_uart_fifo RING
GCT_CONTAINER_TYPES(uart_fifo, uint8_t, CONFIG_DRIVER_CHAR_CC26XX_UART_SWFIFO);
GCT_CONTAINER_FCNS(uart_fifo, static inline, uart_fifo,
                    init, destroy, isempty, isfull,
                    pop, pop_array, pushback, pushback_array);
#endif

struct cc26xx_uart_context_s
{
  uintptr_t addr;
  /* tty input request queue and char fifo */
  dev_request_queue_root_t    read_q;
  dev_request_queue_root_t    write_q;
#if CONFIG_DRIVER_CHAR_CC26XX_UART_SWFIFO > 0
  uart_fifo_root_t            read_fifo;
# ifdef CONFIG_DEVICE_IRQ
  uart_fifo_root_t            write_fifo;
# endif
#endif
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_src_s        irq_ep;
#endif
  uint32_t                    mode;

  bool_t                      read_started:1;
  bool_t                      write_started:1;
};

static void cc26xx_uart_try_read(struct device_s *dev)
{
  struct cc26xx_uart_context_s  *pv = dev->drv_pv;
  struct dev_char_rq_s          *rq;

  while ((rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->read_q))))
    {
      size_t size = 0;

#if CONFIG_DRIVER_CHAR_CC26XX_UART_SWFIFO > 0
      /* read as many characters as possible from driver fifo */
      size = uart_fifo_pop_array(&pv->read_fifo, rq->data, rq->size);
#endif

      /* try to read more characters directly from device fifo */
      while (size < rq->size && !(cpu_mem_read_32(pv->addr +
          CC26XX_UART_FR_ADDR) & endian_le32(CC26XX_UART_FR_RXFE)))
        {
          rq->data[size++] = endian_le32(cpu_mem_read_32(pv->addr +
            CC26XX_UART_DR_ADDR));
        }

      if (size)
        {
          rq->size -= size;
          rq->error = 0;
          rq->data += size;

          if (rq->type == _DEV_CHAR_PARTIAL || rq->size == 0)
            {
              dev_request_queue_pop(&pv->read_q);
              kroutine_exec(&rq->base.kr);
              continue;
            }
        }

#ifdef CONFIG_DEVICE_IRQ
    /* more data will be available on next interrupt */
    return;
#endif
    }

  pv->read_started = 0;

#ifdef CONFIG_DEVICE_IRQ
  /* Disable RX timeout IRQ */
  uint32_t reg = cpu_mem_read_32(pv->addr + CC26XX_UART_IMSC_ADDR);
  reg &= ~CC26XX_UART_IMSC_RTIM;
  cpu_mem_write_32(pv->addr + CC26XX_UART_IMSC_ADDR, reg);
  cpu_mem_write_32(pv->addr + CC26XX_UART_ICR_ADDR, CC26XX_UART_ICR_RTIC);
#endif

#ifdef CONFIG_DEVICE_IRQ
  /* copy more data from device fifo to driver fifo (or drop it) */
  /* if no request currently need it */
  while ((cpu_mem_read_32(pv->addr + CC26XX_UART_MIS_ADDR) &
      endian_le32(CC26XX_UART_MIS_RXMIS)))
    {
      __unused__ uint8_t c = endian_le32(cpu_mem_read_32(pv->addr +
        CC26XX_UART_DR_ADDR));
# if CONFIG_DRIVER_CHAR_CC26XX_UART_SWFIFO > 0
      if (uart_fifo_isfull(&pv->read_fifo))
        uart_fifo_pop(&pv->read_fifo);
      uart_fifo_pushback(&pv->read_fifo, c);
# endif
    }
#endif
}

static void cc26xx_uart_try_write(struct device_s *dev)
{
  struct cc26xx_uart_context_s *pv = dev->drv_pv;
  struct dev_char_rq_s         *rq;

#if defined(CONFIG_DEVICE_IRQ) && CONFIG_DRIVER_CHAR_CC26XX_UART_SWFIFO > 0
  /* try to write data from driver fifo to device */
  while (!uart_fifo_isempty(&pv->write_fifo) &&
      !(cpu_mem_read_32(pv->addr + CC26XX_UART_FR_ADDR) & CC26XX_UART_FR_TXFF))
    {
      uint8_t c = uart_fifo_pop(&pv->write_fifo);
      cpu_mem_write_8(pv->addr + CC26XX_UART_DR_ADDR, endian_le32(c));
    }
#endif

  while (1)
    {
      rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->write_q));
      if (rq == NULL)
        {
          pv->write_started = 0;
          break;
        }

      size_t size = 0;

#if defined(CONFIG_DEVICE_IRQ) && CONFIG_DRIVER_CHAR_CC26XX_UART_SWFIFO > 0
      if (uart_fifo_isempty(&pv->write_fifo))
        {
          /* driver fifo is empty, try to write as many characters as
             possible from request directly to the device fifo */
#endif
          while (size < rq->size)
            {
              if (cpu_mem_read_32(pv->addr + CC26XX_UART_FR_ADDR) &
                CC26XX_UART_FR_TXFF)
                break;

              cpu_mem_write_8(pv->addr + CC26XX_UART_DR_ADDR,
                endian_le32(rq->data[size++]));
            }

#if defined(CONFIG_DEVICE_IRQ) && CONFIG_DRIVER_CHAR_CC26XX_UART_SWFIFO > 0
        }

    /* some characters were not written to the device fifo, */
    /* push to driver fifo */
    if (size < rq->size)
      {
        size += uart_fifo_pushback_array(&pv->write_fifo, rq->data +
          size, rq->size - size);
      }
#endif

    if (size)
      {
        rq->size -= size;
        rq->data += size;
        rq->error = 0;

        if (rq->type == _DEV_CHAR_PARTIAL || rq->size == 0)
          {
            dev_request_queue_pop(&pv->write_q);
            kroutine_exec(&rq->base.kr);
            continue;
          }
      }

#ifdef CONFIG_DEVICE_IRQ
      /* more fifo space will be available on next interrupt */
      break;
#endif
    }

#ifdef CONFIG_DEVICE_IRQ
  /* enable TX interupt if it remains rq or if the sw fifo is not empty*/
  uint32_t reg = cpu_mem_read_32(pv->addr + CC26XX_UART_IMSC_ADDR);
  reg &= ~CC26XX_UART_IMSC_TXIM;
  if (rq)
    reg |= CC26XX_UART_IMSC_TXIM;
# if CONFIG_DRIVER_CHAR_CC26XX_UART_SWFIFO > 0
  if (!uart_fifo_isempty(&pv->write_fifo))
    reg |= CC26XX_UART_IMSC_TXIM;
# endif
  cpu_mem_write_32(pv->addr + CC26XX_UART_IMSC_ADDR, reg);
#endif
}


#define cc26xx_uart_cancel (dev_char_cancel_t*)&dev_driver_notsup_fcn

DEV_CHAR_REQUEST(cc26xx_uart_request)
{
  struct device_s               *dev = accessor->dev;
  struct cc26xx_uart_context_s  *pv = dev->drv_pv;
  error_t                       err = 0;

  assert(rq->size);

  LOCK_SPIN_IRQ(&dev->lock);

  switch (rq->type)
    {
      case DEV_CHAR_READ_PARTIAL:
      case DEV_CHAR_READ:
        {
          dev_request_queue_pushback(&pv->read_q, dev_char_rq_s_base(rq));

#ifdef CONFIG_DEVICE_IRQ
          /* Enable RX timeout IRQ */
          uint32_t reg = cpu_mem_read_32(pv->addr + CC26XX_UART_IMSC_ADDR);
          reg |= CC26XX_UART_IMSC_RTIM;
          cpu_mem_write_32(pv->addr + CC26XX_UART_IMSC_ADDR, reg);
#endif

          if (!pv->read_started)
            {
              pv->read_started = 1;
              cc26xx_uart_try_read(dev);
            }
          break;
        }

      case DEV_CHAR_WRITE_PARTIAL_FLUSH:
      case DEV_CHAR_WRITE_FLUSH:
      case DEV_CHAR_WRITE_PARTIAL:
      case DEV_CHAR_WRITE:
        {
          dev_request_queue_pushback(&pv->write_q, dev_char_rq_s_base(rq));
          if (!pv->write_started)
            {
              pv->write_started = 1;
              cc26xx_uart_try_write(dev);
            }
          break;
        }
      default:
        err = -ENOTSUP;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  if (err)
    {
      rq->error = err;
      kroutine_exec(&rq->base.kr);
    }
}

#ifdef CONFIG_DEVICE_IRQ
static DEV_IRQ_SRC_PROCESS(cc26xx_uart_irq)
{
  struct device_s               *dev = ep->base.dev;
  struct cc26xx_uart_context_s  *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  while (1)
    {
      uint32_t ir = cpu_mem_read_32(pv->addr + CC26XX_UART_MIS_ADDR);

      if (!ir)
        break;

      if (ir & endian_le32(CC26XX_UART_MIS_TXMIS))
        cc26xx_uart_try_write(dev);

      if (ir & endian_le32(CC26XX_UART_MIS_RXMIS | CC26XX_UART_MIS_RTMIS))
        cc26xx_uart_try_read(dev);
    }

  lock_release(&dev->lock);
}
#endif

static DEV_INIT(cc26xx_uart_init);
static DEV_CLEANUP(cc26xx_uart_cleanup);

#define cc26xx_uart_use dev_use_generic

DRIVER_DECLARE(cc26xx_uart_drv, 0, "cc26xx UART", cc26xx_uart,
               DRIVER_CHAR_METHODS(cc26xx_uart));

DRIVER_REGISTER(cc26xx_uart_drv,
                DEV_ENUM_FDTNAME_ENTRY("cc26xx_uart"));

static void power_domain_on(void)
{
  uint32_t reg;

  //peripheral power domain on
  reg = cpu_mem_read_32(CC26XX_PRCM_BASE + CC26XX_PRCM_PDCTL0PERIPH_ADDR);
  reg |= CC26XX_PRCM_PDCTL0PERIPH_ON;
  cpu_mem_write_32(CC26XX_PRCM_BASE + CC26XX_PRCM_PDCTL0PERIPH_ADDR, reg);

  //serial power domain on
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

  //gpio clock enable
  reg = cpu_mem_read_32(CC26XX_PRCM_BASE + CC26XX_PRCM_GPIOCLKGR_ADDR);
  reg |= CC26XX_PRCM_GPIOCLKGR_CLK_EN;
  cpu_mem_write_32(CC26XX_PRCM_BASE + CC26XX_PRCM_GPIOCLKGR_ADDR, reg);

  //uart clock enable
  reg = cpu_mem_read_32(CC26XX_PRCM_BASE + CC26XX_PRCM_UARTCLKGR_ADDR);
  reg |= CC26XX_PRCM_UARTCLKGR_CLK_EN;
  cpu_mem_write_32(CC26XX_PRCM_BASE + CC26XX_PRCM_UARTCLKGR_ADDR, reg);

  //loading clocks modif
  reg = CC26XX_PRCM_CLKLOADCTL_LOAD;
  cpu_mem_write_32(CC26XX_PRCM_NONBUF_BASE + CC26XX_PRCM_CLKLOADCTL_ADDR, reg);

  //waiting for clocks
  while (!(cpu_mem_read_32(CC26XX_PRCM_BASE + CC26XX_PRCM_CLKLOADCTL_ADDR) &
    CC26XX_PRCM_CLKLOADCTL_LOAD_DONE));
}

static DEV_INIT(cc26xx_uart_init)
{
  uint32_t                      reg;
  struct cc26xx_uart_context_s  *pv;

  device_mem_map( dev , 1 << 0 );

  cpu_interrupt_disable();

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  pv->read_started = pv->write_started = 0;

  power_domain_on();
  clk_enable();

  /* setup pinmux */
  if (device_iomux_setup(dev, ">tx <rx", NULL, NULL, NULL))
    goto err_mem;

  /* wait for empty fifo */
  while (!(cpu_mem_read_32(CC26XX_UART0_BASE + CC26XX_UART_FR_ADDR)
    & CC26XX_UART_FR_TXFE));

  //disable the uart
  cpu_mem_write_32(CC26XX_UART0_BASE + CC26XX_UART_CTL_ADDR, 0);

  dev_request_queue_init(&pv->read_q);
  dev_request_queue_init(&pv->write_q);

#if CONFIG_DRIVER_CHAR_CC26XX_UART_SWFIFO > 0
  uart_fifo_init(&pv->read_fifo);
#endif

#ifdef CONFIG_DEVICE_IRQ
# if CONFIG_DRIVER_CHAR_CC26XX_UART_SWFIFO > 0
  uart_fifo_init(&pv->write_fifo);
# endif

  device_irq_source_init(dev, &pv->irq_ep, 1, &cc26xx_uart_irq);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, 1))
    goto err_fifo;
#endif

#ifdef CONFIG_DEVICE_IRQ
  /* enable irqs */
  reg = CC26XX_UART_IMSC_RXIM;
  cpu_mem_write_32(pv->addr + CC26XX_UART_IMSC_ADDR, reg);
  cpu_mem_write_32(pv->addr + CC26XX_UART_ICR_ADDR, 0xffffffff);
#endif

  //write the integer portion of the BRD
  reg = 0;
  //CC26XX_UART_IBRD_DIVINT_SET(reg, 0x138); //9600
  CC26XX_UART_IBRD_DIVINT_SET(reg, 0x1A);  //115200
  cpu_mem_write_32(CC26XX_UART0_BASE + CC26XX_UART_IBRD_ADDR, reg);

  //write the fractionnal portion of the BRD
  reg = 0;
  //CC26XX_UART_FBRD_DIVFRAC_SET(reg, 0); //9600
  CC26XX_UART_FBRD_DIVFRAC_SET(reg, 3); //115200
  cpu_mem_write_32(CC26XX_UART0_BASE + CC26XX_UART_FBRD_ADDR, reg);

  //write the desired serial parameters
  reg = 0;
  CC26XX_UART_LCRH_WLEN_SET(reg, 8);
  CC26XX_UART_LCRH_PEN_SET(reg, DIS);
  reg &= ~CC26XX_UART_LCRH_STP2;
  CC26XX_UART_LCRH_FEN_SET(reg, EN);
  cpu_mem_write_32(CC26XX_UART0_BASE + CC26XX_UART_LCRH_ADDR, reg);

  //fifo IRQ levels
  reg = 0;
  CC26XX_UART_IFLS_TXSEL_SET(reg, 4_8);
  CC26XX_UART_IFLS_RXSEL_SET(reg, 7_8);
  cpu_mem_write_32(CC26XX_UART0_BASE + CC26XX_UART_IFLS_ADDR, reg);

  //enable the uart
  reg = 0;
  CC26XX_UART_CTL_TXE_SET(reg, EN);
  CC26XX_UART_CTL_RXE_SET(reg, EN);
  CC26XX_UART_CTL_UARTEN_SET(reg, EN);
  cpu_mem_write_32(CC26XX_UART0_BASE + CC26XX_UART_CTL_ADDR, reg);

  dev->drv = &cc26xx_uart_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

#ifdef CONFIG_DEVICE_IRQ
 err_fifo:
# if CONFIG_DRIVER_CHAR_CC26XX_UART_SWFIFO > 0
  uart_fifo_destroy(&pv->write_fifo);
  uart_fifo_destroy(&pv->read_fifo);
# endif
  dev_request_queue_destroy(&pv->read_q);
  dev_request_queue_destroy(&pv->write_q);
#endif
 err_mem:
  mem_free(pv);
  return -1;
}

DEV_CLEANUP(cc26xx_uart_cleanup)
{
  struct cc26xx_uart_context_s  *pv = dev->drv_pv;

  if (pv->read_started || pv->write_started)
    return -EBUSY;

#ifdef CONFIG_DEVICE_IRQ
  /* disable irqs */
  cpu_mem_write_32(pv->addr + CC26XX_UART_IMSC_ADDR, 0);
  cpu_mem_write_32(pv->addr + CC26XX_UART_ICR_ADDR, 0xffffffff);
#endif
  /* disable the uart */
  /* !used by the printk driver */
  //cpu_mem_write_32(CC26XX_UART0_BASE + CC26XX_UART_CTL_ADDR, 0);

#if CONFIG_DRIVER_CHAR_CC26XX_UART_SWFIFO > 0
# ifdef CONFIG_DEVICE_IRQ
  uart_fifo_destroy(&pv->write_fifo);
# endif
  uart_fifo_destroy(&pv->read_fifo);
#endif

  dev_request_queue_destroy(&pv->read_q);
  dev_request_queue_destroy(&pv->write_q);

  mem_free(pv);
  return 0;
}


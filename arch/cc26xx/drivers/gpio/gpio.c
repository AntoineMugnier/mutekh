#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/irq.h>
#include <device/class/gpio.h>
#include <device/class/icu.h>
#include <device/class/iomux.h>

#include <string.h>

#include <arch/cc26xx/memory_map.h>
#include <arch/cc26xx/ioc.h>
#include <arch/cc26xx/gpio.h>
#include <arch/cc26xx/prcm.h>
#include <arch/cc26xx/dio.h>


#define DIO_NB 32
#define GPIO_IRQ_DIO_COUNT 32

struct cc26xx_gpio_private_s
{
#ifdef CONFIG_DRIVER_CC26XX_GPIO_ICU
  struct dev_irq_sink_s sinks_ep[CONFIG_DRIVER_CC26XX_GPIO_IRQ_COUNT];
  struct dev_irq_src_s  src_ep;
  uint8_t               sinks_map[GPIO_IRQ_DIO_COUNT];
#endif
};

static void cc26xx_gpio_mode_reg(gpio_id_t io_first, gpio_id_t io_last,
              const uint8_t *mask, uint32_t iocfg_reg, bool_t oe)
{
  gpio_id_t len = io_last - io_first + 1;
  uint32_t  mask32 = endian_le32_na_load(mask);
  uint32_t  tmask32 = 1;
  uint32_t  old;
  uint32_t  irq_mask = CC26XX_IOC_IOCFG_EDGE_IRQ_EN | CC26XX_IOC_IOCFG_EDGE_DET;

  for (gpio_id_t i = 0; i < len; i++)
  {
    if (mask32 & tmask32)
      {
        old = cpu_mem_read_32(CC26XX_IOC_BASE +
          CC26XX_IOC_IOCFG_ADDR(io_first + i));
        cpu_mem_write_32(CC26XX_IOC_BASE + CC26XX_IOC_IOCFG_ADDR(io_first + i),
          (iocfg_reg & ~irq_mask) | (old & irq_mask));
      }
    tmask32 <<= 1;
  }

  uint32_t doe_reg;
  doe_reg = cpu_mem_read_32(CC26XX_GPIO_BASE + CC26XX_GPIO_DOE31_0_ADDR);
  mask32 = (mask32 & ((1 << len) - 1)) << io_first;
  doe_reg = oe ? doe_reg | mask32 : doe_reg & ~mask32;
  cpu_mem_write_32(CC26XX_GPIO_BASE + CC26XX_GPIO_DOE31_0_ADDR, doe_reg);
}


static void iocfg_set_irq(uint32_t *reg, enum dev_irq_sense_modes_e sense)
{
  switch (sense)
    {
      case DEV_IRQ_SENSE_NONE:
        CC26XX_IOC_IOCFG_EDGE_DET_SET(*reg, NONE);
        *reg &= ~CC26XX_IOC_IOCFG_EDGE_IRQ_EN;
        return;
      case DEV_IRQ_SENSE_FALLING_EDGE:
        CC26XX_IOC_IOCFG_EDGE_DET_SET(*reg, NEG);
        break;
      case DEV_IRQ_SENSE_RISING_EDGE:
        CC26XX_IOC_IOCFG_EDGE_DET_SET(*reg, POS);
        break;
      case DEV_IRQ_SENSE_ANY_EDGE:
        CC26XX_IOC_IOCFG_EDGE_DET_SET(*reg, BOTH);
        break;
      default:
        return;
    }
  *reg |= CC26XX_IOC_IOCFG_EDGE_IRQ_EN;
}

static void iocfg_set_mode(uint32_t *reg, bool_t *oe,
                            enum dev_pin_driving_e mode)
{
  *reg &= ~CC26XX_IOC_IOCFG_IE;               //Input disable
  CC26XX_IOC_IOCFG_PULL_CTL_SET(*reg, DIS);   //No pull
  CC26XX_IOC_IOCFG_IOMODE_SET(*reg, NORMAL);  //Normal mode
  *oe = 0;                                    //Output disable

  switch (mode)
    {
      case DEV_PIN_DISABLED:
        break;
      case DEV_PIN_PUSHPULL:
        *oe = 1;
        break;
      case DEV_PIN_INPUT:
      case DEV_PIN_INPUT_PULL:
        *reg |= CC26XX_IOC_IOCFG_IE;
        break;
      case DEV_PIN_INPUT_PULLUP:
        *reg |= CC26XX_IOC_IOCFG_IE;
        CC26XX_IOC_IOCFG_PULL_CTL_SET(*reg, UP);
        break;
      case DEV_PIN_INPUT_PULLDOWN:
        *reg |= CC26XX_IOC_IOCFG_IE;
        CC26XX_IOC_IOCFG_PULL_CTL_SET(*reg, DWN);
        break;
      case DEV_PIN_OPENDRAIN:
        *oe = 1;
        CC26XX_IOC_IOCFG_IOMODE_SET(*reg, OPENDR);
        break;
      case DEV_PIN_OPENSOURCE:
        *oe = 1;
        CC26XX_IOC_IOCFG_IOMODE_SET(*reg, OPENSRC);
        break;
      case DEV_PIN_OPENDRAIN_PULLUP:
        *oe = 1;
        CC26XX_IOC_IOCFG_IOMODE_SET(*reg, OPENDR);
        CC26XX_IOC_IOCFG_PULL_CTL_SET(*reg, UP);
        break;
      case DEV_PIN_OPENSOURCE_PULLDOWN:
        *oe = 1;
        CC26XX_IOC_IOCFG_IOMODE_SET(*reg, OPENSRC);
        CC26XX_IOC_IOCFG_PULL_CTL_SET(*reg, DWN);
        break;
      default:
        break;
    }
}

static void iocfg_set_mux(uint32_t *reg, iomux_mux_t mux)
{
  *reg &= ~CC26XX_IOC_IOCFG_PORT_ID;
  *reg |= mux;                               //IO mux
}

static error_t cc26xx_gpio_mode(gpio_id_t io_first, gpio_id_t io_last,
                                  const uint8_t *mask,
                                  enum dev_pin_driving_e mode,
                                  iomux_mux_t mux,
                                  enum dev_irq_sense_modes_e sense)
{
  uint32_t  iocfg_reg;
  bool_t    oe;

  iocfg_reg = 0;
  CC26XX_IOC_IOCFG_IOSTR_SET(iocfg_reg, AUTO);    //AUTO, MIN, MED, MAX
  CC26XX_IOC_IOCFG_IOCURR_SET(iocfg_reg, 2MA);    //2MA, 4MA, 4_8MA
  iocfg_reg &= ~CC26XX_IOC_IOCFG_SLEW_RED;        //Normal slew rate
  CC26XX_IOC_IOCFG_WU_CFG_SET(iocfg_reg, 0);      //No wake up
  iocfg_reg &= ~CC26XX_IOC_IOCFG_HYST_EN;         //Input Hysteresis disable

  iocfg_set_mode(&iocfg_reg, &oe, mode);
  iocfg_set_mux(&iocfg_reg, mux);
  iocfg_set_irq(&iocfg_reg, sense);

  cc26xx_gpio_mode_reg(io_first, io_last, mask, iocfg_reg, oe);

  return 0;
}

static DEV_GPIO_SET_MODE(cc26xx_gpio_set_mode)
{
  struct device_s *dev = gpio->dev;
  error_t         err;

  if (io_last >= DIO_NB || io_last < io_first)
    return -ERANGE;

  LOCK_SPIN_IRQ(&dev->lock);

  err = cc26xx_gpio_mode(io_first, io_last, mask, mode,
    CC26XX_IOC_IOCFG_PORT_ID_GPIO, DEV_IRQ_SENSE_NONE);
  if (err)
    return err;

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEV_GPIO_SET_OUTPUT(cc26xx_gpio_set_output)
{
  struct device_s *dev = gpio->dev;

  if (io_last >= DIO_NB || io_last < io_first)
    return -ERANGE;

  LOCK_SPIN_IRQ(&dev->lock);

  uint32_t len_mask32 = (1 << (io_last - io_first + 1)) - 1;
  uint32_t set_mask32 = endian_le32_na_load(set_mask);
  uint32_t clear_mask32 = endian_le32_na_load(clear_mask);

  set_mask32 = (set_mask32 & len_mask32) << io_first;
  clear_mask32 = (clear_mask32 & len_mask32) << io_first;

  uint32_t old = cpu_mem_read_32(CC26XX_GPIO_BASE + CC26XX_GPIO_DOUT31_0_ADDR);
  uint32_t new = set_mask32 ^ (old & (set_mask32 ^ clear_mask32));

  cpu_mem_write_32(CC26XX_GPIO_BASE + CC26XX_GPIO_DOUT31_0_ADDR, new);

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEV_GPIO_GET_INPUT(cc26xx_gpio_get_input)
{
  struct device_s *dev = gpio->dev;

  if (io_last >= DIO_NB || io_last < io_first)
    return -ERANGE;

  LOCK_SPIN_IRQ(&dev->lock);

  uint32_t len_mask32 = (1 << (io_last - io_first + 1)) - 1;
  uint32_t val = cpu_mem_read_32(CC26XX_GPIO_BASE + CC26XX_GPIO_DIN31_0_ADDR);
  val = (val >> io_first) & len_mask32;
  endian_le32_na_store(data, val);

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

#define cc26xx_gpio_use dev_use_generic
#define cc26xx_gpio_input_irq_range (dev_gpio_input_irq_range_t*)dev_driver_notsup_fcn


/******** GPIO iomux controller driver part *********************/

static DEV_IOMUX_SETUP(cc26xx_gpio_iomux_setup)
{
  struct device_s *dev = accessor->dev;
  error_t         err;

  if (io_id >= DIO_NB)
    return -ERANGE;

  LOCK_SPIN_IRQ(&dev->lock);

  err = cc26xx_gpio_mode(io_id, io_id, dev_gpio_mask1, dir, mux,
          DEV_IRQ_SENSE_NONE);
  if (err)
    return err;

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}


/******** GPIO irq controller driver part *********************/

#ifdef CONFIG_DRIVER_CC26XX_GPIO_ICU

static DEV_IRQ_SINK_UPDATE(cc26xx_gpio_icu_sink_update)
{
  uint_fast8_t dio_id = sink->icu_pv;
  uint32_t reg = cpu_mem_read_32(CC26XX_IOC_BASE +
    CC26XX_IOC_IOCFG_ADDR(dio_id));

  iocfg_set_irq(&reg, sense);

  cpu_mem_write_32(CC26XX_IOC_BASE + CC26XX_IOC_IOCFG_ADDR(dio_id), reg);
}

static DEV_ICU_GET_SINK(cc26xx_gpio_icu_get_sink)
{
  struct device_s *dev = accessor->dev;
  struct cc26xx_gpio_private_s *pv = dev->drv_pv;

  if (id >= GPIO_IRQ_DIO_COUNT)
    return NULL;

  for (uint_fast8_t i = 0; i < CONFIG_DRIVER_CC26XX_GPIO_IRQ_COUNT; i++)
    {
      if (!pv->sinks_ep[i].base.link_count)
        {
          pv->sinks_map[id] = i;
          pv->sinks_ep[i].icu_pv = id;
          return pv->sinks_ep + i;
        }
    }

  return NULL;
}

static DEV_ICU_LINK(cc26xx_gpio_icu_link)
{
  if (!route_mask || *bypass)
    return 0;

  uint_fast8_t dio_id = sink->icu_pv;

#ifdef CONFIG_DEVICE_IRQ_SHARING
  if (sink->base.link_count > 1)
    return 0;
#endif

  /* Change pin mode to input */
  cc26xx_gpio_mode(dio_id, dio_id, dev_gpio_mask1, DEV_PIN_INPUT,
    CC26XX_IOC_IOCFG_PORT_ID_GPIO, DEV_IRQ_SENSE_NONE);

  /* Clear interrupt */
  cpu_mem_write_32(CC26XX_GPIO_BASE + CC26XX_GPIO_EVFLAGS31_0_ADDR,
    CC26XX_GPIO_EVFLAGS31_0_DIO(dio_id));

  return 0;
}

static DEV_IRQ_SRC_PROCESS(cc26xx_gpio_source_process)
{
  struct cc26xx_gpio_private_s *pv = ep->base.dev->drv_pv;

  while (1)
    {
      uint32_t x = cpu_mem_read_32(CC26XX_GPIO_BASE +
        CC26XX_GPIO_EVFLAGS31_0_ADDR);

      if (!x)
        break;

      cpu_mem_write_32(CC26XX_GPIO_BASE + CC26XX_GPIO_EVFLAGS31_0_ADDR, x);
      x = endian_le32(x);

      while (x)
        {
          uint_fast8_t dio_id = __builtin_ctz(x);
          struct dev_irq_sink_s *sink = pv->sinks_ep + pv->sinks_map[dio_id];
          device_irq_sink_process(sink, 0);
          x ^= 1 << dio_id;
        }
    }
}

#endif


/******** GPIO generic driver part *********************/

static DEV_INIT(cc26xx_gpio_init);
static DEV_CLEANUP(cc26xx_gpio_cleanup);

#define cc26xx_gpio_request dev_gpio_request_async_to_sync

DRIVER_DECLARE(cc26xx_gpio_drv, 0, "CC26XX GPIO", cc26xx_gpio,
               DRIVER_GPIO_METHODS(cc26xx_gpio),
               DRIVER_IOMUX_METHODS(cc26xx_gpio_iomux),
#ifdef CONFIG_DRIVER_CC26XX_GPIO_ICU
               DRIVER_ICU_METHODS(cc26xx_gpio_icu)
#endif
);

DRIVER_REGISTER(cc26xx_gpio_drv);

static void power_domain_on(void)
{
  uint32_t reg;

  //peripheral power domain on
  reg = cpu_mem_read_32(CC26XX_PRCM_BASE + CC26XX_PRCM_PDCTL0PERIPH_ADDR);
  reg |= CC26XX_PRCM_PDCTL0PERIPH_ON;
  cpu_mem_write_32(CC26XX_PRCM_BASE + CC26XX_PRCM_PDCTL0PERIPH_ADDR, reg);

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

  //loading clocks modif
  reg = CC26XX_PRCM_CLKLOADCTL_LOAD;
  cpu_mem_write_32(CC26XX_PRCM_NONBUF_BASE + CC26XX_PRCM_CLKLOADCTL_ADDR, reg);

  //waiting for clocks
  while (!(cpu_mem_read_32(CC26XX_PRCM_BASE + CC26XX_PRCM_CLKLOADCTL_ADDR) &
    CC26XX_PRCM_CLKLOADCTL_LOAD_DONE));
}

static DEV_INIT(cc26xx_gpio_init)
{
  struct cc26xx_gpio_private_s *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  __unused__ uintptr_t addr = 0;
  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL) ||
    CC26XX_GPIO_BASE != addr)
    goto err_mem;

  power_domain_on();
  clk_enable();

#ifdef CONFIG_DRIVER_CC26XX_GPIO_ICU
  device_irq_source_init(dev, &pv->src_ep, 1,
                    &cc26xx_gpio_source_process);

  if (device_irq_source_link(dev, &pv->src_ep, 1, 1))
    goto err_mem;

  device_irq_sink_init(dev, pv->sinks_ep, CONFIG_DRIVER_CC26XX_GPIO_IRQ_COUNT,
                       &cc26xx_gpio_icu_sink_update,
                       DEV_IRQ_SENSE_FALLING_EDGE | DEV_IRQ_SENSE_RISING_EDGE |
                       DEV_IRQ_SENSE_ANY_EDGE);

  memset(pv->sinks_map, 0, sizeof(*(pv->sinks_map)));
#endif

  dev->drv = &cc26xx_gpio_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;
  return 0;

#ifdef CONFIG_DRIVER_CC26XX_GPIO_ICU
 err_unlink:
  device_irq_source_unlink(dev, &pv->src_ep, 1);
#endif
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(cc26xx_gpio_cleanup)
{
  struct cc26xx_gpio_private_s  *pv = dev->drv_pv;

#ifdef CONFIG_DRIVER_CC26XX_GPIO_ICU
  device_irq_source_unlink(dev, &pv->src_ep, 1);
  for (uint_fast8_t i = 0; i < CONFIG_DRIVER_CC26XX_GPIO_IRQ_COUNT; i++)
    {
      if (!pv->sinks_ep[i].base.link_count)
        {
          uint_fast8_t dio_id = pv->sinks_ep[i].icu_pv;
          uint32_t reg = cpu_mem_read_32(CC26XX_IOC_BASE +
            CC26XX_IOC_IOCFG_ADDR(dio_id));
          reg &= ~CC26XX_IOC_IOCFG_EDGE_IRQ_EN;
          cpu_mem_write_32(CC26XX_IOC_BASE +
            CC26XX_IOC_IOCFG_ADDR(dio_id), reg);
        }
    }
#endif

  mem_free(pv);
}

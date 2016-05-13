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
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright (c) 2016 Nicolas Pouillon <nipo@ssji.net>
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>

#include <mutek/mem_alloc.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/irq.h>
#include <device/class/gpio.h>
#include <device/class/icu.h>
#include <device/class/iomux.h>

#include <arch/psoc4/gpio_port.h>
#include <arch/psoc4/gpio.h>
#include <arch/psoc4/hsiom_port.h>
#include <arch/psoc4/hsiom.h>
#include <arch/psoc4/variant.h>

#include <mutek/printk.h>
#include <string.h>

#define dprintk(...) do{}while(0)
//#define dprintk printk

#define GPA PSOC4_GPIO_PRT_ADDR
#define HPA PSOC4_HSIOM_PRT_ADDR
#define GA PSOC4_GPIO_ADDR
#define HA PSOC4_HSIOM_ADDR

#if CONFIG_DRIVER_PSOC4_GPIO_ICU_CHANNEL_COUNT
struct psoc4_gpio_private_s
{
  struct dev_irq_src_s irq_in[PSOC4_GPIO_PORT_COUNT];
  struct dev_irq_sink_s irq_out[CONFIG_DRIVER_PSOC4_GPIO_ICU_CHANNEL_COUNT];
  uint8_t irq_sink_pin[CONFIG_DRIVER_PSOC4_GPIO_ICU_CHANNEL_COUNT];
};
#endif

static
int psoc4_gpio_mode(enum dev_pin_driving_e mode,
                    uint32_t *pin_value,
                    uint32_t *pin_mode)
{
  *pin_value = 0;

  switch (mode) {
  case DEV_PIN_DISABLED:
    *pin_mode = GPIO_PORT_PC_DM_OFF;
    return 0;
  case DEV_PIN_PUSHPULL:
    *pin_mode = GPIO_PORT_PC_DM_0_1;
    return 0;
  case DEV_PIN_INPUT:
    *pin_mode = GPIO_PORT_PC_DM_INPUT;
    return 0;
  case DEV_PIN_INPUT_PULLUP:
    *pin_mode = GPIO_PORT_PC_DM_0_PU;
    *pin_value = 1;
    return 0;
  case DEV_PIN_INPUT_PULLDOWN:
    *pin_mode = GPIO_PORT_PC_DM_PD_1;
    return 0;
  case DEV_PIN_OPENDRAIN:
    *pin_mode = GPIO_PORT_PC_DM_0_Z;
    *pin_value = 1;
    return 0;
  case DEV_PIN_OPENSOURCE:
    *pin_mode = GPIO_PORT_PC_DM_Z_1;
    return 0;
  case DEV_PIN_OPENDRAIN_PULLUP:
    *pin_mode = GPIO_PORT_PC_DM_0_PU;
    *pin_value = 1;
    return 0;
  case DEV_PIN_OPENSOURCE_PULLDOWN:
    *pin_mode = GPIO_PORT_PC_DM_PD_1;
    return 0;
  default:
    return -ENOTSUP;
  }

  return -ENOTSUP;
}

static DEV_GPIO_SET_MODE(psoc4_gpio_set_mode)
{
  struct device_s *dev = gpio->dev;
  uint32_t pin_value, pin_mode;
  error_t err;

  err = psoc4_gpio_mode(mode, &pin_value, &pin_mode);
  if (err)
    return err;

  uint64_t m
    = (endian_le64_na_load(mask) & ((1 << (io_last + 1)) - 1))
    << io_first;

  LOCK_SPIN_IRQ(&dev->lock);

  for (uint8_t pin = io_first; pin <= io_last; ++pin) {
    uint32_t tmp;

    if (!((m >> pin) & 1))
      continue;

    dprintk("%s %d mode %x val %d\n", __FUNCTION__, pin, pin_mode, pin_value);
    tmp = cpu_mem_read_32(GPA(PSOC4_IO_PORT(pin)) + GPIO_PORT_PC_ADDR);
    GPIO_PORT_PC_DM_SETVAL(PSOC4_IO_PIN(pin), tmp, pin_mode);
    cpu_mem_write_32(GPA(PSOC4_IO_PORT(pin)) + GPIO_PORT_PC_ADDR, tmp);

    if (pin_value)
      cpu_mem_write_32(GPA(PSOC4_IO_PORT(pin)) + GPIO_PORT_DR_SET_ADDR,
                       1 << PSOC4_IO_PIN(pin));
    else
      cpu_mem_write_32(GPA(PSOC4_IO_PORT(pin)) + GPIO_PORT_DR_CLR_ADDR,
                       1 << PSOC4_IO_PIN(pin));
  }

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEV_GPIO_SET_OUTPUT(psoc4_gpio_set_output)
{
  struct device_s *dev = gpio->dev;

  uint64_t mask = ((uint64_t)1 << (io_last + 1)) - ((uint64_t)1 << io_first);
  uint64_t _setm = (endian_le64_na_load(set_mask) << io_first) & mask;
  uint64_t _clearm = (endian_le64_na_load(clear_mask) << io_first) & mask;
  uint64_t setm = _setm & _clearm;
  uint64_t clearm = ~_setm & ~_clearm;
  uint64_t togglem = _setm & ~_clearm;

  LOCK_SPIN_IRQ(&dev->lock);

  for (uint8_t port = PSOC4_IO_PORT(io_first);
       port <= PSOC4_IO_PORT(io_last);
       ++port) {

    cpu_mem_write_32(GPA(port) + GPIO_PORT_DR_SET_ADDR, (setm >> (port * 8)) & 0xff);
    cpu_mem_write_32(GPA(port) + GPIO_PORT_DR_CLR_ADDR, (clearm >> (port * 8)) & 0xff);
    cpu_mem_write_32(GPA(port) + GPIO_PORT_DR_TOGGLE_ADDR, (togglem >> (port * 8)) & 0xff);
  }

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEV_GPIO_GET_INPUT(psoc4_gpio_get_input)
{
  struct device_s *dev = gpio->dev;
  uint64_t tmp = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  for (uint8_t port = PSOC4_IO_PORT(io_first);
       port <= PSOC4_IO_PORT(io_last);
       ++port) {

    uint64_t val = cpu_mem_read_32(GPA(port) + GPIO_PORT_PS_ADDR);
    tmp |= val << (port * 8);
  }

  dprintk("Read %llx\n", tmp);

  tmp >>= io_first;
  tmp &= ((uint64_t)1 << (io_last - io_first + 1)) - 1;
  endian_le64_na_store(data, tmp);

  dprintk("Read2 %llx\n", tmp);

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

#if CONFIG_DRIVER_PSOC4_GPIO_ICU_CHANNEL_COUNT
static DEV_IRQ_SINK_UPDATE(psoc4_gpio_icu_sink_update)
{
  struct device_s *dev = sink->base.dev;
  struct psoc4_gpio_private_s *pv = dev->drv_pv;
  uint_fast8_t sid = sink - pv->irq_out;
  uint8_t pin = pv->irq_sink_pin[sid];
  uint32_t config;
  uint32_t tmp;

  if (sid >= CONFIG_DRIVER_PSOC4_GPIO_ICU_CHANNEL_COUNT)
    return;

  switch (sense) {
  case DEV_IRQ_SENSE_RISING_EDGE:
    config = GPIO_PORT_INTR_CFG_EDGE_SEL_RISING;
    break;
  case DEV_IRQ_SENSE_FALLING_EDGE:
    config = GPIO_PORT_INTR_CFG_EDGE_SEL_FALLING;
    break;
  case DEV_IRQ_SENSE_NONE:
    config = GPIO_PORT_INTR_CFG_EDGE_SEL_DISABLE;
    break;
  case DEV_IRQ_SENSE_ANY_EDGE:
    config = GPIO_PORT_INTR_CFG_EDGE_SEL_BOTH;
    break;
  default:
    return;
  }

  LOCK_SPIN_IRQ(&dev->lock);

  tmp = cpu_mem_read_32(GPA(PSOC4_IO_PORT(pin)) + GPIO_PORT_PC_ADDR);
  GPIO_PORT_PC_DM_SET(PSOC4_IO_PIN(pin), tmp, INPUT);
  cpu_mem_write_32(GPA(PSOC4_IO_PORT(pin)) + GPIO_PORT_PC_ADDR, tmp);

  tmp = cpu_mem_read_32(GPA(PSOC4_IO_PORT(pin)) + GPIO_PORT_INTR_CFG_ADDR);
  GPIO_PORT_INTR_CFG_EDGE_SEL_SETVAL(PSOC4_IO_PIN(pin), tmp, config);
  cpu_mem_write_32(GPA(PSOC4_IO_PORT(pin)) + GPIO_PORT_INTR_CFG_ADDR, tmp);

  LOCK_RELEASE_IRQ(&dev->lock);

  return;
}

static DEV_ICU_GET_SINK(psoc4_gpio_icu_get_sink)
{
  struct device_s *dev = accessor->dev;
  struct psoc4_gpio_private_s *pv = dev->drv_pv;

  for (uint32_t sid; sid < CONFIG_DRIVER_PSOC4_GPIO_ICU_CHANNEL_COUNT; ++sid)
    if (pv->irq_sink_pin[sid] == id)
      return pv->irq_out + i;

  for (uint32_t sid; sid < CONFIG_DRIVER_PSOC4_GPIO_ICU_CHANNEL_COUNT; ++sid) {
    if (pv->irq_sink_pin[sid] != SID_UNUSED)
      continue;

    pv->irq_sink_pin[sid] = id;
    return pv->irq_out + i;
  }

  return NULL;
}

static DEV_IRQ_SRC_PROCESS(psoc4_gpio_process)
{
  struct device_s *dev = ep->base.dev;
  struct psoc4_gpio_private_s *pv = dev->drv_pv;
  uint8_t port = ep - pv->irq_in;

  cpu_mem_write_32(GA + GPIO_INTR_CAUSE_ADDR, 1 << port);

  uint32_t pins = cpu_mem_read_32(GPA(port) + GPIO_PORT_INTR_ADDR);
  cpu_mem_write_32(GPA(port) + GPIO_PORT_INTR_ADDR, pins);

  while (pins) {
    uint8_t pin = __builtin_ctzl(pins);
    pins &= ~(1 << pin);

    uint8_t io = PSOC4_IO(port, pin);

    for (uint32_t sid = 0; sid < CONFIG_DRIVER_PSOC4_GPIO_ICU_CHANNEL_COUNT; ++sid) {
      if (irq_sink_pin[sid] != io)
        continue;

      device_irq_sink_process(pv->irq_out + sid, 0);
      break;
    }
  }
}
#endif // CONFIG_DRIVER_PSOC4_GPIO_ICU_CHANNEL_COUNT

static DEV_IOMUX_SETUP(psoc4_gpio_iomux_setup)
{
  struct device_s *dev = accessor->dev;

  uint32_t pin_value, pin_mode, tmp;
  error_t err;

  err = psoc4_gpio_mode(dir, &pin_value, &pin_mode);
  if (err)
    return err;

  LOCK_SPIN_IRQ(&dev->lock);

  dprintk("%s %d mode %x val %d mux %d\n", __FUNCTION__, io_id, pin_mode, pin_value, mux);

  /* Pin mode */
  tmp = cpu_mem_read_32(GPA(PSOC4_IO_PORT(io_id)) + GPIO_PORT_PC_ADDR);
  GPIO_PORT_PC_DM_SETVAL(PSOC4_IO_PIN(io_id), tmp, pin_mode);
  cpu_mem_write_32(GPA(PSOC4_IO_PORT(io_id)) + GPIO_PORT_PC_ADDR, tmp);

  /* Default value (for pullups) */
  if (pin_value)
    cpu_mem_write_32(GPA(PSOC4_IO_PORT(io_id)) + GPIO_PORT_DR_SET_ADDR,
                     1 << PSOC4_IO_PIN(io_id));
  else
    cpu_mem_write_32(GPA(PSOC4_IO_PORT(io_id)) + GPIO_PORT_DR_CLR_ADDR,
                     1 << PSOC4_IO_PIN(io_id));

  /* HSIOM Mux mode */
  tmp = cpu_mem_read_32(HPA(PSOC4_IO_PORT(io_id)) + HSIOM_PORT_SEL_ADDR);
  HSIOM_PORT_SEL_IO_SEL_SETVAL(PSOC4_IO_PIN(io_id), tmp, mux);
  cpu_mem_write_32(HPA(PSOC4_IO_PORT(io_id)) + HSIOM_PORT_SEL_ADDR, tmp);

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEV_INIT(psoc4_gpio_init)
{
  for (int8_t port = 0; port < PSOC4_GPIO_PORT_COUNT; ++port) {
    cpu_mem_write_32(GPA(port) + GPIO_PORT_INTR_CFG_ADDR, 0);
    cpu_mem_write_32(GPA(port) + GPIO_PORT_INTR_ADDR,
                     cpu_mem_read_32(GPA(port) + GPIO_PORT_INTR_ADDR));
  }

#if CONFIG_DRIVER_PSOC4_GPIO_ICU_CHANNEL_COUNT
  struct psoc4_gpio_private_s *pv;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  device_irq_source_init(dev, pv->irq_in, PSOC4_GPIO_PORT_COUNT, &psoc4_gpio_process);
  if (device_irq_source_link(dev, pv->irq_in, PSOC4_GPIO_PORT_COUNT, -1))
    goto err_mem;

  device_irq_sink_init(dev, pv->irq_out,
                       CONFIG_DRIVER_PSOC4_GPIO_ICU_CHANNEL_COUNT,
                       &psoc4_gpio_icu_sink_update,
                       DEV_IRQ_SENSE_RISING_EDGE
                       | DEV_IRQ_SENSE_FALLING_EDGE
                       | DEV_IRQ_SENSE_ANY_EDGE);
#endif

  return 0;

#if CONFIG_DRIVER_PSOC4_GPIO_ICU_CHANNEL_COUNT
 err_mem:
  mem_free(pv);
#endif

  return -1;
}

static DEV_CLEANUP(psoc4_gpio_cleanup)
{
#if CONFIG_DRIVER_PSOC4_GPIO_ICU_CHANNEL_COUNT
  struct psoc4_gpio_private_s  *pv = dev->drv_pv;

  for (int8_t i = 0; i < PSOC4_GPIO_PORT_COUNT; ++i) {
    cpu_mem_write_32(GPA(PSOC4_IO_PORT(pin)) + GPIO_PORT_INTR_CFG_ADDR, 0);
    cpu_mem_write_32(GPA(PSOC4_IO_PORT(pin)) + GPIO_PORT_INTR_ADDR,
                     cpu_mem_read_32(GPA(PSOC4_IO_PORT(pin)) + GPIO_PORT_INTR_ADDR));
  }

  device_irq_source_unlink(dev, pv->irq_in, PSOC4_GPIO_PORT_COUNT);

  mem_free(pv);
#endif

  return 0;
}

#define psoc4_gpio_request dev_gpio_request_async_to_sync
#define psoc4_gpio_use dev_use_generic
#define psoc4_gpio_icu_link device_icu_dummy_link
#define psoc4_gpio_input_irq_range (dev_gpio_input_irq_range_t*)dev_driver_notsup_fcn

DRIVER_DECLARE(psoc4_gpio_drv, 0, "PSoC4 GPIO", psoc4_gpio,
#if CONFIG_DRIVER_PSOC4_GPIO_ICU_CHANNEL_COUNT
               DRIVER_ICU_METHODS(psoc4_gpio_icu),
#endif
               DRIVER_GPIO_METHODS(psoc4_gpio),
               DRIVER_IOMUX_METHODS(psoc4_gpio_iomux));

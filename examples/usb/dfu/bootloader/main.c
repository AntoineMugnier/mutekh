#define LOGK_MODULE_ID "main"

#include <mutek/printk.h>
#include <mutek/startup.h>
#include <hexo/context.h>
#include <device/class/gpio.h>
#include <arch/efm32/pin.h>

#include "usb.h"

static
void jump_to_firmware(void)
{
  const uint32_t *firmware = (const uint32_t *)(CONFIG_LOAD_ROM_RO_SIZE + CONFIG_LOAD_ROM_RO_ADDR);
  size_t invalid_count = 0;
  size_t check_count = 64;

  for (size_t i = 0; i < check_count; ++i)
    invalid_count += (firmware[i] == 0xffffffff || firmware[i] == 0);

  if (invalid_count > check_count / 4)
    return;

  void (*entry)() = (void*)((uintptr_t)firmware | 1);

  cpu_interrupt_disable();
    
  entry();
}

void app_start(void)
{
  error_t err;
  struct device_gpio_s gpio;
    
  err = device_get_accessor_by_path(&gpio.base, NULL, "gpio", DRIVER_CLASS_GPIO);
  if (err) {
    logk_error("Error getting GPIO device: %d", err);
    return;
  }

  dev_gpio_mode(&gpio, EFM32_PB10, DEV_PIN_INPUT_PULLUP);
  bool_t pressed = !dev_gpio_input(&gpio, EFM32_PB10, NULL);

  if (!pressed)
    jump_to_firmware();

  dev_gpio_mode(&gpio, EFM32_PE2, DEV_PIN_OPENSOURCE);
  dev_gpio_out(&gpio, EFM32_PE2, 1);
    
  usb_dev_init("DFU");

  logk("DFU Mode");
}

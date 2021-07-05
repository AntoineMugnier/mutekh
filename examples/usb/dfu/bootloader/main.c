#define LOGK_MODULE_ID "main"

#include <mutek/printk.h>
#include <mutek/startup.h>
#include <hexo/context.h>
#include <device/class/gpio.h>
#if defined(CONFIG_ARCH_EFM32)
# include <arch/efm32/pin.h>
# define PIN_BUTTON PB10
# define PIN_LED PE2
# define LED_ACT 1
#elif defined(CONFIG_ARCH_NRF5X)
# define PIN_BUTTON 11
# define PIN_LED 13
# define LED_ACT 0
#endif

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

  dev_gpio_mode(&gpio, PIN_BUTTON, DEV_PIN_INPUT_PULLUP);
  bool_t pressed = !dev_gpio_input(&gpio, PIN_BUTTON, NULL);

  if (!pressed)
    jump_to_firmware();

  dev_gpio_mode(&gpio, PIN_LED, DEV_PIN_PUSHPULL);
  dev_gpio_out(&gpio, PIN_LED, LED_ACT);
    
  usb_dev_init("DFU");

  logk("DFU Mode");
}

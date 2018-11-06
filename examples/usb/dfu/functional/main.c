#define LOGK_MODULE_ID "main"

#include <mutek/printk.h>
#include <mutek/startup.h>
#include <device/class/gpio.h>
#include <arch/efm32/pin.h>

void app_start()
{
  error_t err;
  struct device_gpio_s gpio;
    
  err = device_get_accessor_by_path(&gpio.base, NULL, "gpio", DRIVER_CLASS_GPIO);
  if (err) {
    logk_error("Error getting GPIO device: %d", err);
    return;
  }

  dev_gpio_mode(&gpio, EFM32_PE3, DEV_PIN_OPENSOURCE);
  dev_gpio_out(&gpio, EFM32_PE3, 1);
    
  usb_dev_init("1234");

  logk("Hello from target!");
}



#include <assert.h>

#include <mutek/console.h>
#include <mutek/shell.h>

#include <device/driver.h>
#include <device/device.h>
#include <device/class/char.h>

void main()
{
  assert(device_check_accessor(&console_dev.base));

  /* start a shell on the mutekh console */
  mutek_shell_start(&console_dev, "xterm");
}


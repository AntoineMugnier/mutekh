#include <mutek/printk.h>
#include <hexo/power.h>

void app_start()
{
    writek("Hello, minimal\n", 15);
    writek("__TEST__", 8);
    writek("OK__\n", 5);

    power_shutdown();
    power_reboot();
}


#include <pthread.h>

#if defined(CONFIG_HEXO_DEVICE_TREE)
#include <device/enum.h>
#include <device/device.h>
#include <device/driver.h>

extern struct device_s enum_root;

static const char *device_class_str[] = {
	"none", "block", "char", "enum", "fb", "icu", "input", "net",
	"sound", "timer", "spi", "lcd", "gpio", "i2c", "mem",
};

static void dump_enumerator(struct device_s *root, uint_fast8_t prefix)
{
	uint_fast8_t i;
	for (i=0; i<prefix; ++i)
		printk(" ");
	printk("device %p, type %s\n", root,
		   root->drv ? device_class_str[root->drv->class] : "[undriven]");
	CONTAINER_FOREACH(device_list, CLIST, &root->children, {
			dump_enumerator(item, prefix+1);
		});
}
#endif

void app_start()
{
#if defined(CONFIG_HEXO_DEVICE_TREE)
	dump_enumerator(&enum_root, 0);
#endif

	printk("Demo ended\n");
}


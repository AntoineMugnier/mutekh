

#include <device/enum.h>
#include <drivers/device/enum/pci/enum-pci.h>
#include <drivers/device/icu/8259/icu-8259.h>
#include <netinet/dhcp.h>
#include <netinet/if.h>
#include <hexo/interrupt.h>
#include <mutek/printk.h>
#include <stdio.h>

#include <device/device.h>
#include <device/driver.h>

//#include <drivers/fs/nfs/nfs.h>

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

extern struct device_s enum_pci;
extern struct device_s icu_dev;

void app_start()
{
  cpu_interrupt_enable();
  assert(cpu_is_interruptible());

#if defined(CONFIG_DRIVER_ENUM_PCI)
	device_init(&enum_pci);
	enum_pci.icudev = &icu_dev;
	enum_pci_init(&enum_pci, NULL);
#endif

  dump_enumerator(&enum_root, 0);

  if_up("eth0");

  if (dhcp_client("eth0"))
    while (1)
      continue;

  if (rarp_client("eth0"))
    while (1)
      continue;

  // Initialize NFS
  // to 192.168.1.237 -> 0xC0A801ED
  // to 10.0.2.2      -> 0x0A000202
/*   if ((err = nfs_mount("/nfs", 0xC0A801ED))) */
/*     printk("error while initializing NFS: %d\n",err); */

  printk("LEAVE\n");
}


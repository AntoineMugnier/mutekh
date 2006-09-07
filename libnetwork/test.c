#include <hexo/device.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_FRAG

extern struct device_s enum_pci;

struct device_s	ne2000 = DEVICE_INITIALIZER;

/*
 * test main.
 */

int_fast8_t		main()
{
  /* XXX plutot que d'initialiser un autre device, reprendre celui de l'enum pci */

  net_ns8390_init(&ne2000);

  while (1)
    net_ns8390_irq(&ne2000);

  return 0;
}


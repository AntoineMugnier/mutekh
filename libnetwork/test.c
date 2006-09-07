#include <hexo/device.h>
#include <hexo/error.h>
#include <hexo/alloc.h>
#include <../drivers/enum-pci/enum-pci.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

CONTAINER_OBJECT_FUNC(static inline, device_list, DLIST, device_list, NOLOCK, device_obj, list_entry);

extern struct device_s enum_pci;

struct device_s	*ne2000;

/*
 * test main.
 */

int_fast8_t		main()
{
  struct enum_id_pci_s	*enum_pv;
  struct net_proto_s	*rarp;

  CONTAINER_FOREACH(device_list, DLIST, device_list, &enum_pci.children,
  {
    enum_pv = item->enum_pv;
    if (enum_pv->vendor == 0x10ec &&
	enum_pv->devid == 0x8029)
      {
	ne2000 = item;
	if (!net_ns8390_init(item))
	  goto ok;
      }
  });

 ok:
  //  dev_net_register_proto(ne2000, &arp_protocol);
  rarp = dev_net_register_proto(ne2000, &rarp_protocol);

  rarp_request(ne2000, rarp, NULL);

  while (1)
    net_ns8390_irq(ne2000);

  return 0;
}


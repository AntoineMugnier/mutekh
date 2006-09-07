#ifndef DRIVER_NET_NS8390_H_
#define DRIVER_NET_NS8390_H_

#include <hexo/device.h>

/* devices addresses slots */

#define NET_NS8390_ADDR	0

/* tty device functions */

DEV_IRQ(net_ns8390_irq);
DEV_INIT(net_ns8390_init);
DEV_CLEANUP(net_ns8390_cleanup);
DEVNET_PREPAREPKT(net_ns8390_preparepkt);
DEVNET_SENDPKT(net_ns8390_sendpkt);
DEVNET_REGISTER_PROTO(net_ns8390_register_proto);

#endif


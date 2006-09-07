#ifndef DRIVER_NET_NS8390_H_
#define DRIVER_NET_NS8390_H_

#include <hexo/device.h>

/* devices addresses slots */

/* XXX */

/* tty device functions */

//DEV_IRQ(uart_8250_irq);
DEV_INIT(net_ns8390_init);
DEV_CLEANUP(net_ns8390_cleanup);
DEVNET_READ(net_ns8390_read);
DEVNET_WRITE(net_ns8390_write);

#endif


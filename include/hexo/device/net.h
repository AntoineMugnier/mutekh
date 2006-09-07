#if !defined(DEVICE_H) || defined(DEVICE_NET_H_)
#error This file can not be included directly
#else

#define DEVICE_NET_H_

#include "../types.h"
#include "../error.h"

#include <netinet/packet.h>
#include <netinet/protos.h>

#define DEVNET_PREPAREPKT(n)	void  (n) (struct device_s *dev, struct net_packet_s *packet, size_t size)

typedef DEVNET_PREPAREPKT(devnet_preparepkt_t);

#define dev_net_preparepkt(dev, ...) (dev)->drv->f.net.f_preparepkt(dev, __VA_ARGS__ )

#define DEVNET_SENDPKT(n)	void  (n) (struct device_s *dev, struct net_packet_s *packet, net_proto_id_t proto)

typedef DEVNET_SENDPKT(devnet_sendpkt_t);

#define dev_net_sendpkt(dev, ...) (dev)->drv->f.net.f_sendpkt(dev, __VA_ARGS__ )

#define DEVNET_REGISTER_PROTO(n)	struct net_proto_s  *(n) (struct device_s *dev, const struct net_proto_desc_s *desc)

typedef DEVNET_REGISTER_PROTO(devnet_register_proto_t);

#define dev_net_register_proto(dev, ...) (dev)->drv->f.net.f_register_proto(dev, __VA_ARGS__ )


/** Net device class methodes */
struct dev_class_net_s
{
  devnet_preparepkt_t		*f_preparepkt;
  devnet_sendpkt_t		*f_sendpkt;
  devnet_register_proto_t	*f_register_proto;
};


#endif


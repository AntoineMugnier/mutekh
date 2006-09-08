#ifndef NETINET_DUMMY_H_
#define NETINET_DUMMY_H_

#include <hexo/types.h>
#include <netinet/packet.h>
#include <netinet/ether.h>

struct		dummy_interface_s { };

#include <netinet/protos.h>

NET_PUSHPKT(dummy_push);

extern const struct net_proto_desc_s	dummy_protocol;

#endif


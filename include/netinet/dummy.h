#ifndef NETINET_DUMMY_H_
#define NETINET_DUMMY_H_

#include <hexo/types.h>
#include <netinet/packet.h>
#include <netinet/ether.h>

struct		dummy_interface_s
{
  /* XXX */
};

#include <netinet/protos.h>

ETH_PUSH(dummy_push_ether);

#endif


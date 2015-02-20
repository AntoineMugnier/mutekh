#ifndef NET_ADDR_H
#define NET_ADDR_H

#include <hexo/types.h>

struct net_addr_s
{
#if defined(CONFIG_BLE)
  uint8_t reliable;
  uint8_t llid;
  uint16_t att;
  uint16_t cid;
#endif
#if defined(CONFIG_NET_TCP) || defined(CONFIG_NET_UDP)
  uint16_t port;
#endif
#if defined(CONFIG_NET_IPV4)
  uint8_t ipv4[4];
#endif
#if defined(CONFIG_NET_IPV6)
  uint8_t ipv6[16];
#endif
#if defined(CONFIG_NET_ETHERNET)
  uint8_t mac[6];
#endif
  
};

#endif

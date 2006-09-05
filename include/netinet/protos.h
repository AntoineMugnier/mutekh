#ifndef NETINET_PROTOS_H_
#define NETINET_PROTOS_H_

#include <hexo/types.h>
#include <netinet/packet.h>
#include <netinet/ether.h>

/*
 * Prototype of a push function for ethernel level protocols.
 */

#define ETH_PUSH(f)	void (f)(struct ether_addr *source,		\
				 struct ether_addr *target,		\
				 struct packet_s *packet)

typedef ETH_PUSH(ether_push_t);

#include <netinet/arp.h>
#include <netinet/ip.h>
#include <netinet/dummy.h>

/*
 * This structure defines a protocol
 */

struct				ether_proto_s
{
  const char				*name;	/* the name of the protocol */
  uint_fast16_t				id;	/* protocol identifier */
  ether_push_t				*pushpkt; /* push packet function */
  union
  {
    const struct ip_interface_s		*ip;	/* ip protocol interface */
    const struct arp_interface_s	*arp;	/* arp protocol interface */
    const struct rarp_interface_s	*rarp;	/* rarp protocol interface */
    const struct dummy_interface_s	*dummy;	/* fake protocol for debug */
    const void				*other;	/* other protocol interface */
  } f;
};

#endif


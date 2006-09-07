#ifndef NETINET_PROTOS_H_
#define NETINET_PROTOS_H_

#include <hexo/types.h>
#include <netinet/packet.h>

#include <hexo/gpct_platform_hexo.h>
#include <hexo/gpct_lock_hexo.h>
#include <gpct/cont_hashlist.h>

struct device_s;

/*
 * Container type for protocols list.
 */

CONTAINER_TYPE(net_protos, HASHLIST, struct net_proto_s, NOLOCK, 8, UNSIGNED);

/*
 * Prototype of a push function.
 */

#define NET_PUSHPKT(f)	void (f)(struct device_s	*dev,		\
				 struct net_packet_s	*packet,	\
				 struct	net_proto_s	*protocol)

typedef NET_PUSHPKT(net_pushpkt_t);

/*
 * Prototype of the function used to prepare a packet.
 */

#define NET_PREPAREPKT(f)	void (f)(struct device_s	*dev,	   \
					 struct net_packet_s	*packet)

typedef NET_PREPAREPKT(net_preparepkt_t);

#include <netinet/ether.h>
#include <netinet/arp.h>
#include <netinet/ip.h>
#include <netinet/dummy.h>

typedef uint_fast16_t net_pkt_size_t;
typedef uint_fast16_t net_proto_id_t;

/*
 * This structure defines a protocol.
 */

struct					net_proto_desc_s
{
  const char				*name;	/* the name of the protocol */
  net_proto_id_t			id;	/* protocol identifier */
  net_pushpkt_t				*pushpkt; /* push packet function */
  net_preparepkt_t			*preparepkt; /* prepare packet func */
  union
  {
    const struct ether_interface_s	*ether;	/* ethernet interface */
    const struct ip_interface_s		*ip;	/* ip protocol interface */
    const struct arp_interface_s	*arp;	/* arp protocol interface */
    const struct rarp_interface_s	*rarp;	/* rarp protocol interface */
#if 0
    const struct icmp_interface_s	*icmp;	/* icmp protocol interface */
    const struct udp_interface_s	*udp;	/* udp protocol interface */
    const struct tcp_interface_s	*tcp;	/* tcp protocol interface */
#endif
    const struct dummy_interface_s	*dummy;	/* fake protocol for debug */
    const void				*other;	/* other protocol interface */
  } f;
  size_t				pv_size;
};

struct					net_proto_s
{
  const struct net_proto_desc_s		*desc;	/* protocol descriptor */
  net_protos_entry_t			list_entry;
  net_proto_id_t			id;	/* protocol identifier */
  struct net_proto_pv_s			*pv;	/* private data */
};

/*
 * Container functions.
 */

CONTAINER_FUNC(static inline, net_protos, HASHLIST, net_protos, NOLOCK, list_entry, UNSIGNED, id);

#endif


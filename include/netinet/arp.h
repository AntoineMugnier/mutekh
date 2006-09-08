#ifndef NETINET_ARP_H_
#define NETINET_ARP_H_

#include <hexo/types.h>
#include <netinet/ether.h>

/* ARP protocol opcodes. */
#define	ARPOP_REQUEST	1		/* ARP request.  */
#define	ARPOP_REPLY	2		/* ARP reply.  */
#define	ARPOP_RREQUEST	3		/* RARP request.  */
#define	ARPOP_RREPLY	4		/* RARP reply.  */
#define	ARPOP_InREQUEST	8		/* InARP request.  */
#define	ARPOP_InREPLY	9		/* InARP reply.  */
#define	ARPOP_NAK	10		/* (ATM)ARP NAK.  */

/* This structure defines an ethernet arp header.  */
struct		arphdr
{
  uint16_t	ar_hrd;		/* Format of hardware address.  */
  uint16_t	ar_pro;		/* Format of protocol address.  */
  uint8_t	ar_hln;		/* Length of hardware address.  */
  uint8_t	ar_pln;		/* Length of protocol address.  */
  uint16_t	ar_op;		/* ARP opcode (command).  */
} __attribute__ ((packed));

/* ARP protocol HARDWARE identifiers. */
#define ARPHRD_NETROM	0		/* From KA9Q: NET/ROM pseudo. */
#define ARPHRD_ETHER 	1		/* Ethernet 10/100Mbps.  */
#define	ARPHRD_EETHER	2		/* Experimental Ethernet.  */
#define	ARPHRD_AX25	3		/* AX.25 Level 2.  */
#define	ARPHRD_PRONET	4		/* PROnet token ring.  */
#define	ARPHRD_CHAOS	5		/* Chaosnet.  */
#define	ARPHRD_IEEE802	6		/* IEEE 802.2 Ethernet/TR/TB.  */
#define	ARPHRD_ARCNET	7		/* ARCnet.  */
#define	ARPHRD_APPLETLK	8		/* APPLEtalk.  */
#define	ARPHRD_DLCI	15		/* Frame Relay DLCI.  */
#define	ARPHRD_ATM	19		/* ATM.  */
#define	ARPHRD_METRICOM	23		/* Metricom STRIP (new IANA id).  */
#define ARPHRD_IEEE1394	24		/* IEEE 1394 IPv4 - RFC 2734.  */
#define ARPHRD_EUI64		27		/* EUI-64.  */
#define ARPHRD_INFINIBAND	32		/* InfiniBand.  */

/*
 * Ethernet Address Resolution Protocol.
 *
 * See RFC 826 for protocol description.  Structure below is adapted
 * to resolving internet addresses.  Field names used correspond to
 * RFC 826.
 */
struct		ether_arp
{
  struct arphdr	ea_hdr;		/* fixed-size header */
  uint8_t	arp_sha[ETH_ALEN];	/* sender hardware address */
  uint8_t	arp_spa[4];		/* sender protocol address */
  uint8_t	arp_tha[ETH_ALEN];	/* target hardware address */
  uint8_t	arp_tpa[4];		/* target protocol address */
} __attribute__ ((packed));

#define	arp_hrd	ea_hdr.ar_hrd
#define	arp_pro	ea_hdr.ar_pro
#define	arp_hln	ea_hdr.ar_hln
#define	arp_pln	ea_hdr.ar_pln
#define	arp_op	ea_hdr.ar_op

/*
 * -----8<-----
 */

#include <netinet/packet.h>
#include <netinet/protos.h>

#define NET_ARP_REQUEST(f)	void (f)(struct device_s	*dev,	\
					 struct net_proto_s	*arp,	\
					 uint8_t		*address)

typedef NET_ARP_REQUEST(net_arp_request_t);

#define NET_ARP_REPLY(f)	void (f)(struct device_s	*dev,	\
					 struct net_proto_s	*arp,	\
					 uint8_t		*mac,	\
					 uint8_t		*ip)

typedef NET_ARP_REPLY(net_arp_reply_t);

#define NET_ARP_UPDATE_TABLE(f)	void (f)(struct device_s	*dev,	\
					 struct net_proto_s	*arp,	\
					 uint8_t		*ip,	\
					 uint8_t		*mac)

typedef NET_ARP_UPDATE_TABLE(net_arp_update_table_t);

#define NET_ARP_GET_MAC(f)	uint8_t *(f)(struct device_s	*dev,	\
					     struct net_proto_s	*arp,	\
					     uint8_t		*ip)

typedef NET_ARP_GET_MAC(net_arp_get_mac_t);

#define NET_RARP_REQUEST(f)	void (f)(struct device_s	*dev,	\
					 struct net_proto_s	*arp,	\
					 uint8_t		*mac)

typedef NET_RARP_REQUEST(net_rarp_request_t);

/*
 * ARP interface.
 */

struct				arp_interface_s
{
  net_arp_request_t		*request;
  net_arp_reply_t		*reply;
  net_arp_update_table_t	*update_table;
  net_arp_get_mac_t		*get_mac;
};

/*
 * ARP table types.
 */

CONTAINER_TYPE(arp_table, HASHLIST, struct arp_entry_s, NOLOCK, 8, UNSIGNED);

/*
 * ARP private data.
 */

struct			net_pv_arp_s
{
  struct net_proto_s	*ip;
  arp_table_root_t	table;
};

/*
 * ARP entry.
 */

struct			arp_entry_s
{
  uint8_t		ip[4];
  uint8_t		mac[ETH_ALEN];
  arp_table_entry_t	list_entry;
};

/*
 * RARP interface.
 */

struct	rarp_interface_s
{
  net_rarp_request_t	*request;
};

/*
 * RARP private data.
 */

struct			net_pv_rarp_s
{
  struct net_proto_s	*ip;
};

NET_INITPROTO(arp_init);
NET_PUSHPKT(arp_pushpkt);
NET_PREPAREPKT(arp_preparepkt);
NET_ARP_REQUEST(arp_request);
NET_ARP_REPLY(arp_reply);
NET_ARP_UPDATE_TABLE(arp_update_table);
NET_ARP_GET_MAC(arp_get_mac);

NET_INITPROTO(rarp_init);
NET_PUSHPKT(rarp_pushpkt);
NET_PREPAREPKT(rarp_preparepkt);
NET_RARP_REQUEST(rarp_request);

extern const struct net_proto_desc_s	arp_protocol;
extern const struct net_proto_desc_s	rarp_protocol;

#endif


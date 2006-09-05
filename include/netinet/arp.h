#ifndef NETINET_ARP_H_
#define NETINET_ARP_H_

#include <hexo/types.h>

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
#include <netinet/ether.h>

/*
 * ARP interface.
 */

struct	arp_interface_s
{
  /* XXX */
};

/*
 * RARP interface.
 */

struct	rarp_interface_s
{
  /* XXX */
};

#include <netinet/protos.h>

ETH_PUSH(arp_push);
ETH_PUSH(rarp_push);

#endif


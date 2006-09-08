#ifndef NETINET_IP_H_
#define NETINET_IP_H_

#include <hexo/types.h>
#include <hexo/endian.h>

/* Type Of Service flags */
#define IPTOS_TOS_MASK		0x1E
#define IPTOS_TOS(tos)		((tos)&IPTOS_TOS_MASK)
#define	IPTOS_LOWDELAY		0x10
#define	IPTOS_THROUGHPUT	0x08
#define	IPTOS_RELIABILITY	0x04
#define	IPTOS_MINCOST		0x02

#define IPTOS_PREC_MASK		0xE0
#define IPTOS_PREC(tos)		((tos)&IPTOS_PREC_MASK)
#define IPTOS_PREC_NETCONTROL           0xe0
#define IPTOS_PREC_INTERNETCONTROL      0xc0
#define IPTOS_PREC_CRITIC_ECP           0xa0
#define IPTOS_PREC_FLASHOVERRIDE        0x80
#define IPTOS_PREC_FLASH                0x60
#define IPTOS_PREC_IMMEDIATE            0x40
#define IPTOS_PREC_PRIORITY             0x20
#define IPTOS_PREC_ROUTINE              0x00

/* IP options */
#define IPOPT_COPY		0x80
#define IPOPT_CLASS_MASK	0x60
#define IPOPT_NUMBER_MASK	0x1f

#define	IPOPT_COPIED(o)		((o)&IPOPT_COPY)
#define	IPOPT_CLASS(o)		((o)&IPOPT_CLASS_MASK)
#define	IPOPT_NUMBER(o)		((o)&IPOPT_NUMBER_MASK)

#define	IPOPT_CONTROL		0x00
#define	IPOPT_RESERVED1		0x20
#define	IPOPT_MEASUREMENT	0x40
#define	IPOPT_RESERVED2		0x60

#define IPOPT_END	(0 |IPOPT_CONTROL)
#define IPOPT_NOOP	(1 |IPOPT_CONTROL)
#define IPOPT_SEC	(2 |IPOPT_CONTROL|IPOPT_COPY)
#define IPOPT_LSRR	(3 |IPOPT_CONTROL|IPOPT_COPY)
#define IPOPT_TIMESTAMP	(4 |IPOPT_MEASUREMENT)
#define IPOPT_RR	(7 |IPOPT_CONTROL)
#define IPOPT_SID	(8 |IPOPT_CONTROL|IPOPT_COPY)
#define IPOPT_SSRR	(9 |IPOPT_CONTROL|IPOPT_COPY)
#define IPOPT_RA	(20|IPOPT_CONTROL|IPOPT_COPY)

#define IPOPT_OPTVAL 0
#define IPOPT_OLEN   1
#define IPOPT_OFFSET 2
#define IPOPT_MINOFF 4
#define MAX_IPOPTLEN 40
#define IPOPT_NOP IPOPT_NOOP
#define IPOPT_EOL IPOPT_END
#define IPOPT_TS  IPOPT_TIMESTAMP

#define	IPOPT_TS_TSONLY		0		/* timestamps only */
#define	IPOPT_TS_TSANDADDR	1		/* timestamps and addresses */
#define	IPOPT_TS_PRESPEC	3		/* specified modules only */

/* Misc */
#define IPVERSION	4
#define MAXTTL		255
#define IPDEFTTL	64

/* IP packet header */
struct iphdr {
  ENDIAN_BITFIELD(uint8_t	version:4,
		  uint8_t	ihl:4);
  uint8_t	tos;
  uint16_t	tot_len;
  uint16_t	id;
  ENDIAN_BITFIELD(uint16_t	flags:3,
		  uint16_t	fragment:13);
  uint8_t	ttl;
  uint8_t	protocol;
  uint16_t	check;
  uint32_t	saddr;
  uint32_t	daddr;
} __attribute__((packed));

/*
 * -----8<-----
 */

#include <netinet/packet.h>
#include <netinet/protos.h>

/*
 * IP protocol interface.
 */

struct	ip_interface_s
{
  /* XXX */
};

/*
 * IP private data.
 */

struct			net_pv_ip_s
{
  uint8_t		addr[4];
};

/*
 * IP functions
 */

NET_INITPROTO(ip_init);
NET_PUSHPKT(ip_pushpkt);
NET_PREPAREPKT(ip_preparepkt);

extern const struct net_proto_desc_s	ip_protocol;

#endif


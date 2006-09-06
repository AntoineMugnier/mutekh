#ifndef NETINET_PACKET_H_
#define NETINET_PACKET_H_

#include <hexo/types.h>
#include <hexo/alloc.h>
#include <string.h>

/*
 * A macro function to check alignment of headers.
 */

#define ALIGNED(a, v)							\
  ((uintptr_t)(a) % (v))

/*
 * Memory access macros.
 */

#ifdef CONFIG_NETWORK_AUTOALIGN
/* when auto-align is activated, non-aligned accesses are useless */
# define net_le16_load(a)		endian_le16(a)
# define net_le32_load(a)		endian_le32(a)
# define net_le64_load(a)		endian_le64(a)
# define net_be16_load(a)		endian_be16(a)
# define net_be32_load(a)		endian_be32(a)
# define net_be64_load(a)		endian_be64(a)
# define net_le16_store(a, v)		(a = endian_le16(v))
# define net_le32_store(a, v)		(a = endian_le32(v))
# define net_le64_store(a, v)		(a = endian_le64(v))
# define net_be16_store(a, v)		(a = endian_be16(v))
# define net_be32_store(a, v)		(a = endian_be32(v))
# define net_be64_store(a, v)		(a = endian_be64(v))
#else
/* otherwise, use non-aligned accesses */
# define net_le16_load(a)		endian_le16_na_load(&a)
# define net_le32_load(a)		endian_le32_na_load(&a)
# define net_le64_load(a)		endian_le64_na_load(&a)
# define net_be16_load(a)		endian_be16_na_load(&a)
# define net_be32_load(a)		endian_be32_na_load(&a)
# define net_be64_load(a)		endian_be64_na_load(&a)
# define net_le16_store(a, v)		endian_le16_na_store(&a, v)
# define net_le32_store(a, v)		endian_le32_na_store(&a, v)
# define net_le64_store(a, v)		endian_le64_na_store(&a, v)
# define net_be16_store(a, v)		endian_be16_na_store(&a, v)
# define net_be32_store(a, v)		endian_be32_na_store(&a, v)
# define net_be64_store(a, v)		endian_be64_na_store(&a, v)
#endif

/*
 * Maximum number of stages in our stack.
 */

#define NETWORK_MAX_STAGES	5

/*
 * This structure defines a packet.
 */

struct		net_packet_s
{
  uint8_t	*header[NETWORK_MAX_STAGES];	/* pointers to headers */
  uint_fast16_t	size[NETWORK_MAX_STAGES];	/* size of subpackets */
  uint_fast8_t	stage;				/* current stage */
  uint8_t	*packet;			/* raw packet */
  uint8_t	*sMAC;				/* source MAC address */
  uint8_t	*tMAC;				/* target MAC address */
  uint_fast8_t	MAClen;				/* length of MAC addresses */
};

/*
 * Create and initialise a new packet.
 */

static inline struct net_packet_s	*packet_create(void)
{
  struct net_packet_s			*packet;

  packet = mem_alloc(sizeof (struct net_packet_s), MEM_SCOPE_THREAD);
  memset(packet, 0, sizeof (struct net_packet_s));
  packet->stage = NETWORK_MAX_STAGES - 1;
  return packet;
}

#endif


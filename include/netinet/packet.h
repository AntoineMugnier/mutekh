/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Matthieu Bucchianeri <matthieu.bucchianeri@epita.fr> (c) 2006

*/

#ifndef NETINET_PACKET_H_
#define NETINET_PACKET_H_

#include <hexo/types.h>
#include <hexo/alloc.h>
#include <string.h>

/*
 * A macro function to check alignment of headers.
 */

#define NET_ALIGNED(a, v)						\
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
# define net_16_load(a)			(a)
# define net_32_load(a)			(a)
# define net_64_load(a)			(a)
# define net_16_store(a, v)		(a = (v))
# define net_32_store(a, v)		(a = (v))
# define net_64_store(a, v)		(a = (v))
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
# define net_16_load(a)			endian_16_na_load(&a)
# define net_32_load(a)			endian_32_na_load(&a)
# define net_64_load(a)			endian_64_na_load(&a)
# define net_16_store(a, v)		endian_16_na_store(&a, v)
# define net_32_store(a, v)		endian_32_na_store(&a, v)
# define net_64_store(a, v)		endian_64_na_store(&a, v)
#endif

/*
 * Maximum number of stages in our stack.
 */

#define NETWORK_MAX_STAGES	5

/*
 * XXX commenter tout ca
 */

struct		net_header_s
{
  uint8_t	*data;	/* pointers to headers */
  uint_fast16_t	size;	/* size of subpackets */
};

#include <hexo/gpct_platform_hexo.h>
#include <hexo/gpct_lock_hexo.h>
#include <gpct/object_refcount.h>
#include <gpct/cont_dlist.h>
#include <gpct/cont_slist.h>

#include <semaphore.h>

OBJECT_TYPE(packet_obj, REFCOUNT, struct net_packet_s);

CONTAINER_TYPE(packet_queue, DLIST, struct net_packet_s, NOLOCK);
CONTAINER_TYPE(packet_queue_lock, DLIST, struct net_packet_s, HEXO_SPIN_IRQ);

/*
 * This structure defines a packet.
 */

struct				net_packet_s
{
  struct net_header_s		header[NETWORK_MAX_STAGES];
  uint_fast8_t			stage;		/* current stage */
  uint8_t			*packet;	/* raw packet */
  uint8_t			*sMAC;		/* source MAC address */
  uint8_t			*tMAC;		/* target MAC address */
  uint8_t			*sIP;		/* source IP address */
  uint8_t			*tIP;		/* target IP address */
  uint_fast8_t			MAClen;		/* length of MAC addresses */
  uint_fast16_t			proto;		/* level 2 protocol id */

  packet_obj_entry_t		obj_entry;
  packet_queue_entry_t		queue_entry;
  packet_queue_lock_entry_t	queue_entry_spin;
};

#include <netinet/protos.h>

/*
 * Used to give info to the dispatch thread.
 */

struct				net_dispatch_s
{
  packet_queue_lock_root_t	*packets;
  net_protos_root_t		*protocols;
  struct device_s		*device;
  sem_t				*sem;
};

/*
 * The packet object.
 */

OBJECT_CONSTRUCTOR(packet_obj);
OBJECT_DESTRUCTOR(packet_obj);
uint_fast16_t		packet_checksum(uint8_t		*data,
					size_t		size);
void			*packet_dispatch(void	*data);

OBJECT_FUNC(static inline, packet_obj, REFCOUNT, packet_obj, obj_entry);
CONTAINER_PROTOTYPE(, packet_queue, packet_queue);
CONTAINER_PROTOTYPE(, packet_queue_lock, packet_queue_lock);

/*
 * XXX debug
 */

static inline uint_fast8_t printf_void(void *v, ...) { return 0; }

#define net_debug printf_void

#endif


#ifndef NETINET_PACKET_H_
#define NETINET_PACKET_H_

#include <hexo/types.h>

/*
 * Maximum number of stages in our stack.
 */

#define NETWORK_MAX_STAGES	3

/*
 * This structure defines a packet.
 */

struct		net_packet_s
{
  uint8_t	*header[NETWORK_MAX_STAGES];	/* pointers to subpackets */
  uint_fast16_t	size[NETWORK_MAX_STAGES];	/* size of subpackets */
  uint_fast8_t	stage;				/* current stage */
  uint8_t	*packet;			/* raw packet */
};

#endif


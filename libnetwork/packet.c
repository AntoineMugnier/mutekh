#include <netinet/packet.h>

/*
 * The packet object constructor.
 */

OBJECT_CONSTRUCTOR(packet_obj)
{
  struct net_packet_s		*packet;

  packet = mem_alloc(sizeof (struct net_packet_s), MEM_SCOPE_THREAD);
  memset(packet, 0, sizeof (struct net_packet_s));
  return packet;
}

/*
 * The packet object destructor.
 */

OBJECT_DESTRUCTOR(packet_obj)
{
  mem_free(obj);
}

/*
 * Compute the checksum of a packet chunk.
 */

uint_fast16_t		packet_checksum(uint8_t		*data,
					size_t		size)
{
  uint_fast32_t		checksum = 0;
  uint16_t		*d = (uint16_t*)data;

  while(size > 1)
    {
      checksum = checksum + *d++;
      size = size - 2;
    }

  if (size)
    checksum = checksum + *(uint8_t*)d;

  while (checksum >> 16)
    checksum = (checksum & 0xffff) + (checksum >> 16);

  return ~checksum;
}

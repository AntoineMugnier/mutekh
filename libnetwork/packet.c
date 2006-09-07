#include <netinet/packet.h>

OBJECT_CONSTRUCTOR(packet_obj)
{
  struct net_packet_s		*packet;

  packet = mem_alloc(sizeof (struct net_packet_s), MEM_SCOPE_THREAD);
  memset(packet, 0, sizeof (struct net_packet_s));
  return packet;
}

OBJECT_DESTRUCTOR(packet_obj)
{
  mem_free(obj);
}


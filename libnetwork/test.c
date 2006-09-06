#include <netinet/packet.h>
#include <netinet/ether.h>
#include <netinet/dummy.h>
#include <netinet/in.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * test main.
 */

int_fast8_t		main()
{
  struct net_packet_s	pkt;
  uint8_t		buf[1500];
  net_protos_root_t	protocols;
  struct ether_header	*eth;

  memset(buf, 0, sizeof (buf));
  eth = (struct ether_header*)buf;
  memcpy(eth->ether_shost, "\x00\x01\x01\xd4\x82\x92", 6);
  memcpy(eth->ether_dhost, "\xff\xff\xff\xff\xff\xff", 6);
  eth->ether_type = htons(ETHERTYPE_ARP);
  memset(&pkt, 0, sizeof (pkt));
  pkt.packet = buf;
  pkt.stage = 0;
  pkt.header[0] = pkt.packet;
  pkt.size[0] = 1500;

  net_protos_init(&protocols);

  net_protos_push(&protocols, &ether_protocol);
  net_protos_push(&protocols, &dummy_protocol);

  ether_push(NULL, &pkt, protocols);

  net_protos_destroy(&protocols);

  while (1)
    ;

  return 0;
}


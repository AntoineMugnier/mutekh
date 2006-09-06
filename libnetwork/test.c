#include <netinet/packet.h>
#include <netinet/ether.h>
#include <netinet/dummy.h>
#include <netinet/in.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_FRAG

extern struct device_s enum_pci;

/*
 * test main.
 */

int_fast8_t		main()
{
#if 0
  struct net_packet_s	pkt;
  uint8_t		buf[1500];
  net_protos_root_t	protocols;
  struct ether_header	*eth;
  struct ether_arp	*arp;

  memset(buf, 0, sizeof (buf));
  eth = (struct ether_header*)buf;
  memcpy(eth->ether_shost, "\x00\x01\x01\xd4\x82\x92", 6);
  memcpy(eth->ether_dhost, "\xff\xff\xff\xff\xff\xff", 6);
  eth->ether_type = htons(ETHERTYPE_ARP);

#ifdef TEST_FRAG
  /* we align the second header on 32 bits */
  arp = (struct ether_arp*)(buf + sizeof (struct ether_header) + 2);
#else
  arp = (struct ether_arp*)(buf + sizeof (struct ether_header));
#endif
  arp->arp_hrd = htons(ARPHRD_ETHER);
  arp->arp_pro = htons(0x0800);
  arp->arp_hln = 6;
  arp->arp_pln = 4;
  arp->arp_op = htons(ARPOP_REQUEST);
  memcpy(arp->arp_sha, "\x00\x01\x01\xd4\x82\xb9", 6);
  memcpy(arp->arp_spa, "\x0a\x02\x02\x33", 4);
  memcpy(arp->arp_tha, "\x00\x00\x00\x00\x00\x00", 6);
  memcpy(arp->arp_tpa, "\xc0\xa8\x01\x03", 4);

  memset(&pkt, 0, sizeof (pkt));
  pkt.packet = buf;
  pkt.stage = 0;
  pkt.header[0] = eth;
  pkt.size[0] = sizeof (struct ether_header) + sizeof (struct ether_arp);
#ifdef TEST_FRAG
  pkt.header[1] = arp;
  pkt.size[1] = sizeof (struct ether_arp);
#endif

  net_protos_init(&protocols);

  net_protos_push(&protocols, &ether_protocol);
  net_protos_push(&protocols, &dummy_protocol);
  net_protos_push(&protocols, &arp_protocol);

#if 0
  ether_push(NULL, &pkt, protocols);
#endif
#if 1
  arp_request("\xc0\xa8\x01\x03");
#endif
  net_protos_destroy(&protocols);
#endif
  struct device_s	ne2000 = DEVICE_INITIALIZER;
  struct net_packet_s	pkt;
  net_protos_root_t	protocols;
  uint8_t		*buff[1514];
  size_t		len;

  /* XXX plutot que d'initialiser un autre device, reprendre celui de l'enum pci */

  net_ns8390_init(&ne2000);

  net_protos_init(&protocols);

  net_protos_push(&protocols, &ether_protocol);
  net_protos_push(&protocols, &arp_protocol);

  while (1)
    {
      if ((len = dev_char_read(&ne2000, buff, 1514)))
	{
	  printf("New frame\n");
	  memset(&pkt, 0, sizeof (pkt));
	  pkt.packet = buff;
	  pkt.stage = 0;
	  pkt.header[0] = buff;
	  pkt.size[0] = len;

	  ether_push(&ne2000, &pkt, protocols);
	}
    }

  while (1)
    ;

  return 0;
}


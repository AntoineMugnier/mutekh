#include <netinet/ether.h>
#include <netinet/in.h>
#include <netinet/dummy.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#if 0
/*
 * simulate a fake packet.
 */

static void		test_pkt(void)
{
  struct ether_header	*hdr;
  uint8_t		pkt[512];

  memset(pkt, 0, sizeof (pkt));
  hdr = (struct ether_header*)pkt;
  memcpy(hdr->ether_shost, "\x0\x01\x42\x42\x42\x42", 6);
  memcpy(hdr->ether_dhost, "\xff\xff\xff\xff\xff\xff", 6);
  hdr->ether_type = ntohs(ETHERTYPE_ARP);

  strcpy((char*)pkt + sizeof (struct ether_header), "hello world !");

  ethernet_pushpkt(NULL, pkt, sizeof (pkt));
}

/*
 * test main.
 */

int_fast8_t		main()
{
  ethernet_init();

  ethernet_register(ETHERTYPE_ARP, &dummy_ethernet);

  test_pkt();

  ethernet_cleanup();

  while (1)
    ;

  return 0;
}
#endif

extern struct device_s tty_uart_dev;

int_fast8_t		main()
{
  printf("serial dump:\n");
  while (1)
    {
      uint8_t buf[16];
      uint32_t len;
      uint32_t i;

      len = dev_char_read(&tty_uart_dev, buf, 16);

      for (i = 0; i < len; i++)
	putchar(buf[i]);
    }
}

#include <hexo/types.h>
#include <netinet/packet.h>
#include <netinet/ether.h>
#include <netinet/in.h>
#include <hexo/device.h>

#include <stdio.h>

#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_dlist.h>

/*
 * We use a container for the list of protocols.
 */


/*
 * this function is the entry of the protocol stack.
 */

void			ethernet_push(struct device_s	*dev,
				      struct packet_s	*packet)
{
  struct ether_header	*ethhdr;
  uint_fast16_t		proto;

  ethhdr = (struct ether_header*)packet->buf[packet->stage];

  proto = ntohs(ethhdr->ether_type);

  printf("Ethernet protocol: %u\n", proto);

  /* XXX no protocol is able to handler the packet */
}

/*
 * initialize ethernet layer.
 */

void		ethernet_init(void)
{
}

/*
 * cleanup ethernet layer.
 */

void		ethernet_cleanup(void)
{
}

/*
 * register a new protocol.
 */

error_t		ethernet_register(struct ether_proto_s	*proto)
{
  return 0;
}

/*
 * this is a dummy packet handler.
 *
 * it only dumps packets.
 */

#include <netinet/dummy.h>
#include <netinet/packet.h>
#include <netinet/protos.h>

#include <stdio.h>

/*
 * Structures for declaring the protocol's properties & interface.
 */

static const struct dummy_interface_s	dummy_interface =
{
  /* XXX */
};

const struct ether_proto_s	dummy_protocol_eth =
  {
    .name = "dummy",
    .id = 0,
    .pushpkt = dummy_push_ether,
    .f.dummy = &dummy_interface
  };

/*
 * This function hexdump a packet.
 */

static void		dummy_hexdump(uint8_t	*data,
				      size_t	sz)
{
  uint_fast32_t		i, j;
  uint8_t		c;

  printf("    Data len = %u\n", sz);
  for (i = 0; i < sz; i += 16)
    {
      printf("%7x ", i);

      for (j = 0; j < 16 && i + j < sz; j++)
	{
	  printf("%2x ", data[i + j]);
	}

      for (; j < 16; j++)
	printf("   ");

      for (j = 0; j < 16 && i + j < sz; j++)
	{
	  c = data[i + j];

	  if (c >= 32 && c < 127)
	    putchar(c);
	  else
	    putchar('.');
	}
      putchar('\n');
    }
}

/*
 * Dump a packet coming from the ethernet layer.
 */

ETH_PUSH(dummy_push_ether)
{
  printf("Dummy packet dump:\n");

  printf("Source: %2x:%2x:%2x:%2x:%2x:%2x\n",
	 source->ether_addr_octet[0], source->ether_addr_octet[1],
	 source->ether_addr_octet[2], source->ether_addr_octet[3],
	 source->ether_addr_octet[4], source->ether_addr_octet[5]);

  printf("Target: %2x:%2x:%2x:%2x:%2x:%2x\n",
	 target->ether_addr_octet[0], target->ether_addr_octet[1],
	 target->ether_addr_octet[2], target->ether_addr_octet[3],
	 target->ether_addr_octet[4], target->ether_addr_octet[5]);

  dummy_hexdump(packet->buf[packet->stage], packet->size[packet->stage]);
}


#ifndef NET_NS8390_PRIVATE_H_
#define NET_NS8390_PRIVATE_H_

#include <hexo/types.h>
#include <hexo/lock.h>
#include <netinet/ether.h>
#include <netinet/protos.h>

/*
 * private data of a ns8390 network device
 */

struct			net_ns8390_context_s
{
  lock_t		lock;

  uint_fast16_t		base;
  uint_fast16_t		asic;
  uint_fast16_t		rx_start;
  uint_fast16_t		tx_start;
  uint_fast16_t		mem;
  uint_fast8_t		mode_16bits;

  uint8_t		mac[ETH_ALEN];

  net_protos_root_t	protocols;
};

#endif


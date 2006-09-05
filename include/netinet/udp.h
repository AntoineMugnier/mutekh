#ifndef NETINET_UDP_H_
#define NETINET_UDP_H_

#include <hexo/types.h>

/* The UDP packet header */
struct		udphdr
{
  uint16_t	source;
  uint16_t	dest;
  uint16_t	len;
  uint16_t	check;
} __attribute__ ((packed));

#endif


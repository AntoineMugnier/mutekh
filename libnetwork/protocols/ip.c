/*
 * IP protocol
 *
 */

#include <netinet/ip.h>
#include <netinet/packet.h>
#include <netinet/protos.h>
#include <hexo/device.h>

#include <stdio.h>

/*
 * Structures for declaring the protocol's properties & interface.
 */

static const struct ip_interface_s	ip_interface =
{
  /* XXX */
};

const struct net_proto_desc_s	ip_protocol =
  {
    .name = "IP",
    .id = ETHERTYPE_IP,
    .pushpkt = ip_pushpkt,
    .preparepkt = ip_preparepkt,
    .initproto = NULL,
    .f.ip = &ip_interface,
    .pv_size = sizeof (struct net_pv_ip_s),
  };


NET_PUSHPKT(ip_pushpkt)
{

}

NET_PREPAREPKT(ip_preparepkt)
{

}

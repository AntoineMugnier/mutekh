/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2016
*/

#ifndef INET_PROTOCOL_IP_H_
#define INET_PROTOCOL_IP_H_

enum inet_ip_proto_e
{
  INET_IP_PROTO_HOPOPT          = 0x00, // RFC 2460
  INET_IP_PROTO_ICMP            = 0x01, // RFC 792
  INET_IP_PROTO_IGMP            = 0x02, // RFC 1112
  INET_IP_PROTO_GGP             = 0x03, // RFC 823
  INET_IP_PROTO_IP_IN_IP        = 0x04, // RFC 2003
  INET_IP_PROTO_ST              = 0x05, // RFC 1819
  INET_IP_PROTO_TCP             = 0x06, // RFC 793
  INET_IP_PROTO_CBT             = 0x07, // RFC 2189
  INET_IP_PROTO_EGP             = 0x08, // RFC 888
  INET_IP_PROTO_NVP_II          = 0x0B, // RFC 741
  INET_IP_PROTO_UDP             = 0x11, // RFC 768
  INET_IP_PROTO_HMP             = 0x14, // RFC 869
  INET_IP_PROTO_RDP             = 0x1B, // RFC 908
  INET_IP_PROTO_IRTP            = 0x1C, // RFC 938
  INET_IP_PROTO_ISO_TP4         = 0x1D, // RFC 905
  INET_IP_PROTO_NETBLT          = 0x1E, // RFC 998
  INET_IP_PROTO_DCCP            = 0x21, // RFC 4340
  INET_IP_PROTO_IDPR            = 0x23, // RFC 1479
  INET_IP_PROTO_IPV6            = 0x29, // RFC 2473
  INET_IP_PROTO_SDRP            = 0x2A, // RFC 1940
  INET_IP_PROTO_IPV6_ROUTE      = 0x2B, // RFC 2460
  INET_IP_PROTO_IPV6_FRAG       = 0x2C, // RFC 2460
  INET_IP_PROTO_RSVP            = 0x2E, // RFC 2205
  INET_IP_PROTO_GRE             = 0x2F, // RFC 2890
  INET_IP_PROTO_ESP             = 0x32, // RFC 4303
  INET_IP_PROTO_AH              = 0x33, // RFC 4302
  INET_IP_PROTO_NARP            = 0x36, // RFC 1735
  INET_IP_PROTO_MOBILE          = 0x37, // RFC 2004
  INET_IP_PROTO_SKIP            = 0x39, // RFC 2356
  INET_IP_PROTO_IPV6_ICMP       = 0x3A, // RFC 4884
  INET_IP_PROTO_IPV6_NONXT      = 0x3B, // RFC 2460
  INET_IP_PROTO_IPV6_OPTS       = 0x3C, // RFC 2460
  INET_IP_PROTO_VMTP            = 0x51, // RFC 1045
  INET_IP_PROTO_SECURE_VMTP     = 0x52, // RFC 1045
  INET_IP_PROTO_OSPF            = 0x59, // RFC 1583
  INET_IP_PROTO_IPIP            = 0x5E, // RFC 2003
  INET_IP_PROTO_ETHERIP         = 0x61, // RFC 3378
  INET_IP_PROTO_ENCAP           = 0x62, // RFC 1241
  INET_IP_PROTO_IPCOMP          = 0x6C, // RFC 3173
  INET_IP_PROTO_PGM             = 0x71, // RFC 3208
  INET_IP_PROTO_L2TP            = 0x73, // RFC 3931
  INET_IP_PROTO_MOBILITY_HEADER = 0x87, // RFC 6275
  INET_IP_PROTO_UDPLITE         = 0x88, // RFC 3828
  INET_IP_PROTO_MPLS_IN_IP      = 0x89, // RFC 4023
  INET_IP_PROTO_MANET           = 0x8A, // RFC 5498
  INET_IP_PROTO_HIP             = 0x8B, // RFC 5201
  INET_IP_PROTO_SHIM6           = 0x8C, // RFC 5533
  INET_IP_PROTO_WESP            = 0x8D, // RFC 5840
  INET_IP_PROTO_ROHC            = 0x8E, // RFC 5856
};

#endif

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

/**
   @file
   @module {Libraries::Internet protocol suite}
*/

#ifndef INET_PROTOCOL_IPV4_H_
#define INET_PROTOCOL_IPV4_H_

#include <hexo/decls.h>
#include <hexo/types.h>
#include <string.h>
#include <inet/protocol/checksum.h>

struct inet_ipv4_hdr_s
{
  uint8_t version_ihl;
  uint8_t dscp_ecn;
  uint16_t total_length;
  uint16_t id;
  uint16_t flags_offset;
  uint8_t ttl;
  uint8_t protocol;
  uint16_t checksum;
  uint8_t src_addr[4];
  uint8_t dst_addr[4];
};

#define INET_IPV4_HDR_SIZE_MIN sizeof(struct inet_ipv4_hdr_s)

ALWAYS_INLINE
void inet_ipv4_hdr_ihl_set(struct inet_ipv4_hdr_s *hdr, uint_fast8_t ihl)
{
  hdr->version_ihl = 0x40 | ihl;
}

ALWAYS_INLINE
uint_fast8_t inet_ipv4_hdr_version_get(const struct inet_ipv4_hdr_s *hdr)
{
  return hdr->version_ihl >> 4;
}

ALWAYS_INLINE
size_t inet_ipv4_hdr_size_get(const struct inet_ipv4_hdr_s *hdr)
{
  return (hdr->version_ihl & 0xf) * 4;
}

ALWAYS_INLINE
uint16_t inet_ipv4_hdr_checksum(const struct inet_ipv4_hdr_s *hdr)
{
  return inet_checksum(hdr, inet_ipv4_hdr_size_get(hdr));
}

ALWAYS_INLINE
void inet_ipv4_hdr_init(struct inet_ipv4_hdr_s *hdr, size_t options_words)
{
  memset(hdr, 0, sizeof(*hdr) + options_words * 4);
  inet_ipv4_hdr_ihl_set(hdr, 5 + options_words);
}

#define INET_IPV4_FMT "%d.%d.%d.%d"
#define INET_IPV4_ARG(x) (x)[0],(x)[1],(x)[2],(x)[3]

#endif

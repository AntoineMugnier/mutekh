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

#ifndef INET_PROTOCOL_ICMP_H_
#define INET_PROTOCOL_ICMP_H_

#include <hexo/decls.h>
#include <hexo/types.h>

struct inet_icmp_hdr_s
{
  uint8_t type;
  uint8_t code;
  uint16_t checksum;
  union {
    struct {
      uint16_t identifier;
      uint16_t seq_no;
    } __attribute__((__packed__));
    uint8_t pointer;
  } __attribute__((__packed__));
};

enum inet_icmp_type_e
{
  INET_ICMP_ECHO_REPLY              = 0,
  INET_ICMP_DESTINATION_UNREACHABLE = 3,
  INET_ICMP_SOURCE_QUENCH           = 4,
  INET_ICMP_REDIRECT                = 5,
  INET_ICMP_ECHO                    = 8,
  INET_ICMP_TIME_EXCEEDED           = 11,
  INET_ICMP_PARAMETER_PROBLEM       = 12,
  INET_ICMP_TIMESTAMP               = 13,
  INET_ICMP_TIMESTAMP_REPLY         = 14,
  INET_ICMP_INFORMATION_REQUEST     = 15,
  INET_ICMP_INFORMATION_REPLY       = 16,
};

#endif

/*
 * Copyright (c) 1982, 1986, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	@(#)tcp.h	8.1 (Berkeley) 6/10/93
 */

#ifndef NETINET_TCP_H_
#define NETINET_TCP_H_

#include <hexo/types.h>
#include <hexo/endian.h>

/*
 * TCP header flags.
 */

#define TH_FIN	0x01
#define TH_SYN	0x02
#define TH_RST	0x04
#define TH_PUSH	0x08
#define TH_ACK	0x10
#define TH_URG	0x20

/*
 * TCP header.
 * Per RFC 793, September, 1981.
 */

struct		tcphdr
{
  uint16_t	th_sport;		/* source port */
  uint16_t	th_dport;		/* destination port */
  uint32_t	th_seq;		/* sequence number */
  uint32_t	th_ack;		/* acknowledgement number */
  ENDIAN_BITFIELD(uint8_t th_off:4,		/* data offset */
		  uint8_t th_x2:4);		/* (unused) */
  uint8_t	th_flags;
  uint16_t	th_win;		/* window */
  uint16_t	th_sum;		/* checksum */
  uint16_t	th_urp;		/* urgent pointer */
} __attribute__((packed));

/*
 * Default maximum segment size for TCP.
 * With an IP MSS of 576, this is 536,
 * but 512 is probably more convenient.
 * This should be defined as MIN(512, IP_MSS - sizeof (struct tcpiphdr)).
 */
#define TCP_MSS			512

#define TCP_MAXWIN		65535	/* largest value for (unscaled) window */

#define TCP_MAX_WINSHIFT	14	/* maximum window shift */

/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Matthieu Bucchianeri <matthieu.bucchianeri@epita.fr> (c) 2006

*/

#include <netinet/packet.h>
#include <netinet/protos.h>
#include <netinet/if.h>
#include <netinet/libtcp.h>

#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_hashlist.h>

/*
 * Container types for tcp session list.
 */

CONTAINER_TYPE(tcp_session, HASHLIST, struct net_tcp_session_s, NOLOCK, 64, BLOB, sizeof (struct net_tcp_addr_s));

/*
 * Window default size
 */

#define TCP_DFL_WINDOW	4096

/*
 * Total length of headers for a TCP packet
 */

#define TCP_HEADERS_LEN	40	/* IP + TCP headers */

/*
 * Connection stage
 */

#define TCP_STATE_ERROR		0
#define TCP_STATE_SYN_SENT	1
#define TCP_STATE_FIN_WAIT	2
#define TCP_STATE_ESTABLISHED	3
#define TCP_STATE_LISTEN	4
#define TCP_STATE_FIN_REQ	5

/*
 * Control operations
 */

#define TCP_OPEN	0
#define TCP_ACK_OPEN	1
#define TCP_ACK_DATA	2
#define TCP_FIN		3
#define TCP_ACK_FIN	4

/*
 * This structure defines a TCP session.
 */

struct			net_tcp_session_s
{
  struct net_if_s	*interface;
  struct net_proto_s	*addressing;
  struct net_tcp_addr_s	local;
  struct net_tcp_addr_s	remote[1];

  uint_fast32_t		curr_seq;
  uint_fast32_t		to_ack;
  uint_fast16_t		send_win;
  uint_fast16_t		send_mss;
  uint_fast32_t		recv_seq;
  uint_fast16_t		recv_win;
  uint_fast16_t		recv_mss;

  tcp_connect_t		*connect;
  void			*connect_data;
  tcp_receive_t		*receive;
  void			*receive_data;
  tcp_close_t		*close;
  void			*close_data;
  tcp_accept_t		*accept;
  void			*accept_data;

  uint_fast8_t		state;

  tcp_session_entry_t	list_entry;
};

/*
 * Prototypes
 */

NET_PUSHPKT(tcp_pushpkt);
uint8_t	*tcp_preparepkt(struct net_if_s		*interface,
			struct net_proto_s	*addressing,
			struct net_packet_s	*packet,
			size_t			size,
			size_t			max_padding);
void	tcp_send_controlpkt(struct net_tcp_session_s	*session,
			    uint_fast8_t		operation);
void	tcp_send_datapkt(struct net_tcp_session_s	*session,
			 void				*data,
			 size_t				size,
			 uint_fast8_t			flags);

extern const struct net_proto_desc_s	tcp_protocol;

#endif


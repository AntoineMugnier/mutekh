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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <netinet/if.h>
#include <netinet/in.h>
#include <netinet/libudp.h>
#include <netinet/libtcp.h>
#include <netinet/tcp.h>
#include <netinet/nfs.h>
#include <netinet/socket.h>

#include <stdio.h>
#include <string.h>
#include <timer.h>

#include <pthread.h>
#include <timer.h>

#ifdef CONFIG_NETWORK_TCP
void			connection_close(struct net_tcp_session_s *session,
					 void			  *ptr)
{
  printf("%p closed\n", session);
}

void			data_arrival(struct net_tcp_session_s	*session,
				     void			*data,
				     size_t			size,
				     void			*ptr)
{
  //  printf("%p received %P\n", session, data, size);

  //tcp_send(session, ptr, strlen(ptr));
}

void			after_connect(struct net_tcp_session_s	*session,
				      void			*ptr)
{
  char			req[] = "GET /icons/apache_pb2.gif HTTP/1.1\r\nHost: 10.2.2.37\r\nConnection: Keep-Alive\r\n\r\n";

  if (session->state == TCP_STATE_ERROR)
    {
      printf("error %d\n", session->state);
      return;
    }

  printf("%p opened\n", session);

  tcp_on_receive(session, data_arrival, ptr);
  tcp_on_close(session, connection_close, NULL);

  tcp_send(session, req, strlen(req));

  // tcp_close(session);

}

void			*tcp_test(void *p)
{
  struct net_tcp_addr_s local;
  struct net_tcp_addr_s remote;

  IPV4_ADDR_SET(local.address, 0x0a0202f0);
  IPV4_ADDR_SET(remote.address, 0x0a02026d);
  remote.port = htons(80);

  tcp_open(&local, &remote, after_connect, NULL);

  return NULL;
}
#endif

#ifdef CONFIG_NETWORK_UDP
#include "test_server.c"

void			*err_test(void *p)
{
  struct net_udp_addr_s local;
  struct net_udp_addr_s remote;

  IPV4_ADDR_SET(local.address, 0x0a0202f0);
  local.port = htons(4242);
  IPV4_ADDR_SET(remote.address, 0x0a020266);
  remote.port = htons(80);

  udp_send(&local, &remote, "test", 4);

  return NULL;
}
#endif

#ifdef CONFIG_NETWORK_NFS
void			*nfs_test(void *p)
{

  struct nfs_s	nfs;
  nfs_handle_t	root;
  nfs_handle_t	test;

  memset(&nfs, 0, sizeof (nfs));
  IPV4_ADDR_SET(nfs.address, 0x0a02026d);
  IPV4_ADDR_SET(nfs.local.address, 0x0a0202f0);
  nfs_init(&nfs);

  printf("mountd port: %u\nnfsd port: %u\n", ntohs(nfs.mountd.port), ntohs(nfs.nfsd.port));

  if (!nfs_mount(&nfs, "/home/buck/export", root))
    {
      if (!nfs_lookup(&nfs, "test", root, test, NULL))
	{
	  char buf[1024];
	  ssize_t read;

	  read = nfs_read(&nfs, test, buf, 0, 1024);
	  printf("read(%d): %P\n", read, buf, read);
	}

      nfs_umount(&nfs);
    }

  nfs_destroy(&nfs);

  return NULL;
}
#endif

void			*pf_packet_test(void *p)
{
  uint8_t		buff[1514];
  socket_t		s;
  ssize_t		sz;
  struct sockaddr_ll	addr;
  socklen_t		addrlen = sizeof (addr);

  s = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_ALL));

  assert(s != NULL);

  addr.sll_ifindex = 1;
  addr.sll_protocol = htons(ETH_P_ARP);
  assert(bind(s, (struct sockaddr *)&addr, addrlen) >= 0);

  while ((sz = recvfrom(s, buff, sizeof (buff), 0, (struct sockaddr *)&addr, &addrlen)) > 0)
    {
      printf("\n\n\n%P\n", buff, sz);
    }

  return NULL;
}


#ifdef CONFIG_NETWORK_PROFILING
static TIMER_CALLBACK(profiling)
{
  netprofile_show();
  timer_add_event(&timer_ms, timer);
}
#endif

void *net_up(void *p)
{
  pthread_t th;

  if_up("eth0");
  if_up("eth1");

  rarp_client("eth0");

#ifdef CONFIG_NETWORK_ROUTING

  struct net_route_s *route = mem_alloc(sizeof(struct net_route_s), MEM_SCOPE_SYS);


  route->interface = if_get_by_name("eth0");
  IPV4_ADDR_SET(route->target, 0x0a020200);
  IPV4_ADDR_SET(route->mask, 0xffffff00);
  route->type = ROUTETYPE_NET | ROUTETYPE_DIRECT;
  route_add(route);

  route = mem_alloc(sizeof(struct net_route_s), MEM_SCOPE_SYS);

  route->interface = if_get_by_name("eth1");
  IPV4_ADDR_SET(route->target, 0x0a020300);
  IPV4_ADDR_SET(route->mask, 0xffffff00);
  route->type = ROUTETYPE_NET | ROUTETYPE_DIRECT;
  route_add(route);

#endif

#ifdef CONFIG_NETWORK_UDP
  eval_server();
#endif

#if 0
  pthread_create(&th, NULL, err_test, NULL);
#endif

#if 0
  pthread_create(&th, NULL, tcp_test, NULL);
#endif

#if 0
  pthread_create(&th, NULL, nfs_test, NULL);
#endif

#if 0
  pthread_create(&th, NULL, pf_packet_test, NULL);
#endif

#ifdef CONFIG_NETWORK_PROFILING
  static struct timer_event_s prof;

  prof.delay = 30000;
  prof.pv = NULL;
  prof.callback = profiling;

  timer_add_event(&timer_ms, &prof);
#endif

  return NULL;
}


/*
 * test main.
 */

#ifndef LINUXSIM
int_fast8_t		main()
#else
int_fast8_t		_main()
#endif
{
  pthread_t th;
  uint_fast32_t i;

  for (i = 0; i < 10000000; i++)
    ;

  /*
   * Enable AC (on linux sim)
   */

  asm volatile("	pushf						\n"
	       "	orl	$0x40000, (%esp)			\n"
	       "	popf						\n");


  pthread_create(&th, NULL, net_up, NULL);

  return 0;
}

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

#include <stdio.h>
#include <string.h>

#include <pthread.h>
#include <timer.h>

void			eval_server();

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
  char			req[] = "GET / HTTP/1.1\r\nHost: 10.2.2.37\r\nConnection: keep-alive\r\n\r\n";

  if (session->state == TCP_STATE_ERROR)
    {
      printf("error %d\n", session->state);
      return;
    }

  printf("%p opened\n", session);

  tcp_on_receive(session, data_arrival, ptr);
  tcp_on_close(session, connection_close, NULL);

  tcp_send(session, req, strlen(req));

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
	  printf("read: %P\n", buf, read);
	}

      nfs_umount(&nfs);
    }

  return NULL;
}

/*
 * test main.
 */

int_fast8_t		main()
{
  uint_fast32_t i;

  for (i = 0; i < 10000000; i++)
    ;

  if_up("eth0");
  if_up("eth1");

#if 1

  struct net_route_s *route = mem_alloc(sizeof(struct net_route_s), MEM_SCOPE_SYS);


  route->interface = if_get("eth0");
  IPV4_ADDR_SET(route->target, 0x0a020200);
  IPV4_ADDR_SET(route->mask, 0xffffff00);
  route->type = ROUTETYPE_NET | ROUTETYPE_DIRECT;
  route_add(if_get("eth0"), route);

  route = mem_alloc(sizeof(struct net_route_s), MEM_SCOPE_SYS);

  route->interface = if_get("eth1");
  IPV4_ADDR_SET(route->target, 0x0a020300);
  IPV4_ADDR_SET(route->mask, 0xffffff00);
  route->type = ROUTETYPE_NET | ROUTETYPE_DIRECT;
  route_add(if_get("eth1"), route);

  route = mem_alloc(sizeof(struct net_route_s), MEM_SCOPE_SYS);

  route->interface = if_get("eth0");
  IPV4_ADDR_SET(route->target, 0x0a020200);
  IPV4_ADDR_SET(route->mask, 0xffffff00);
  route->type = ROUTETYPE_NET | ROUTETYPE_DIRECT;
  route_add(if_get("eth1"), route);

  route = mem_alloc(sizeof(struct net_route_s), MEM_SCOPE_SYS);

  route->interface = if_get("eth1");
  IPV4_ADDR_SET(route->target, 0x0a020300);
  IPV4_ADDR_SET(route->mask, 0xffffff00);
  route->type = ROUTETYPE_NET | ROUTETYPE_DIRECT;
  route_add(if_get("eth0"), route);

  //  if_up("eth2");
#endif

#if 0
  eval_server();
#endif

#if 0
  pthread_t th;
  pthread_create(&th, NULL, tcp_test, NULL);
#endif

#if 1
  pthread_t th;
  pthread_create(&th, NULL, nfs_test, NULL);
#endif

  return 0;
}



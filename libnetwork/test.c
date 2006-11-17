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
#include <netinet/ping.h>
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

#if !defined(CONFIG_NETWORK_UDP)
#include "test_server.c"

void			*err_test(void *p)
{
  struct net_udp_addr_s remote;

  IPV4_ADDR_SET(remote.address, 0x0a020266);
  remote.port = htons(80);

  udp_send(NULL, &remote, "test", 4);

  return NULL;
}
#endif

#ifdef CONFIG_NETWORK_NFS
NFS_READDIR_CB(rdir)
{
  printf("  %s\n", filename);

  return 0;
}

void			*nfs_test(void *p)
{

  struct nfs_s	nfs;
  nfs_handle_t	root;
  nfs_handle_t	test, test_dir;
  struct nfs_attr_s attr;

  memset(&nfs, 0, sizeof (nfs));
  IPV4_ADDR_SET(nfs.address, 0x0a02026d);
  nfs_init(&nfs);

  printf("mountd port: %u\nnfsd port: %u\n", ntohs(nfs.mountd.port), ntohs(nfs.nfsd.port));

  if (!nfs_mount(&nfs, "/home/buck/export", root))
    {
      struct nfs_statfs_s	fs;

      if (!nfs_statfs(&nfs, root, &fs))
	{
	  printf(" NFS transfer unit: %u bytes/request\n", fs.transfer_unit);
	  printf(" NFS filesystem: %u Mb total, %u Mb free, %u Mb quota\n",
		 (uint32_t)(((uint64_t)fs.blocks * fs.block_size) / 1048576),
		 (uint32_t)(((uint64_t)fs.blocks_free * fs.block_size) / 1048576),
		 (uint32_t)(((uint64_t)fs.blocks_avail * fs.block_size) / 1048576));
	}

      if (!nfs_lookup(&nfs, "test", root, test, NULL))
	{
	  char buf[1024];
	  ssize_t read;

	  read = nfs_read(&nfs, test, buf, 0, 1024);
	  printf("read(%d): %P\n", read, buf, read);

	  buf[4] = (((buf[4] - 'a') + 1) % 26) + 'a';
	  printf("wrote %d\n", nfs_write(&nfs, test, buf, 0, read));

	  read = nfs_read(&nfs, test, buf, 0, 1024);
	  printf("read(%d): %P\n", read, buf, read);
	}

      printf("remove: %d\n", nfs_unlink(&nfs, root, "new_file"));

      memset(&attr, 0xff, sizeof (struct nfs_attr_s));
      attr.uid = 500;
      attr.gid = 500;
      attr.mode = 0644;
      printf("create: %d\n", nfs_creat(&nfs, root, "new_file", &attr, test));

      memset(&attr, 0xff, sizeof (struct nfs_attr_s));
      attr.uid = 500;
      attr.gid = 500;
      attr.mode = 0755;
      printf("mkdir: %d\n", nfs_mkdir(&nfs, root, "new_dir", &attr, test_dir));
      memset(&attr, 0xff, sizeof (struct nfs_attr_s));
      attr.uid = 500;
      attr.gid = 500;
      attr.mode = 0644;
      printf("create: %d\n", nfs_creat(&nfs, test_dir, "new_file_in_dir", &attr, test));

      nfs_readdir(&nfs, root, rdir, NULL);

      nfs_umount(&nfs, "/home/buck/export");
    }

  nfs_destroy(&nfs);

  return NULL;
}
#endif

#ifdef CONFIG_NETWORK_SOCKET_PACKET
void			*pf_packet_test(void *p)
{
  uint8_t		buff[1514];
  socket_t		s;
  ssize_t		sz;
  struct sockaddr_ll	addr;
  socklen_t		addrlen = sizeof (addr);
  struct packet_mreq	req;

  s = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_ALL));

  assert(s != NULL);

  addr.sll_family = AF_PACKET;
  addr.sll_ifindex = 1;
  addr.sll_protocol = htons(ETH_P_ALL);
  assert(bind(s, (struct sockaddr *)&addr, addrlen) >= 0);

  req.mr_ifindex = 1;
  req.mr_type = PACKET_MR_PROMISC;
  assert(setsockopt(s, SOL_PACKET, PACKET_ADD_MEMBERSHIP, &req, sizeof (req)) >= 0);

  while ((sz = recvfrom(s, buff, sizeof (buff), 0, (struct sockaddr *)&addr, &addrlen)) > 0)
    {
      printf("\n\n\n%P\n", buff, sz);
    }

  return NULL;
}
#endif

#ifdef CONFIG_NETWORK_PROFILING
static TIMER_CALLBACK(profiling)
{
  netprofile_show();
  timer_add_event(&timer_ms, timer);
}
#endif

 sem_t sem;

void *toto(void *p)
{
  printf("wait...");
  sem_wait(&sem);
  printf("ok\n");
}

void *net_up(void *p)
{
  pthread_t th;

#if 0
  sem_init(&sem, 0, 0);

  pthread_create(&th, NULL, toto, NULL);

  int i;
  for (i = 0; 0 && i < 1000000000; i++)
    ;

  printf("post\n");
   sem_post(&sem);
#endif

#ifdef CONFIG_NETWORK

  if_up("eth0");
  if_up("eth1");

#ifdef CONFIG_NETWORK_RARP
  rarp_client("eth0");
#endif

#ifdef CONFIG_NETWORK_DHCLIENT
  dhcp_client("eth0");
#endif

  struct net_route_s *route = mem_alloc(sizeof(struct net_route_s), MEM_SCOPE_SYS);


  route->interface = if_get_by_name("eth0");
  IPV4_ADDR_SET(route->target, 0x0a020200);
  IPV4_ADDR_SET(route->mask, 0xffffff00);
  //IPV4_ADDR_SET(route->target, 0x0);
  //IPV4_ADDR_SET(route->mask, 0x0);
  //IPV4_ADDR_SET(route->router, 0x0a020201);
  route->type = ROUTETYPE_NET | ROUTETYPE_DIRECT;
  route_add(route);

  route = mem_alloc(sizeof(struct net_route_s), MEM_SCOPE_SYS);

  route->interface = if_get_by_name("eth1");
  IPV4_ADDR_SET(route->target, 0x0a020300);
  IPV4_ADDR_SET(route->mask, 0xffffff00);
  route->type = ROUTETYPE_NET | ROUTETYPE_DIRECT;
  route_add(route);

#ifdef CONFIG_NETWORK_PING
  struct net_addr_s	addr;
  struct ping_s		stat;

  //  IPV4_ADDR_SET(addr, 0x84e33f32);
  IPV4_ADDR_SET(addr, 0x0a020201);

  ping(&addr, 3, 56, &stat);

  printf("Total: %u transmitted, %u lost, %u error, min/avg/max = %u/%u/%u ms\n",
	 stat.total, stat.lost, stat.error, stat.min, stat.avg, stat.max);
#endif

#ifdef CONFIG_NETWORK_UDP
  //eval_server();
#endif

#if 0
  pthread_create(&th, NULL, err_test, NULL);
#endif

#if 0
  pthread_create(&th, NULL, tcp_test, NULL);
#endif

#ifdef CONFIG_NETWORK_NFS
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
#if 0
  asm volatile("	pushf						\n"
	       "	orl	$0x40000, (%esp)			\n"
	       "	popf						\n");
#endif

  pthread_create(&th, NULL, net_up, NULL);

  return 0;
}

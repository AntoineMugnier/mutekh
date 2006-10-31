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

#include <hexo/types.h>
#include <netinet/in.h>
#include <netinet/udp.h>

#include <netinet/nfs.h>

#include <semaphore.h>
#include <timer.h>

/*
 * NFS version 2 lightweight implementation
 * Multithreaded and read-only support
 *
 * Supported NFS operations:
 *
 *  + LOOKUP: get handle to a file
 *  + READ: read data from a file
 *
 */

CONTAINER_FUNC(static inline, rpcb, HASHLIST, rpcb, NOLOCK, id);
CONTAINER_KEY_FUNC(static inline, rpcb, HASHLIST, rpcb, NOLOCK, id);

/*
 * RPC timeout.
 */

TIMER_CALLBACK(rpc_timeout)
{
  struct rpcb_s	*rpcb = (struct rpcb_s *)pv;

  if (rpcb->data == NULL)
    {
      /* wake up */
      sem_post(&rpcb->sem);
    }
}

/*
 * Receive RPC replies.
 */

UDP_CALLBACK(rpc_callback)
{
  struct nfs_s	*server = (struct nfs_s *)pv;
  struct rpcb_s	*rpcb;
  uint_fast32_t	id;

  id = ((struct rpc_reply_s *)data)->id;

  if ((rpcb = rpcb_lookup(&server->rpc_blocks, id)) != NULL)
    {
      rpcb->data = data;
      rpcb->size = size;

      /* cancel the timeout */
      timer_cancel_event(&rpcb->timeout, 0);

      /* wake up */
      sem_post(&rpcb->sem);
    }
}

/*
 * Do RPC call.
 */

static inline error_t	do_rpc(struct nfs_s	*server,
			       uint_fast32_t	program,
			       uint_fast32_t	version,
			       uint_fast32_t	procedure,
			       void		**data,
			       size_t		*size)
{
  struct net_udp_addr_s	*dest;
  struct rpc_call_s	*call;
  struct rpc_reply_s	*reply;
  struct rpcb_s		rpcb;
  void			*pkt;
  void			*p;
  size_t		sz = *size;
  uint_fast8_t		tries = 0;

  /* build the call packet */
  if ((pkt = mem_alloc(sizeof (struct rpc_call_s) + sz, MEM_SCOPE_CONTEXT)) == NULL)
    return -ENOMEM;
  call = (struct rpc_call_s *)pkt;

  /* fill the request */
  rpcb.id = call->id = htonl(server->rpc_id++);
  call->type = htonl(MSG_CALL);
  call->rpcvers = htonl(2);
  call->prog = htonl(program);
  call->vers = htonl(version);
  call->proc = htonl(procedure);

  /* copy request body */
  p = (void *)(call + 1);
  memcpy(p, *data, sz);

  /* fill RPC block */
  sem_init(&rpcb.sem, 0, 0);
  rpcb.data = NULL;
  rpcb_push(&server->rpc_blocks, &rpcb);

 retry:

  /* send the packet */
  switch (program)
    {
      case PROGRAM_PORTMAP:
	dest = &server->portmap;
	break;
      case PROGRAM_MOUNTD:
	dest = &server->mountd;
	break;
      case PROGRAM_NFSD:
	dest = &server->nfsd;
	break;
      default:
	assert(!"bad RPC program id");
    }

  udp_send(&server->local, dest, pkt, sizeof (struct rpc_call_s) + sz);

  /* start timeout */
  rpcb.timeout.delay = RPC_TIMEOUT;
  rpcb.timeout.callback = rpc_timeout;
  rpcb.timeout.pv = &rpcb;
  timer_add_event(&timer_ms, &rpcb.timeout);

  /* wait reply */
  sem_wait(&rpcb.sem);

  /* get reply */
  reply = (struct rpc_reply_s *)rpcb.data;

  /* check for error */
  if (reply == NULL)
    {
      /* retry 3 times */
      if (tries++ < 3)
	goto retry;
      else
	{
	  printf("RPC timeout\n");

	  rpcb_remove(&server->rpc_blocks, &rpcb);
	  mem_free(pkt);

	  return -EAGAIN;
	}
    }

  rpcb_remove(&server->rpc_blocks, &rpcb);

  if (reply->rstatus || reply->astatus || reply->verifier)
    {
      mem_free(pkt);
      mem_free(reply);

      return -EINVAL;
    }

  mem_free(pkt);

  *data = rpcb.data;
  *size = rpcb.size;

  return 0;
}

/*
 * Initialize NFS connection by identifing remote ports using portmap.
 */

error_t		nfs_init(struct nfs_s	*server)
{
  static struct
  {
    uint32_t	mbz[4];
    uint32_t	program;
    uint32_t	version;
    uint32_t	protocol;
    uint32_t	mbz2;
  } mountd =
    {
      .mbz = { 0, 0, 0, 0 },
      .mbz2 = 0
    },
    nfsd =
    {
      .mbz = { 0, 0, 0, 0 },
      .mbz2 = 0
    };
  void		*req;
  size_t	size;
  uint32_t	*port;

  /* init RPC block list */
  rpcb_init(&server->rpc_blocks);

  /* set local port so UDP will determine a free port */
  server->local.port = htons(700); /* XXX */

  /* fill the portmap address */
  memcpy(&server->portmap.address, &server->address, sizeof (struct net_addr_s));
  server->portmap.port = htons(111);

  /* generate a RPC identifier */
  server->rpc_id = timer_get_tick(&timer_ms);

  /* register local rpc socket */
  udp_callback(&server->local, rpc_callback, server);

  /*
   * get mountd port.
   */

  mountd.program = htonl(PROGRAM_MOUNTD);
  mountd.version = htonl(1);
  mountd.protocol = htonl(IPPROTO_UDP);
  req = &mountd;
  size = sizeof (mountd);

  if (do_rpc(server, PROGRAM_PORTMAP, 2, PORTMAP_GETPORT, &req, &size))
    return -1;

  port = (uint32_t *)((struct rpc_reply_s *)req + 1);
  memcpy(&server->mountd.address, &server->address, sizeof (struct net_addr_s));
  server->mountd.port = htons(ntohl(*port));

  mem_free(req);

  /*
   * get nfsd port.
   */

  nfsd.program = htonl(PROGRAM_NFSD);
  nfsd.version = htonl(2);
  nfsd.protocol = htonl(IPPROTO_UDP);
  req = &nfsd;
  size = sizeof (nfsd);

  if (do_rpc(server, PROGRAM_PORTMAP, 2, PORTMAP_GETPORT, &req, &size))
    return -1;

  port = (uint32_t *)((struct rpc_reply_s *)req + 1);
  memcpy(&server->nfsd.address, &server->address, sizeof (struct net_addr_s));
  server->nfsd.port = htons(ntohl(*port));

  mem_free(req);

  return 0;
}

/*
 * Close a NFS connection, unmounting all exports.
 */

void			nfs_destroy(struct nfs_s	*server)
{
  nfs_umount(server);

  udp_close(&server->local);
  rpcb_destroy(&server->rpc_blocks);
}

/*
 * Mount an export.
 */

error_t			nfs_mount(struct nfs_s	*server,
				  char		*path,
				  nfs_handle_t	root)
{
  struct nfs_auth_s	*auth;
  void			*req, *to_free;
  size_t		path_len = strlen(path);
  uint32_t		*p;
  size_t		sz = sizeof (struct nfs_auth_s) + 4 + ALIGN_VALUE_UP(path_len, 4);

  /* allocate packet for the request */
  if ((to_free = req = mem_alloc(sz, MEM_SCOPE_CONTEXT)) == NULL)
    return -ENOMEM;
  auth = (struct nfs_auth_s *)req;

  /* insert auth info */
  auth->credential = htonl(1);
  auth->cred_len = htonl(20);
  auth->timestamp = 0;
  auth->hostname_len = 0;
  auth->uid = 0;
  auth->gid = 0;
  auth->aux_gid = 0;
  auth->verifier = 0;
  auth->verif_len = 0;

  /* copy path to the export */
  p = (uint32_t *)(auth + 1);
  *p = htonl(path_len);
  p[path_len / 4] = 0;
  memcpy(++p, path, path_len);

  /* RPC */
  if (do_rpc(server, PROGRAM_MOUNTD, 1, MOUNT_MOUNT, &req, &sz))
    {
      mem_free(to_free);

      return -EINVAL;
    }

  mem_free(to_free);

  /* read error code */
  p = (uint32_t *)((struct rpc_reply_s *)req + 1);
  if (*p)
    {
      mem_free(req);

      return *p;
    }

  /* read root handle */
  memcpy(root, ++p, sizeof (nfs_handle_t));

  mem_free(req);

  return 0;
}

/*
 * Unmount the mounted exports.
 */

error_t		nfs_umount(struct nfs_s	*server)
{
  struct nfs_auth_s	*auth;
  void			*req, *to_free;
  size_t		sz = sizeof (struct nfs_auth_s);

  /* allocate packet for the request */
  if ((to_free = req = mem_alloc(sz, MEM_SCOPE_CONTEXT)) == NULL)
    return -ENOMEM;
  auth = (struct nfs_auth_s *)req;

  /* insert auth info */
  auth->credential = htonl(1);
  auth->cred_len = htonl(20);
  auth->timestamp = 0;
  auth->hostname_len = 0;
  auth->uid = 0;
  auth->gid = 0;
  auth->aux_gid = 0;
  auth->verifier = 0;
  auth->verif_len = 0;

  /* RPC */
  if (do_rpc(server, PROGRAM_MOUNTD, 1, MOUNT_UMOUNTALL, &req, &sz))
    {
      mem_free(to_free);

      return -EINVAL;
    }

  mem_free(req);
  mem_free(to_free);

  return 0;
}

/*
 * Read data from a file.
 */

ssize_t		nfs_read(struct nfs_s	*server,
			 nfs_handle_t	handle,
			 void		*data,
			 off_t		offset,
			 size_t		size)
{
  struct nfs_auth_s	*auth;
  void			*req, *to_free;
  struct nfs_attr_s	*attr;
  uint32_t		*p;
  size_t		sz = sizeof (struct nfs_auth_s) + sizeof (nfs_handle_t) + 12;
  ssize_t		rd;

  /* allocate packet for the request */
  if ((to_free = req = mem_alloc(sz, MEM_SCOPE_CONTEXT)) == NULL)
    return -ENOMEM;
  auth = (struct nfs_auth_s *)req;

  /* insert auth info */
  auth->credential = htonl(1);
  auth->cred_len = htonl(20);
  auth->timestamp = 0;
  auth->hostname_len = 0;
  auth->uid = 0;
  auth->gid = 0;
  auth->aux_gid = 0;
  auth->verifier = 0;
  auth->verif_len = 0;

  /* copy root & path to the export */
  p = (uint32_t *)(auth + 1);
  memcpy(p, handle, sizeof (nfs_handle_t));
  p += sizeof (nfs_handle_t) / 4;
  *p++ = htonl(offset);
  *p++ = htonl(size);
  *p = 0;

  /* RPC */
  if (do_rpc(server, PROGRAM_NFSD, 2, NFS_READ, &req, &sz))
    {
      mem_free(to_free);

      return -EINVAL;
    }

  /* copy data */
  attr = (struct nfs_attr_s *)((struct rpc_reply_s *)req + 1);
  p = (uint32_t *)(attr + 1);
  p++;
  if ((rd = *p))
    {
      rd = ntohl(rd);

      memcpy(data, ++p, rd);
    }

  mem_free(req);
  mem_free(to_free);

  return rd;
}

/*
 * Look for a file.
 */

error_t		nfs_lookup(struct nfs_s		*server,
			   char			*path,
			   nfs_handle_t		directory,
			   nfs_handle_t		handle,
			   struct nfs_attr_s	*stat)
{
  struct nfs_auth_s	*auth;
  void			*req, *to_free;
  size_t		path_len = strlen(path);
  struct nfs_attr_s	*attr;
  uint32_t		*p;
  size_t		sz = sizeof (struct nfs_auth_s) + 4 + sizeof (nfs_handle_t) + ALIGN_VALUE_UP(path_len, 4);

  /* allocate packet for the request */
  if ((to_free = req = mem_alloc(sz, MEM_SCOPE_CONTEXT)) == NULL)
    return -ENOMEM;
  auth = (struct nfs_auth_s *)req;

  /* insert auth info */
  auth->credential = htonl(1);
  auth->cred_len = htonl(20);
  auth->timestamp = 0;
  auth->hostname_len = 0;
  auth->uid = 0;
  auth->gid = 0;
  auth->aux_gid = 0;
  auth->verifier = 0;
  auth->verif_len = 0;

  /* copy root & path to the export */
  p = (uint32_t *)(auth + 1);
  memcpy(p, directory, sizeof (nfs_handle_t));
  p += sizeof (nfs_handle_t) / 4;
  *p = htonl(path_len);
  p[path_len / 4] = 0;
  memcpy(++p, path, path_len);

  /* RPC */
  if (do_rpc(server, PROGRAM_NFSD, 2, NFS_LOOKUP, &req, &sz))
    {
      mem_free(to_free);

      return -EINVAL;
    }

  mem_free(to_free);

  /* read error code */
  p = (uint32_t *)((struct rpc_reply_s *)req + 1);
  if (*p)
    {
      mem_free(req);

      return *p;
    }

  /* read file handle */
  memcpy(handle, ++p, sizeof (nfs_handle_t));

  /* read attributes */
  attr = (struct nfs_attr_s *)(p + 1);

  if (stat != NULL)
    memcpy(stat, attr, sizeof (struct nfs_attr_s));

  mem_free(req);

  return 0;
}


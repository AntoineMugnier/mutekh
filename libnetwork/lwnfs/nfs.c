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
 *  + MOUNT: mount an export
 *  + UMOUNT: unmount an export
 *  + UMOUNTALL: unmount all exports
 *  + GETATTR: get attributes of a file
 *  + SETATTR: set attributes of a file
 *  + LOOKUP: get handle to a file
 *  + READ: read data from a file
 *  + STATFS: general information on filesystem
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
 * Endianness translation
 */

static inline void	attr_endian(struct nfs_attr_s	*attr)
{

}

static inline void	sattr_endian(struct nfs_user_attr_s	*attr)
{

}

static inline void	stat_endian(struct nfs_statfs_s	*stat)
{
  stat->transfer_unit = ntohl(stat->transfer_unit);
  stat->block_size = ntohl(stat->block_size);
  stat->blocks = ntohl(stat->blocks);
  stat->blocks_free = ntohl(stat->blocks_free);
  stat->blocks_avail = ntohl(stat->blocks_avail);
}

/*
 * Do RPC call.
 */

static error_t		do_rpc(struct nfs_s	*server,
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

  if (udp_send(server->conn, dest, pkt, sizeof (struct rpc_call_s) + sz) < 0)
    {
      rpcb_remove(&server->rpc_blocks, &rpcb);
      mem_free(pkt);

      return -EAGAIN;
    }

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
  IPV4_ADDR_SET(server->local.address, INADDR_ANY);
  server->local.port = htons(700) /* XXX */;

  /* fill the portmap address */
  memcpy(&server->portmap.address, &server->address, sizeof (struct net_addr_s));
  server->portmap.port = htons(111);

  /* generate a RPC identifier */
  server->rpc_id = timer_get_tick(&timer_ms);

  /* register local rpc socket */
  server->conn = NULL;
  if (udp_bind(&server->conn, &server->local, rpc_callback, server) < 0)
    return -1;

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
  nfs_umount_all(server);

  udp_close(server->conn);
  rpcb_destroy(&server->rpc_blocks);
}

/*
 * RPC to mount deamon
 */

static error_t		nfs_mountd(struct nfs_s	*server,
				   void		*call,
				   size_t	*size,
				   uint32_t	action,
				   void		**reply)
{
  struct nfs_auth_s	*auth;
  void			*buf;
  void			*req;

  /* allocate packet for the request */
  if ((buf = req = mem_alloc(sizeof (struct nfs_auth_s) + *size, MEM_SCOPE_CONTEXT)) == NULL)
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

  memcpy(auth + 1, call, *size);
  *size += sizeof (struct nfs_auth_s);

  /* RPC */
  if (do_rpc(server, PROGRAM_MOUNTD, 1, action, &req, size))
    {
      mem_free(buf);

      return -EINVAL;
    }

  mem_free(buf);
  *reply = req;

  return 0;
}

/*
 * Mount an export.
 */

error_t			nfs_mount(struct nfs_s	*server,
				  char		*path,
				  nfs_handle_t	root)
{
  struct nfs_dirop_s	*dir;
  struct nfs_status_s	*stat;
  struct rpc_reply_s	*reply;
  error_t		err;
  size_t		path_len;
  size_t		sz;

  path_len = strlen(path);
  sz = sizeof(uint32_t) + ALIGN_VALUE_UP(path_len, 4);

  if ((dir = mem_alloc(sz, MEM_SCOPE_CONTEXT)) == NULL)
    return -ENOMEM;

  /* copy path to the export */
  dir->path_len = htonl(path_len);
  dir->path[path_len] = 0;
  memcpy(dir->path, path, path_len);

  /* call mountd */
  err = nfs_mountd(server, dir, &sz, MOUNT_MOUNT, (void *)&reply);
  mem_free(dir);

  if (err)
    return err;

  /* read error code */
  stat = (struct nfs_status_s *)(reply + 1);

  if (sz < sizeof (uint32_t) || ((err = stat->status)) ||
      sz < sizeof (uint32_t) + sizeof (nfs_handle_t))
    goto leave;

  /* read root handle */
  memcpy(root, stat->u.handle, sizeof (nfs_handle_t));

  mem_free(reply);
  return 0;

 leave:
  mem_free(reply);
  return err ? err : -EINVAL;
}

/*
 * Unmount an export.
 */

error_t			nfs_umount(struct nfs_s	*server,
				   char		*path)
{
  struct nfs_dirop_s	*dir;
  struct rpc_reply_s	*reply;
  error_t		err;
  size_t		path_len;
  size_t		sz;

  path_len = strlen(path);
  sz = sizeof(uint32_t) + ALIGN_VALUE_UP(path_len, 4);

  if ((dir = mem_alloc(sz, MEM_SCOPE_CONTEXT)) == NULL)
    return -ENOMEM;

  /* copy path to the export */
  dir->path_len = htonl(path_len);
  dir->path[path_len] = 0;
  memcpy(dir->path, path, path_len);

  /* call mountd */
  err = nfs_mountd(server, dir, &sz, MOUNT_UMOUNT, (void *)&reply);

  mem_free(dir);
  mem_free(reply);

  return 0;
}

/*
 * Unmount the mounted exports.
 */

error_t		nfs_umount_all(struct nfs_s	*server)
{
  struct rpc_reply_s	*reply;
  size_t		sz = 0;

  nfs_mountd(server, NULL, &sz, MOUNT_UMOUNTALL, (void *)&reply);

  mem_free(reply);

  return 0;
}

/*
 * RPC to nfs daemon
 */

static error_t		nfs_nfsd(struct nfs_s	*server,
				 void		*call,
				 size_t		*size,
				 uint32_t	action,
				 void		**reply)
{
  struct nfs_auth_s	*auth;
  void			*buf;
  void			*req;

  /* allocate packet for the request */
  if ((buf = req = mem_alloc(sizeof (struct nfs_auth_s) + *size, MEM_SCOPE_CONTEXT)) == NULL)
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

  memcpy(auth + 1, call, *size);
  *size += sizeof (struct nfs_auth_s);

  /* RPC */
  if (do_rpc(server, PROGRAM_NFSD, 2, action, &req, size))
    {
      mem_free(buf);

      return -EINVAL;
    }

  mem_free(buf);
  *reply = req;

  return 0;
}

/*
 * Read data from a file.
 */

ssize_t				nfs_read(struct nfs_s	*server,
					 nfs_handle_t	handle,
					 void		*data,
					 off_t		offset,
					 size_t		size)
{
  struct nfs_request_handle_s	*req;
  struct rpc_reply_s		*reply;
  struct nfs_status_s		*status;
  size_t			sz;
  error_t			err;

  /* allocate packet for the request */
  sz = sizeof (nfs_handle_t) + sizeof (struct nfs_read_s);
  if ((req = mem_alloc(sz, MEM_SCOPE_CONTEXT)) == NULL)
    return -ENOMEM;

  /* copy root & path to the export */
  memcpy(req->handle, handle, sizeof (nfs_handle_t));
  req->u.read.offset = htonl(offset);
  req->u.read.count = htonl(size);
  req->u.read.__unused = 0;

  /* call mountd */
  err = nfs_nfsd(server, req, &sz, NFS_READ, (void *)&reply);
  mem_free(req);

  if (err)
    return err;

  status = (struct nfs_status_s *)(reply + 1);

  if (sz < sizeof (uint32_t) || (err = status->status) != NFS_OK ||
      sz < sizeof (struct nfs_attr_s) + 2 * sizeof (uint32_t))
    goto leave;

  /* copy data */
  if ((sz = status->u.attr_data.len))
    {
      sz = ntohl(sz);
      memcpy(data, status->u.attr_data.data, sz);
    }

  mem_free(reply);
  return sz;

 leave:
  mem_free(reply);
  return err ? err : -EINVAL;
}

/*
 * Look for a file.
 */

error_t				nfs_lookup(struct nfs_s		*server,
					   char			*path,
					   nfs_handle_t		directory,
					   nfs_handle_t		handle,
					   struct nfs_attr_s	*stat)
{
  struct nfs_request_handle_s	*req;
  struct rpc_reply_s		*reply;
  struct nfs_status_s		*status;
  size_t			sz;
  error_t			err;
  size_t			path_len;

  path_len = strlen(path);

  /* allocate packet for the request */
  sz = sizeof (nfs_handle_t) + sizeof (uint32_t) + ALIGN_VALUE_UP(path_len, 4);
  if ((req = mem_alloc(sz, MEM_SCOPE_CONTEXT)) == NULL)
    return -ENOMEM;

  /* copy root & path to the entity */
  memcpy(req->handle, directory, sizeof (nfs_handle_t));
  req->u.dirop.path_len = htonl(path_len);
  memcpy(req->u.dirop.path, path, path_len);
  req->u.dirop.path[path_len] = 0;

  /* call mountd */
  err = nfs_nfsd(server, req, &sz, NFS_LOOKUP, (void *)&reply);
  mem_free(req);

  if (err)
    return err;

  status = (struct nfs_status_s *)(reply + 1);

  if (sz < sizeof (uint32_t) || (err = status->status) != NFS_OK ||
      sz < sizeof (uint32_t) + sizeof (struct nfs_attr_s) + sizeof (nfs_handle_t))
    goto leave;

  /* copy stat if neeeded */
  if (stat != NULL)
    {
      memcpy(stat, &status->u.handle_attr.attr, sizeof (struct nfs_attr_s));
      attr_endian(stat);
    }

  /* copy handle */
  memcpy(handle, status->u.handle_attr.handle, sizeof (nfs_handle_t));

  mem_free(reply);
  return 0;

 leave:
  mem_free(reply);
  return err ? err : -EINVAL;
}

/*
 * Get filesystem statistics
 */

error_t				nfs_statfs(struct nfs_s		*server,
					   nfs_handle_t		root,
					   struct nfs_statfs_s	*stats)
{
  struct nfs_request_handle_s	*req;
  struct rpc_reply_s		*reply;
  struct nfs_status_s		*status;
  size_t			sz;
  error_t			err;

  /* allocate packet for the request */
  sz = sizeof (nfs_handle_t);
  if ((req = mem_alloc(sz, MEM_SCOPE_CONTEXT)) == NULL)
    return -ENOMEM;

  /* copy root & path to the entity */
  memcpy(req->handle, root, sizeof (nfs_handle_t));

  /* call mountd */
  err = nfs_nfsd(server, req, &sz, NFS_STATFS, (void *)&reply);
  mem_free(req);

  if (err)
    return err;

  status = (struct nfs_status_s *)(reply + 1);

  printf("%P\n", status, sizeof (uint32_t) + sizeof (struct nfs_statfs_s));

  if (sz < sizeof (uint32_t) || (err = status->status) != NFS_OK ||
      sz < sizeof (uint32_t) + sizeof (struct nfs_statfs_s))
    goto leave;

  memcpy(stats, &status->u.statfs, sizeof (struct nfs_statfs_s));
  stat_endian(stats);

  mem_free(reply);
  return 0;

 leave:
  mem_free(reply);
  return err ? err : -EINVAL;
}

/*
 * Get attributes of a file
 */

error_t				nfs_getattr(struct nfs_s	*server,
					    nfs_handle_t	handle,
					    struct nfs_attr_s	*stat)
{
  struct nfs_request_handle_s	*req;
  struct rpc_reply_s		*reply;
  struct nfs_status_s		*status;
  size_t			sz;
  error_t			err;

  /* allocate packet for the request */
  sz = sizeof (nfs_handle_t);
  if ((req = mem_alloc(sz, MEM_SCOPE_CONTEXT)) == NULL)
    return -ENOMEM;

  /* copy root & path to the entity */
  memcpy(req->handle, handle, sizeof (nfs_handle_t));

  /* call mountd */
  err = nfs_nfsd(server, req, &sz, NFS_GETATTR, (void *)&reply);
  mem_free(req);

  if (err)
    return err;

  status = (struct nfs_status_s *)(reply + 1);

  if (sz < sizeof (uint32_t) || (err = status->status) != NFS_OK ||
      sz < sizeof (uint32_t) + sizeof (struct nfs_attr_s))
    goto leave;

  memcpy(stat, &status->u.attr, sizeof (struct nfs_attr_s));
  attr_endian(stat);

  mem_free(reply);
  return 0;

 leave:
  mem_free(reply);
  return err ? err : -EINVAL;
}

/*
 * Set attributes of a file
 */

error_t				nfs_setattr(struct nfs_s		*server,
					    nfs_handle_t		handle,
					    struct nfs_user_attr_s	*stat,
					    struct nfs_attr_s		*after)
{
  struct nfs_request_handle_s	*req;
  struct rpc_reply_s		*reply;
  struct nfs_status_s		*status;
  size_t			sz;
  error_t			err;

  /* allocate packet for the request */
  sz = sizeof (nfs_handle_t) + sizeof (struct nfs_user_attr_s);
  if ((req = mem_alloc(sz, MEM_SCOPE_CONTEXT)) == NULL)
    return -ENOMEM;

  /* copy root & path to the entity */
  memcpy(req->handle, handle, sizeof (nfs_handle_t));
  sattr_endian(stat);
  memcpy(&req->u.sattr, stat, sizeof (struct nfs_user_attr_s));

  /* call mountd */
  err = nfs_nfsd(server, req, &sz, NFS_SETATTR, (void *)&reply);
  mem_free(req);

  if (err)
    return err;

  status = (struct nfs_status_s *)(reply + 1);

  if (sz < sizeof (uint32_t) || (err = status->status) != NFS_OK ||
      sz < sizeof (uint32_t) + sizeof (struct nfs_attr_s))
    goto leave;

  memcpy(stat, &status->u.attr, sizeof (struct nfs_attr_s));

  mem_free(reply);
  return 0;

 leave:
  mem_free(reply);
  return err ? err : -EINVAL;
}

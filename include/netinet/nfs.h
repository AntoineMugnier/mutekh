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

#ifndef NETINET_NFS_H_
#define NETINET_NFS_H_

#include <hexo/types.h>
#include <netinet/libudp.h>
#include <hexo/gpct_platform_hexo.h>
#include <hexo/gpct_lock_hexo.h>
#include <gpct/cont_hashlist.h>

#include <semaphore.h>
#include <timer.h>

/*
 * RPC timeout.
 */

#define RPC_TIMEOUT	3000 /* 3 seconds */

/*
 * Programs ID.
 */

#define PROGRAM_PORTMAP	100000
#define PROGRAM_MOUNTD	100005
#define PROGRAM_NFSD	100003

/*
 * Message type.
 */

#define MSG_CALL	0
#define MSG_REPLY	1

/*
 * Portmap calls.
 */

#define PORTMAP_GETPORT	3

/*
 * mountd calls.
 */

#define MOUNT_NULL	0
#define MOUNT_MOUNT	1
#define MOUNT_UMOUNT	3
#define MOUNT_UMOUNTALL	4

/*
 * nfsd calls
 */

#define NFS_NULL	0
#define NFS_GETATTR	1
#define NFS_SETATTR	2
#define NFS_ROOT	3
#define NFS_LOOKUP	4
#define NFS_READLINK	5
#define NFS_READ	6
#define NFS_WRITECACHE	7
#define NFS_WRITE	8
#define NFS_CREATE	9
#define NFS_REMOVE	10
#define NFS_RENAME	11
#define NFS_LINK	12
#define NFS_SYMLINK	13
#define NFS_MKDIR	14
#define NFS_RMDIR	15
#define NFS_READDIR	16
#define NFS_STATFS	17

/*
 * File handles.
 */

#define FHSIZE	32

typedef uint8_t nfs_handle_t[FHSIZE];

/*
 * NFS file types.
 */

#define NFNON	0
#define NFREG	1
#define NFDIR	2
#define NFBLK	3
#define NFCHR	4
#define NFLNK	5

/*
 * NFS errors.
 */

#define NFS_OK			0
#define NFSERR_PERM		1
#define NFSERR_NOENT		2
#define NFSERR_IO		5
#define NFSERR_NXIO		6
#define NFSERR_ACCES		13
#define NFSERR_EXIST		17
#define NFSERR_NODEV		19
#define NFSERR_NOTDIR		20
#define NFSERR_ISDIR		21
#define NFSERR_FBIG		27
#define NFSERR_NOSPC		28
#define NFSERR_ROFS		30
#define NFSERR_NAMETOOLONG	63
#define NFSERR_NOTEMPTY		66
#define NFSERR_DQUOT		69
#define NFSERR_STALE		70
#define NFSERR_WFLUSH		99

/*
 * RPC block.
 */

struct					rpcb_s
{
  uint_fast32_t				id;
  struct timer_event_s			timeout;
  void					*data;
  size_t				size;
  sem_t					sem;

  CONTAINER_ENTRY_TYPE(HASHLIST)	list_entry;
};

CONTAINER_TYPE(rpcb, HASHLIST, struct rpcb_s, NOLOCK, NOOBJ, list_entry, 64);
CONTAINER_KEY_TYPE(rpcb, SCALAR, id);

/*
 * NFS connection descriptor.
 */

struct			nfs_s
{
  struct net_udp_addr_s	local;
  struct net_addr_s	address;	/* server address */
  struct net_udp_addr_s	portmap;	/* portmap server address */
  struct net_udp_addr_s	mountd;		/* mountd server address */
  struct net_udp_addr_s	nfsd;		/* nfsd server address */
  uint_fast32_t		rpc_id;		/* rpc sequence id */

  rpcb_root_t		rpc_blocks;
};

/*
 * NFS auth.
 */

struct			nfs_auth_s
{
  uint32_t		credential;
  uint32_t		cred_len;
  uint32_t		timestamp;
  uint32_t		hostname_len;
  uint32_t		uid;
  uint32_t		gid;
  uint32_t		aux_gid;
  uint32_t		verifier;
  uint32_t		verif_len;
};

/*
 * NFS file attributes
 */

struct			nfs_attr_s
{
  uint32_t		ftype;
  uint32_t		mode;
  uint32_t		nlink;
  uint32_t		uid;
  uint32_t		gid;
  uint32_t		size;
  uint32_t		blocksize;
  uint32_t		rdev;
  uint32_t		blocks;
  uint32_t		fsid;
  uint32_t		fileid;
  uint64_t		atime;
  uint64_t		mtime;
  uint64_t		ctime;
};

/*
 * RPC call.
 */

struct			rpc_call_s
{
  uint32_t		id;
  uint32_t		type;
  uint32_t		rpcvers;
  uint32_t		prog;
  uint32_t		vers;
  uint32_t		proc;
};

/*
 * RPC reply.
 */

struct			rpc_reply_s
{
  uint32_t		id;
  uint32_t		type;
  uint32_t		rstatus;
  uint32_t		verifier;
  uint32_t		v2;
  uint32_t		astatus;
};

/*
 * Prototypes.
 */

error_t		nfs_init(struct nfs_s	*server);
void		nfs_destroy(struct nfs_s	*server);
error_t		nfs_mount(struct nfs_s	*server,
			  char		*path,
			  nfs_handle_t	root);
error_t		nfs_umount(struct nfs_s	*server);
ssize_t		nfs_read(struct nfs_s	*server,
			 nfs_handle_t	handle,
			 void		*data,
			 off_t		offset,
			 size_t		size);
error_t		nfs_lookup(struct nfs_s		*server,
			   char			*path,
			   nfs_handle_t		directory,
			   nfs_handle_t		handle,
			   struct nfs_attr_s	*stat);

#endif

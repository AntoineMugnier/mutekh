#include <stdio.h>
#include <mutek/printk.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>

#include <vfs/vfs.h>

#include "my_rand.h"

#define dprintk(...) do{}while(0)

typedef void (action_t)();

static const char chtab[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ-_0123456789.";

static void create_random_name(char *s, size_t size)
{
	size = (my_rand() % size) + 1;
	size_t i;

	for ( i=0; i<size; ++i ) {
		s[i] = chtab[my_rand() % (sizeof(chtab)-1)];
	}
	s[i] = 0;
}

static error_t get_random_name(struct vfs_node_s *base, char *name)
{
    struct vfs_file_s *dir;
    struct vfs_dirent_s dirent;
    struct vfs_stat_s stat;

	memset(name, 0, CONFIG_VFS_NAMELEN);

	error_t err = vfs_stat(vfs_get_root(), vfs_get_cwd(), ".", &stat);
	if ( ! stat.size )
		return -EIO;

	size_t n = my_rand() % stat.size;

	err = vfs_open(vfs_get_root(), base, ".",
				   VFS_OPEN_READ | VFS_OPEN_DIR, &dir);
    if (err) {
		return -EIO;
	}

    while ( (vfs_file_read(dir, &dirent, sizeof(dirent)) == sizeof(dirent))
			&& n-- )
		;
	memcpy(name, dirent.name, CONFIG_VFS_NAMELEN);

    vfs_file_close(dir);
	return 0;
}

void action_cwd()
{
	struct vfs_node_s *node;
	struct vfs_node_s *base = vfs_get_cwd();
	error_t err;
	char name[CONFIG_VFS_NAMELEN];

	if ( my_rand() > 0xd000 ) {
		name[0] = '.';
		name[2] = '.';
		err = 0;
	} else {
		err = get_random_name(base, name);
		if ( err ) {
			err = 0;
			name[0] = '/';
			name[1] = 0;
		}
	}

	dprintk("%s \"%s\"...\n", __FUNCTION__, name);

	err = vfs_lookup(vfs_get_root(), base, name, &node);
	if ( err )
		return;
	if ( node->type == VFS_NODE_DIR )
		vfs_set_cwd(node);
	vfs_node_refdrop(node);
}

void action_mkdir()
{
	char name[CONFIG_VFS_NAMELEN+1];
	create_random_name(name, CONFIG_VFS_NAMELEN);

	dprintk("%s \"%s\"...\n", __FUNCTION__, name);

	struct vfs_node_s *node;
	error_t err = vfs_create(vfs_get_root(), vfs_get_cwd(),
							 name, VFS_NODE_DIR, &node);
	if (err == 0) {
		vfs_node_refdrop(node);
	} else {
		dprintk("%s error %s\n", __FUNCTION__, strerror(err));
	}
}

void action_create_file()
{
	char name[CONFIG_VFS_NAMELEN+1];
	create_random_name(name, CONFIG_VFS_NAMELEN);

	dprintk("%s \"%s\"...\n", __FUNCTION__, name);

	struct vfs_file_s *file;
	error_t err = vfs_open(vfs_get_root(), vfs_get_cwd(),
						   name, VFS_OPEN_WRITE|VFS_OPEN_CREATE, &file);
	if (err)
		goto error_open;

	vfs_file_write(file, action_create_file, 128);
	vfs_file_close(file);
	return;

  error_open:
	dprintk("%s error %s\n", __FUNCTION__, strerror(err));
}

void action_rm()
{
	char name[CONFIG_VFS_NAMELEN];
	error_t err = get_random_name(vfs_get_cwd(), name);

	dprintk("%s \"%s\"...\n", __FUNCTION__, name);

	if ( !err )
		vfs_unlink(vfs_get_root(), vfs_get_cwd(), name);
}

void action_rmrf_inner(struct vfs_node_s *_cwd, const char *name)
{
	struct vfs_node_s *cwd = vfs_node_refnew(_cwd);
	struct vfs_stat_s stat;
	error_t err;

	err = vfs_stat(vfs_get_root(), cwd, name, &stat);
	if ( err )
		goto end;

	if ( stat.type == VFS_NODE_DIR ) {
		struct vfs_node_s *node;
		if ( vfs_lookup(vfs_get_root(), cwd, name, &node) == 0 ) {
			assert(node);

			while ( 1 ) {
				struct vfs_file_s *dir;
				struct vfs_dirent_s dirent;
		
				err = vfs_open(vfs_get_root(), node, ".",
							   VFS_OPEN_READ | VFS_OPEN_DIR, &dir);
				if ( err )
					break;
				ssize_t len = vfs_file_read(dir, &dirent, sizeof(dirent));
				vfs_file_close(dir);
				if ( !len )
					break;

				action_rmrf_inner(node, dirent.name);
			}

			vfs_node_refdrop(node);
		}
	}
	vfs_unlink(vfs_get_root(), cwd, name);

  end:
	vfs_node_refdrop(cwd);
}

void action_rmrf()
{
	char name[CONFIG_VFS_NAMELEN];
	error_t err = get_random_name(vfs_get_cwd(), name);

	dprintk("%s \"%s\"...\n", __FUNCTION__, name);

	if ( !strcmp(name, "..") )
		return;

//	vfs_dump(vfs_get_cwd());

	if ( !err )
		action_rmrf_inner(vfs_get_cwd(), name);
}

void action_mount()
{

}

void action_umount()
{

}

void action_ls()
{
	dprintk("%s...\n", __FUNCTION__);

    struct vfs_file_s *dir;
    struct vfs_dirent_s dirent;

	error_t err = vfs_open(vfs_get_root(), vfs_get_cwd(), ".",
						   VFS_OPEN_READ | VFS_OPEN_DIR, &dir);
    if (err) {
		dprintk("%s error %s\n", __FUNCTION__, strerror(err));
		return;
	}

    while ( vfs_file_read(dir, &dirent, sizeof(dirent)) == sizeof(dirent) )
        printk("%s [%s] %d\n", dirent.name, dirent.type == VFS_NODE_DIR ? "dir" : "reg", dirent.size);

    vfs_file_close(dir);
}

action_t * const actions[] =
{
	action_cwd,
	action_mkdir,
	action_create_file,
	action_rm,
	action_rmrf,
/* 	action_mount, */
/* 	action_umount, */
	action_ls,
	0,
};

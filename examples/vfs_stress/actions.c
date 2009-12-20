#include <stdio.h>
#include <mutek/printk.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>

#include <vfs/vfs.h>
#include <vfs/vfs-private.h>

#include "my_rand.h"

//#define dprintk(...) do{}while(0)
#define dprintk(...) printk(__VA_ARGS__)

#include "cwd.h"

typedef void (action_t)();

static const char chtab[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ-_0123456789.";

static void create_random_name(char *s, size_t size)
{
    memset(s, 0, size);
    --size;
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
    bool_t done = 0;

	memset(name, 0, VFS_MAX_NAME_LENGTH);

	error_t err = vfs_opendir(vfs_get_cwd(), ".",
                              0, &dir);
    if (err) {
		return -EIO;
	}

    for (;;) {
        error_t err = vfs_readdir(dir, &dirent);
        if ( err )
            break;
        if ( !done || my_rand() > 0xc000 ) {
            memcpy(name, dirent.d_name, VFS_MAX_NAME_LENGTH);
            done = 1;
        }
    }

    vfs_closedir(dir);
	return done ? 0 : -EUNKNOWN;
}

static void post_print(struct vfs_node_s *node)
{
    struct vfs_node_s *parent = node->n_parent;
	if ( parent ) {
        if ( parent != node )
            post_print(parent);
		printk("/");
	}
	printk("%s", node->n_name);
}

void action_cwd()
{
	struct vfs_node_s *node = NULL;
	struct vfs_node_s *base = vfs_get_cwd();
	error_t err;
	char name[VFS_MAX_NAME_LENGTH];

	if ( my_rand() > 0xd000 ) {
		name[0] = '.';
		name[1] = '.';
        name[2] = 0;
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

	err = vfs_chdir(base, name, &node);
	if ( err )
		return;

    vfs_set_cwd(node);
}

void action_mkdir()
{
	char name[VFS_MAX_NAME_LENGTH];
	create_random_name(name, VFS_MAX_NAME_LENGTH);

	dprintk("%s \"%s\"...\n", __FUNCTION__, name);

	error_t err = vfs_mkdir(vfs_get_cwd(), name, 0);
	if (err) {
		dprintk("%s error %s\n", __FUNCTION__, strerror(err));
	}
}

void action_create_file()
{
	char name[VFS_MAX_NAME_LENGTH];
	create_random_name(name, VFS_MAX_NAME_LENGTH);

	dprintk("%s \"%s\"...\n", __FUNCTION__, name);

	struct vfs_file_s *file = NULL;
	error_t err = vfs_open(vfs_get_cwd(),
						   name, VFS_O_WRONLY|VFS_O_CREATE, 0644, &file);
	if (err)
		goto error_open;

	vfs_write(file, (uint8_t *)action_create_file, 128);
	vfs_close(file);
	return;

  error_open:
	dprintk("%s error %s\n", __FUNCTION__, strerror(err));
}

void action_rm()
{
	char name[VFS_MAX_NAME_LENGTH];
	error_t err = get_random_name(vfs_get_cwd(), name);

	dprintk("%s \"%s\"...\n", __FUNCTION__, name);

	if ( !err )
		vfs_unlink(vfs_get_cwd(), name);
}

error_t action_rmrf_inner(struct vfs_node_s *_cwd, char *name)
{
    vfs_node_up(_cwd);
	struct vfs_node_s *cwd = _cwd;
	struct vfs_stat_s stat = {0};
	error_t err;

	err = vfs_stat(cwd, name, &stat);
    dprintk("rmrf stat 'name': %s\n", name, strerror(err));
	if ( err )
		goto end;

	if ( stat.attr & VFS_DIR ) {
        dprintk(" is directory\n");
        
		struct vfs_node_s *node = NULL;
        struct vfs_node_s *inner;
        err = vfs_chdir(cwd, name, &inner);
		if ( !err ) {
			while ( 1 ) {
				struct vfs_file_s *dir = NULL;
				struct vfs_dirent_s dirent;
		
				err = vfs_opendir(node, ".", 0, &dir);
				if ( err )
					break;

				err = vfs_readdir(dir, &dirent);
				vfs_closedir(dir);
				if ( err )
					break;

                if ( dirent.d_name[0] != '.' ) {
                    err = action_rmrf_inner(node, dirent.d_name);
                    if ( err )
                        break;
                }
			}
            err = vfs_chdir(cwd, "..", &inner);
		}
	}
	err = vfs_unlink(cwd, name);
    dprintk(" unlink '%s': %s\n", name, strerror(err));

  end:
	vfs_node_down(cwd);
    return err;
}

void action_rmrf()
{
	char name[VFS_MAX_NAME_LENGTH];
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

#if 0
void action_ls()
{
	dprintk("%s...\n", __FUNCTION__);

    struct vfs_file_s *dir = NULL;
    struct vfs_dirent_s dirent;

	error_t err = vfs_open(vfs_get_root(), vfs_get_cwd(), ".",
						   VFS_OPEN_READ | VFS_OPEN_DIR, &dir);
    if (err) {
		dprintk("%s error %s\n", __FUNCTION__, strerror(err));
		return;
	}

    while ( vfs_file_read(dir, &dirent, sizeof(dirent)) == sizeof(dirent) ) {
//        printk("%s [%s] %d\n", dirent.name, dirent.type == VFS_NODE_DIR ? "dir" : "reg", dirent.size);
    }

    vfs_file_close(dir);
}
#endif

action_t * const actions[] =
{
	action_cwd,
	action_mkdir,
	action_create_file,
	action_rm,
	action_rmrf,
/* 	action_mount, */
/* 	action_umount, */
//	action_ls,
	0,
};

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

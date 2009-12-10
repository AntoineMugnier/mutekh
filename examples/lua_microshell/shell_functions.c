
#include <stdio.h>
#include <errno.h>
#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <lua/lua.h>
#include <vfs/vfs.h>

#include <drivers/fs/ramfs/ramfs.h>
#include <drivers/fs/iso9660/iso9660.h>
#include <drivers/device/enum/fdt/enum-fdt.h>

static
struct vfs_node_s * vfs_init()
{
	struct vfs_fs_s *root_fs;

    printk("init vfs... ");
	ramfs_open(&root_fs);
    printk("ok\n");

    struct vfs_node_s *root_node;
    vfs_create_root(root_fs, &root_node);

	return root_node;
}

static inline const char * lua_getstringopt(lua_State *st, size_t n, size_t *size)
{
    const char *s = lua_tolstring(st, n, size);
    if (!s) 
        lua_error(st);
    return s;
}

int ls(lua_State *st)
{
    if (lua_gettop(st) <= 0)
        lua_pushstring(st, ".");

    const char *pathname = lua_getstringopt(st, 1, NULL);

    struct vfs_file_s *dir;
    struct vfs_dirent_s dirent;

	error_t err = vfs_open(vfs_get_root(), vfs_get_cwd(), pathname, VFS_OPEN_DIR | VFS_OPEN_READ, &dir);
    if (err)
    {
        printk("Error: %s\n", strerror(err));
        return -1;
    }

    while ( vfs_file_read(dir, &dirent, sizeof(dirent)) == sizeof(dirent) )
        printk("%s [%s] %d\n", dirent.name, dirent.type == VFS_NODE_DIR ? "dir" : "reg", dirent.size);

    vfs_file_close(dir);

    return 0;
}

int cat(lua_State *st)
{
    unsigned int i;

    for (i = 1; i <= lua_gettop(st); i++)
    {
        switch (lua_type(st, i))
        {
            case LUA_TSTRING:
                {
                    FILE* f;
					const char *pathname = lua_getstringopt(st, 1, NULL);
					char buffer[32];

                    if ((f = fopen(pathname, "r")) == NULL)
                    {
                        printk("error '%s': %s\n", pathname, strerror(errno));
                        break;
                    }
                    while (fgets(buffer, sizeof(buffer), f) > 0)
                        printk("%s", buffer);

                    fclose(f);
                    break;
                }
            default:
                printk("bad argument\n");
                break;
        }
    }

    return 0;
}

int cd(lua_State *st)
{
    if (lua_gettop(st) <= 0)
        lua_pushstring(st, ".");

    const char *pathname = lua_getstringopt(st, 1, NULL);

	struct vfs_node_s *node;
    error_t err = vfs_lookup(vfs_get_root(), vfs_get_cwd(), pathname, &node);
    if(err)
    {
        printk("Error: %d\n", err);
        return -1;
    }

    struct vfs_stat_s stat;
    vfs_node_stat(node, &stat);
	if ( stat.type != VFS_NODE_DIR ) {
        printk("Error: %s is not a directory\n", pathname);
		vfs_node_refdrop(node);
		return -1;
	}

	vfs_set_cwd(node);
	vfs_node_refdrop(node);

    return 0;
}

static void post_print(struct vfs_node_s *node)
{
	if ( node->parent != node ) {
		post_print(node->parent);
		printk("/");
	}
	printk("%s", node->name);
}

int pwd(lua_State *st)
{
	post_print(vfs_get_cwd());
    printk("\n");

    return 0;
}

int rm(lua_State *st)
{
    if ( lua_gettop(st) < 1 )
		return 1;

	const char *filename = lua_getstringopt(st, 1, NULL);

	error_t err = vfs_unlink(vfs_get_root(), vfs_get_cwd(), filename);
	if ( err ) {
		printk("Error: %s\n", strerror(err));
	}
	return 0;
}

int ln(lua_State *st)
{
    if ( lua_gettop(st) < 2 )
		return 1;

	const char *src = lua_getstringopt(st, 1, NULL);
	const char *dst = lua_getstringopt(st, 2, NULL);

	error_t err = vfs_link(vfs_get_root(), vfs_get_cwd(),
                           src, dst);
	if ( err )
		printk("Error: %s\n", strerror(err));

	return err;
}

int mv(lua_State *st)
{
    if ( lua_gettop(st) < 2 )
		return 1;

	const char *src = lua_getstringopt(st, 1, NULL);
	const char *dst = lua_getstringopt(st, 2, NULL);

	error_t err = vfs_link(vfs_get_root(), vfs_get_cwd(),
                           src, dst);
	if ( err ) {
		printk("link error %s\n", strerror(err));
        return err;
    }

    err = vfs_unlink(vfs_get_root(), vfs_get_cwd(),
                             src);
	if ( err )
		printk("unlink error %s\n", strerror(err));

	return err;
}

int mount(lua_State *st)
{
    if ( lua_gettop(st) < 1 )
		return 1;

	const char *filename = lua_getstringopt(st, 1, NULL);

	struct vfs_node_s *node;
	error_t err = vfs_lookup(vfs_get_root(), vfs_get_cwd(), filename, &node);
	if ( err )
		goto err_node;

	struct vfs_fs_s *mount;
	err = ramfs_open(&mount);
	if ( err )
		goto err_open;

	err = vfs_mount(node, mount);
	if ( err )
		goto err_mount;

	return 0;
  err_mount:
//	ramfs_close(mount);
  err_open:
	mem_free(mount);
	vfs_node_refdrop(node);
  err_node:
	printk("Error: %s\n", strerror(err));
	return err;
}

int umount(lua_State *st)
{
    if ( lua_gettop(st) < 1 )
		return 1;

	const char *filename = lua_getstringopt(st, 1, NULL);

	struct vfs_node_s *node;
	error_t err = vfs_lookup(vfs_get_root(), vfs_get_cwd(), filename, &node);
	if ( err )
		goto err_node;

	err = vfs_umount(node);
	if ( err )
		goto err_node;
	return 0;

  err_node:
	printk("Error: %s\n", strerror(err));
	return err;
}

int _mkdir(lua_State *st)
{
    if ( lua_gettop(st) < 1 )
		return 1;

	const char *filename = lua_getstringopt(st, 1, NULL);

	struct vfs_node_s *node;
	error_t err = vfs_create(vfs_get_root(), vfs_get_cwd(), filename, VFS_NODE_DIR, &node);
	if ( !err )
		vfs_node_refdrop(node);
	return 0;
}

int append(lua_State *st)
{
    if ( lua_gettop(st) < 2 )
		return 1;

	struct vfs_file_s *file;
	const char *filename = lua_getstringopt(st, 1, NULL);
	size_t blob_size;
	const char *blob = lua_getstringopt(st, 2, &blob_size);

	error_t err = vfs_open(vfs_get_root(), vfs_get_cwd(),
						   filename,
						   VFS_OPEN_WRITE|VFS_OPEN_CREATE,
						   &file);
	if ( err ) {
		printk("Error opening %s: %d\n", filename, err);
	} else {
		vfs_file_write(file, blob, blob_size);
		vfs_file_close(file);
	}

    return 0;
}

void vfs_dump(struct vfs_node_s *);

int _vfs_dump(lua_State *st)
{
	struct vfs_node_s *root = vfs_get_root();

	vfs_dump(root);

	return 0;
}

void ramfs_dump(struct vfs_fs_s *);

int _ramfs_dump(lua_State *st)
{
	struct vfs_node_s *root = vfs_get_root();

	ramfs_dump(root->fs);

	return 0;
}

struct vfs_node_s *root_mount;

void init_shell(lua_State* luast)
{
	root_mount = vfs_init();

	vfs_set_root(root_mount);
	vfs_set_cwd(root_mount);

#ifdef CONFIG_DRIVER_FS_ISO9660
    {
        struct vfs_fs_s *cdrom_mount;
        struct device_s *bd;

# ifdef CONFIG_ARCH_SOCLIB
        extern struct device_s fdt_enum_dev;

        if ((bd = enum_fdt_lookup(&fdt_enum_dev, "/block@0"))) {
# elif defined (CONFIG_ARCH_EMU)
        extern struct device_s block_dev;
        if ((bd = &block_dev)) {
# endif
            error_t err = iso9660_open(&cdrom_mount, bd);
            if ( err ) {
                printk("Error opening ISO fs: %s\n", strerror(err));
                abort();
            }

            struct vfs_node_s *node;
            err = vfs_create(root_mount, root_mount, "cdrom", VFS_NODE_DIR, &node);
            if ( err ) {
                printk("Error creating \"/cdrom\": %s\n", strerror(err));
                abort();
            }

            err = vfs_mount(node, cdrom_mount);
            if ( err ) {
                printk("Error mounting \"/cdrom\": %s\n", strerror(err));
                abort();
            }
            vfs_node_refdrop(node);
        }
    }
#endif

    printk("ok\n");

    lua_register(luast, "mount", mount);
    lua_register(luast, "umount", umount);
    lua_register(luast, "ls", ls);
    lua_register(luast, "ln", ln);
    lua_register(luast, "mv", mv);
    lua_register(luast, "cat", cat);
    lua_register(luast, "cd", cd);
    lua_register(luast, "mkdir", _mkdir);
    lua_register(luast, "rm", rm);
    lua_register(luast, "pwd", pwd);
    lua_register(luast, "append", append);
    lua_register(luast, "vfs_dump", _vfs_dump);
    lua_register(luast, "ramfs_dump", _ramfs_dump);

    struct vfs_node_s *root = vfs_get_root();

    //	vfs_dump(root);

	vfs_dump(root);
	ramfs_dump(root->fs);


	struct vfs_file_s *file;
	error_t err = vfs_open(root, root, "/test.txt", VFS_OPEN_WRITE|VFS_OPEN_CREATE, &file);
	if ( err ) {
		printk("Error opening /test.txt: %d\n", err);
	} else {
		vfs_file_write(file, "test contents line 1\n", 21);
		vfs_file_write(file, "contents line test 2\n", 21);
		vfs_file_close(file);
	}

	vfs_dump(root);
	ramfs_dump(root->fs);

	// vfs_dump(root);
}

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4


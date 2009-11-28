
#include <stdio.h>
#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <lua/lua.h>
#include <vfs/vfs.h>

#include <drivers/fs/ramfs/ramfs.h>

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

	error_t err = vfs_open(vfs_get_root(), vfs_get_cwd(), pathname, 0, &dir);
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
                        printk("No such file %s\n", pathname);
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

	if ( node->type != VFS_NODE_DIR ) {
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
	if ( node->parent ) {
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

int mount(lua_State *st)
{
    if ( lua_gettop(st) < 1 )
		return 1;

	const char *filename = lua_getstringopt(st, 1, NULL);

	struct vfs_node_s *node;
	error_t err = vfs_lookup(vfs_get_root(), vfs_get_cwd(), filename, &node);
	if ( err )
		goto err_node;

	struct vfs_mount_s *mount;
	err = ramfs_open(NULL, 0, &mount);
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

	err = vfs_umount(node->mount);
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
		printk("Error opening %s: %d\n", err);
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

void init_shell(lua_State* luast)
{
    lua_register(luast, "mount", mount);
    lua_register(luast, "umount", umount);
    lua_register(luast, "ls", ls);
    lua_register(luast, "cat", cat);
    lua_register(luast, "cd", cd);
    lua_register(luast, "mkdir", _mkdir);
    lua_register(luast, "rm", rm);
    lua_register(luast, "pwd", pwd);
    lua_register(luast, "append", append);
    lua_register(luast, "vfs_dump", _vfs_dump);

	struct vfs_node_s *root = vfs_get_root();

	vfs_dump(root);

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
}

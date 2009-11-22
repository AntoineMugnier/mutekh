
#include <stdio.h>
#include <lua/lua.h>
#include <vfs/vfs.h>

extern struct vfs_node_s *vfs_root_term;

static void compute_name(struct vfs_node_s *node, char *buf)
{
    if (!node->n_parent)
        strcat(buf, node->n_name);
    else
    {
        compute_name(node->n_parent, buf);
        strcat(buf, node->n_name);
        strcat(buf, "/");
    }
}

static inline char * lua_getstringopt(lua_State *st)
{
    const char *s = lua_tolstring(st, 1, 0);
    if (!s) 
        lua_error(st);
    return s;
}

int ls(lua_State *st)
{
    if (lua_gettop(st) <= 0)
        lua_pushstring(st, ".");

    const char *pathname = lua_getstringopt(st);

    struct vfs_file_s *dir;
    struct vfs_dirent_s dirent;

    if (vfs_opendir(vfs_root_term, pathname, 0, &dir))
    {
        printk("no such file or directory\n");
        return -1;
    }

    while (!vfs_readdir(dir, &dirent))
        printk("%s  ", dirent.d_name);

    printk("\n");

    vfs_closedir(dir);

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
                    char *filename = lua_tostring(st, i);
                    char *pathname[128];
                    char buffer[10];

                    memset(pathname, 0, 128);
                    if (filename[0] != '/') /* absolute path */
                        compute_name(vfs_root_term, pathname);
                    strncat(pathname, filename, 128-1-strlen(pathname));

                    if ((f = fopen(pathname, "r")) == NULL)
                    {
                        printk("No such file %s\n", pathname);
                        break;
                    }
                    while (fgets(buffer, sizeof(buffer), f))
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

    const char *pathname = lua_getstringopt(st);

    int err = vfs_chdir(pathname, vfs_root_term, &vfs_root_term);
    if(err)
    {
        printk("no such file or directory\n");
        return -1;
    }

    return 0;
}

int pwd(lua_State *st)
{
    char *pathname[128];

    memset(pathname, 0, 128);
    compute_name(vfs_root_term, pathname);

    printk("%s\n", pathname);

    return 0;
}

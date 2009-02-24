#include <drivers/device/icu/soclib/icu-soclib.h>

#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>

#include <hexo/device.h>
#include <device/driver.h>
#include <hexo/interrupt.h>

#include <drivers/device/icu/soclib/icu-soclib.h>
#include <drivers/device/block/soclib/block-soclib.h>

#include <vfs/vfs.h>
#include <lua.h>

#include <termui/term.h>
#include <termui/getline.h>

extern struct device_s icu_dev;

struct device_s bd_dev;

extern struct device_s *tty_dev;

struct vfs_node_s *vfs_root_term;

extern struct stream_ops_s vfs_ops;

pthread_t a;

/* shell functions */
int ls(lua_State*);
int cat(lua_State*);
int cd(lua_State*);
int pwd(lua_State*);

/* line completion handler found in getline_lua_complete.c */
GETLINE_FCN_COMPLETE(lua_complete);

static GETLINE_FCN_PROMPT(prompt)
{
    return term_printf(tm, "[lua:%s] ", vfs_root_term->n_name);
}

static void *l_alloc (void *ud, void *ptr, size_t osize, size_t nsize)
{
    void	*p;

    (void)ud;
    (void)osize;
    if (nsize == 0) {
        free(ptr);
        p = NULL;
    }
    else
        p = realloc(ptr, nsize);

    return p;
}

typedef struct LoadS {
    const char *s;
    size_t size;
} LoadS;

static const char *getS (lua_State *L, void *ud, size_t *size) {
    LoadS *ls = (LoadS *)ud;
    (void)L;
    if (ls->size == 0) return NULL;
    *size = ls->size;
    ls->size = 0;
    return ls->s;
}

int luaL_loadbuffer (lua_State *L, const char *buff, size_t size,
        const char *name) {
    LoadS ls;
    ls.s = buff;
    ls.size = size;
    return lua_load(L, getS, &ls, name);
}

void* shell(void *param)
{
    struct term_s			*tm;
    struct term_behavior_s	*bhv;
    lua_State			*luast;

    printk("init vfs... ");
    vfs_init(&bd_dev, VFS_VFAT_TYPE, 20, 20, &vfs_root_term);
    stream_fops = &vfs_ops;
    printk("ok\n");

    /* create lua state */
    luast = lua_newstate(l_alloc, NULL);

    lua_register(luast, "ls", ls);
    lua_register(luast, "cat", cat);
    lua_register(luast, "cd", cd);
    lua_register(luast, "pwd", pwd);

    /* initialize terminal */
    if (!(tm = term_alloc(tty_dev, tty_dev, luast)))
        return -1;

    /* set capabilities */
    term_set(tm, "xterm");

#if defined(CONFIG_DRIVER_CHAR_SOCLIBTTY)
    char *disable_cr = "\x1b[20l";
    char *enable_cr = "\x1b[20h";
    term_writestr(tm, disable_cr, strlen(disable_cr));
#endif

    term_printf(tm, "lua shell example, use Ctrl-D to quit\n\n");

    /* initialize getline behavior according to term capabilities */
    if (!(bhv = getline_alloc(tm, 256)))	/* max line len = 256 */
        return -1;

    getline_history_init(bhv, 64); /* 64 entries max */
    getline_complete_init(bhv, lua_complete);
    getline_setprompt(bhv, prompt);

    while (1)
    {
        int oldtop;
        const char *line;

        if (!(line = getline_process(bhv)))
            break;

        /* skip blank line */
        if (!*(line += strspn(line, "\n\r\t ")))
            continue;

        getline_history_addlast(bhv);

        if (luaL_loadbuffer(luast, line, strlen(line), ""))
        {
            term_printf(tm, "%91AParse error:%A %s\n", lua_tostring(luast, -1));
            lua_pop(luast, 1);
            continue;
        }

        oldtop = lua_gettop(luast);

#if defined(CONFIG_DRIVER_CHAR_SOCLIBTTY)
        term_writestr(tm, enable_cr, strlen(enable_cr));
#endif
        int err = lua_pcall(luast, 0, LUA_MULTRET, 0);
#if defined(CONFIG_DRIVER_CHAR_SOCLIBTTY)
        term_writestr(tm, disable_cr, strlen(disable_cr));
#endif

        if (err)
        {
            term_printf(tm, "%91AExecution error:%A %s\n", lua_tostring(luast, -1));
            lua_pop(luast, 1);
            continue;
        }

        lua_pop(luast, lua_gettop(luast) - oldtop + 1);

        //term_printf(tm, "\n");
    }

    lua_close(luast);

    /* free resources allocated by getline behavior */
    getline_free(bhv);

    /* free resources and restore terminal attributes */
    term_free(tm);

    return 0;
}

int main()
{
    device_init(&bd_dev);
    bd_dev.addr[0] = 0x65200000;
    bd_dev.irq = 2;
    block_soclib_init(&bd_dev, &icu_dev, NULL);
    DEV_ICU_BIND(&icu_dev, &bd_dev);

    pthread_create(&a, NULL, shell, NULL);
}


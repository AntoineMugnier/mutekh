#if defined (CONFIG_ARCH_SOCLIB)
#include <drivers/device/icu/soclib/icu-soclib.h>
#endif
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>

#include <hexo/device.h>
#include <device/driver.h>
#if defined (CONFIG_ARCH_SOCLIB)
#include <hexo/interrupt.h>

#include <drivers/device/icu/soclib/icu-soclib.h>
#include <drivers/device/block/soclib/block-soclib.h>
#elif defined (CONFIG_ARCH_EMU)
#include <drivers/device/block/file-emu/block-file-emu.h>
#else
#error "Your arch is not supported yet"
#endif

#include <vfs/vfs.h>
#include <lua/lua.h>
#include <lua/lauxlib.h>
#include <lua/lualib.h>

#include <termui/term.h>
#include <termui/getline.h>

#include <rtld/rtld.h>
#include <dsrl/dsrl.h>

#if defined (CONFIG_ARCH_SOCLIB)
extern struct device_s icu_dev;
#endif
extern struct device_s bd_dev;
extern struct device_s *tty_dev;

struct vfs_node_s *vfs_root_term;

pthread_t a;

/* line completion handler found in getline_lua_complete.c */
GETLINE_FCN_COMPLETE(lua_complete);

static GETLINE_FCN_PROMPT(prompt)
{
    uint32_t nb_kcycles = cpu_cycle_count()/1000;
    return term_printf(tm, "[lua:%s:%ukc] ", vfs_root_term->n_name, nb_kcycles);
}

void* shell(void *param)
{
    struct term_s			*tm;
    struct term_behavior_s	*bhv;
    lua_State			*luast;

    printk("init vfs... ");
    if(vfs_init(&bd_dev, VFS_VFAT_TYPE, 20, 20, &vfs_root_term) != 0)
    {
        printk("not ok !\n");
        abort();
    }
    printk("ok\n");

    printk("init rtld...");
    rtld_user_init();
    printk("ok\n");

    /* create lua state */
    luast = lua_open();
    luaL_openlibs(luast);
    luaopen_dsrl(luast);

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
    pthread_create(&a, NULL, shell, NULL);
}


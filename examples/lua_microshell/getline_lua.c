#include <stdio.h>
#include <mutek/printk.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>

#include <drivers/device/icu/soclib-icu/icu-soclib.h>
#include <drivers/device/block/soclib/block-soclib.h>
#include <drivers/device/block/partition/block-partition.h>

#include <device/device.h>
#include <device/driver.h>
#include <hexo/interrupt.h>

#include <vfs/vfs.h>
#include <lua/lauxlib.h>
#include <lua/lua.h>

#include <drivers/fs/ramfs/ramfs.h>

#include <termui/term.h>
#include <termui/getline.h>

extern struct device_s * console_dev;

struct device_s * rootfs_dev;

pthread_t a;


void init_shell(lua_State*);
void init_dsrl_shell(lua_State* luast);
void init_rtld_shell(lua_State* luast);

/* line completion handler found in getline_lua_complete.c */
GETLINE_FCN_COMPLETE(lua_complete);

static GETLINE_FCN_PROMPT(prompt)
{
    char name[CONFIG_VFS_NAMELEN];
    vfs_node_get_name(vfs_get_cwd(), name, CONFIG_VFS_NAMELEN);
    return term_printf(tm, "[lua:%s] ", name);
}

void* shell(void *param)
{
    struct term_s			*tm;
    struct term_behavior_s	*bhv;
    lua_State			*luast;

    /* create lua state */
    luast = luaL_newstate();

    luaL_openlibs(luast);
    init_shell(luast);

#if defined(CONFIG_LIBDSRL)
    init_dsrl_shell(luast);
#endif

#if defined(CONFIG_LIBELF_RTLD)
    init_rtld_shell(luast);
#endif

    /* initialize terminal */
    if (!(tm = term_alloc(console_dev, console_dev, luast)))
        return NULL;

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
        return NULL;

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
    }

    lua_close(luast);

    /* free resources allocated by getline behavior */
    getline_free(bhv);

    /* free resources and restore terminal attributes */
    term_free(tm);

    return 0;
}


void do_block_hexdump(struct device_s *bd, size_t lba)
{
	const struct dev_block_params_s *params = dev_block_getparams(bd);
	
	uint8_t block[params->blk_size];
	uint8_t *blocks[1] = {block};

	error_t err = dev_block_wait_read(bd, blocks, lba, 1);
	if ( err ) {
		printf("Error reading LBA %x, %d\r\n", lba, err);
	} else {
		size_t i;
		printf("LBA %x, read ok\r\n", lba);
		for ( i=0; i<params->blk_size; i+=16 )
			printf(" %p: %P\r\n", (void*)(uintptr_t)i, &block[i], 16);
	}
}


void app_start()
{
/* 	if ( block_partition_create(&bd_dev, 0) > 0 ) { */
/* 		part_dev = device_get_child(&bd_dev, 0); */
/* 	} else { */
/* 		part_dev = &bd_dev; */
/* 	} */

/* 	do_block_hexdump(part_dev, 0); */

    pthread_create(&a, NULL, shell, NULL);
}


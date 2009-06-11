#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <dsrl/dsrl-private.h>

#define CREATE_DSRL_RESOURCE(resource) static resource##_t create_##resource (lua_State *L)
#define USTRING_DSRL_RESOURCE(resource) static void ustring_##resource (lua_State *L, resource##_t c, luaL_Buffer *b)

/*
 * Barrier 
 */
CREATE_DSRL_RESOURCE(dsrl_barrier)
{
    _dsrl_debug("create_dsrl_barrier\n");
    size_t n = luaL_checkinteger(L, 3);
    dsrl_barrier_t barrier = (dsrl_barrier_t)malloc(sizeof(dsrl_barrier_s));
    barrier->count = n;
    barrier->max = n;
    barrier->serial = 0;
    barrier->lock = 0;
    _dsrl_debug("\tmax: %d\n", n);
    _dsrl_debug("\tbarrier is %p\n", barrier);
    return barrier;
}
USTRING_DSRL_RESOURCE(dsrl_barrier)
{
    lua_pushfstring(L, "\tcount=%d\n", c->count);
    luaL_addvalue(b);
    lua_pushfstring(L, "\tmax=%d", c->max);
    luaL_addvalue(b);
}
DSRL_RESOURCE(dsrl_barrier)
DSRL_RESOURCE_TABLE(dsrl_barrier)

/* 
 * Memspace 
 */
CREATE_DSRL_RESOURCE(dsrl_memspace)
{
    _dsrl_debug("create_dsrl_memspace\n");
    size_t size = luaL_checkinteger(L, 3);
    dsrl_memspace_t mem = (dsrl_memspace_t)malloc(sizeof(dsrl_memspace_s) + size);
    mem->buffer = (uint8_t*)mem + sizeof(dsrl_memspace_s);
    mem->size = size;
    _dsrl_debug("\tsize: %d bytes\n", size);
    _dsrl_debug("\tmemspace is %p\n", mem);
    return mem;
}
USTRING_DSRL_RESOURCE(dsrl_memspace)
{
    lua_pushfstring(L, "\tbuf=%p\n", c->buffer);
    luaL_addvalue(b);
    lua_pushfstring(L, "\tsize=%d bytes", c->size);
    luaL_addvalue(b);
}
DSRL_RESOURCE(dsrl_memspace)
DSRL_RESOURCE_TABLE(dsrl_memspace)

/*
 * IO memspace 
 */
CREATE_DSRL_RESOURCE(dsrl_io_memspace)
{
    _dsrl_debug("create_dsrl_io_memspace\n");
    size_t size = luaL_checkinteger(L, 3);
    void *buf = (void*)luaL_checkinteger(L, 4);
    dsrl_memspace_t mem = (dsrl_memspace_t)malloc(sizeof(dsrl_memspace_s));
    mem->buffer = buf;
    mem->size = size;
    _dsrl_debug("\tsize: %d bytes\n", size);
    _dsrl_debug("\tio_memspace is %p\n", mem);
    return mem;
}
USTRING_DSRL_RESOURCE(dsrl_io_memspace)
{
    lua_pushfstring(L, "\taddr=%p\n", c->buffer);
    luaL_addvalue(b);
    lua_pushfstring(L, "\tsize=%d bytes", c->size);
    luaL_addvalue(b);
}
DSRL_RESOURCE(dsrl_io_memspace)
DSRL_RESOURCE_TABLE(dsrl_io_memspace)

/*
 * File 
 *  (this is not really part of original srl api...) 
 */
CREATE_DSRL_RESOURCE(dsrl_file)
{
    _dsrl_debug("create_dsrl_file\n");
    const char *filename = luaL_checkstring(L, 3);
    struct stat st;
    if (stat(filename, &st) != 0)
    {
        _dsrl_debug("\tstat failed on %s\n", filename);
        return NULL;
    }
    _dsrl_debug("\tcreate a new space of %d bytes\n", st.st_size);
    dsrl_file_t mem = (dsrl_file_t)malloc(sizeof(dsrl_file_s) + st.st_size);
    mem->buffer = (uint8_t*)mem + sizeof(dsrl_file_s);
    mem->size = st.st_size;
    _dsrl_debug("\tfill the space with %s\n", filename);
    FILE *f = fopen(filename, "r");
    fread(mem->buffer, st.st_size, 1, f);
    fclose(f);
    _dsrl_debug("\tsize: %d bytes\n", st.st_size);
    _dsrl_debug("\tmem is %p\n", mem);
    return mem;
}
USTRING_DSRL_RESOURCE(dsrl_file)
{
    lua_pushfstring(L, "\tbuf=%p\n", c->buffer);
    luaL_addvalue(b);
    lua_pushfstring(L, "\tsize=%d bytes", c->size);
    luaL_addvalue(b);
}
DSRL_RESOURCE(dsrl_file)
DSRL_RESOURCE_TABLE(dsrl_file)

/* 
 * Mwmr 
 */
CREATE_DSRL_RESOURCE(dsrl_mwmr)
{
    _dsrl_debug("create_dsrl_mwmr\n");
    size_t width = luaL_checkinteger(L, 3);
    size_t depth = luaL_checkinteger(L, 4);
    size_t size = width*depth;
    dsrl_mwmr_t mwmr = (dsrl_mwmr_t)malloc(sizeof(dsrl_mwmr_s) + size);
    mwmr->width = width;
    mwmr->depth = depth;
    mwmr->gdepth = size;
    mwmr->status.free_tail = 0;
    mwmr->status.free_head = 0;
    mwmr->status.free_size = size;
    mwmr->status.data_tail = 0;
    mwmr->status.data_head = 0;
    mwmr->status.data_size = 0;
    mwmr->buffer = (uint8_t*)mwmr + sizeof(dsrl_mwmr_s);
    _dsrl_debug("\tmwmr is %p\n", mwmr);
    return mwmr;
}
USTRING_DSRL_RESOURCE(dsrl_mwmr)
{
    lua_pushfstring(L, "\twidth=%d bytes\n", c->width);
    luaL_addvalue(b);
    lua_pushfstring(L, "\tdepth=%d bytes", c->depth);
    luaL_addvalue(b);
}
DSRL_RESOURCE(dsrl_mwmr)
DSRL_RESOURCE_TABLE(dsrl_mwmr)

/*
 * Const 
 */
CREATE_DSRL_RESOURCE(dsrl_const)
{
    _dsrl_debug("create_dsrl_const\n");
    uintptr_t val = luaL_checkinteger(L, 3);
    dsrl_const_t c = (dsrl_const_t)malloc(sizeof(dsrl_const_s));
    c->val = val;
    _dsrl_debug("\tval: %d\n", val);
    return c;
}
USTRING_DSRL_RESOURCE(dsrl_const)
{
    lua_pushfstring(L, "\tval=%d", c->val);
    luaL_addvalue(b);
}
DSRL_RESOURCE(dsrl_const)
DSRL_RESOURCE_TABLE(dsrl_const)


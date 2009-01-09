
#include <lua.h>

lua_State	*ls;

int cmd_print(lua_State *st)
{
  unsigned int	i;

  for (i = 1; i <= lua_gettop(st); i++)
    {
      switch (lua_type(st, i))
	{
	case LUA_TNUMBER:
	  printf("(lua num %i)\n", lua_tonumber(st, i));
	  break;
	case LUA_TSTRING:
	  printf("(lua str %s)\n", lua_tostring(st, i));
	  break;
	default:
	  printf("(lua type %i)\n", lua_type(st, i));
	}
    }

  return 0;
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

int main()
{
  const char	*cmd = "for i = 1, 6, 1 do print (\"coucou\", i); end";

  ls = lua_newstate(l_alloc, NULL);

  lua_pushstring(ls, "print");
  lua_pushcfunction(ls, cmd_print);
  lua_settable(ls, LUA_GLOBALSINDEX);

  luaL_loadbuffer(ls, cmd, strlen(cmd), cmd);
  lua_pcall(ls, 0, LUA_MULTRET, 0);
}


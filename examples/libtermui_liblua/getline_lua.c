/*
    This file is part of libtermui.

    libtermui is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    libtermui is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with libtermui.  If not, see <http://www.gnu.org/licenses/>.

    Copyright 2006, Alexandre Becoulet <alexandre.becoulet@free.fr>

*/

/*
 * This example show how to use libtermui standalone getline feature
 * with lua script interpreter.
 */

#include <lua.h>

#include <stdlib.h>
#include <string.h>

#include <termui/term.h>
#include <termui/getline.h>

/* line completion handler found in getline_lua_complete.c */
GETLINE_FCN_COMPLETE(lua_complete);

static GETLINE_FCN_PROMPT(prompt)
{
  return term_printf(tm, "[%31Alua%A] ");
}

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

extern struct device_s *tty_dev;

int main()
{
  struct term_s			*tm;
  struct term_behavior_s	*bhv;
  lua_State			*luast;

  /* create lua state */
  luast = lua_newstate(l_alloc, NULL);
  //  luaL_openlibs(luast);

  lua_pushstring(luast, "print");
  lua_pushcfunction(luast, cmd_print);
  lua_settable(luast, LUA_GLOBALSINDEX);

  /* initialize terminal */
  if (!(tm = term_alloc(tty_dev, tty_dev, luast)))
    return -1;

  /* set capabilities */
  term_set(tm, "xterm");

  term_printf(tm, "libtermui lua getline example, use Ctrl-D to quit\n\n");

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

      if (lua_pcall(luast, 0, LUA_MULTRET, 0))
	{
	  term_printf(tm, "%91AExecution error:%A %s\n", lua_tostring(luast, -1));
	  lua_pop(luast, 1);
	  continue;
	}

      lua_pop(luast, lua_gettop(luast) - oldtop + 1);

      term_printf(tm, "\n");
    }

  lua_close(luast);

  /* free resources allocated by getline behavior */
  getline_free(bhv);

  /* free resources and restore terminal attributes */
  term_free(tm);

  return 0;
}


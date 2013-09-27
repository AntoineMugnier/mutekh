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

#include <lua/lauxlib.h>
#include <lua/lualib.h>
#include <lua/lua.h>

#include <stdlib.h>
#include <string.h>

#include <termui/term.h>
#include <termui/getline.h>
#include <termui/mutekh.h>

#include <mutek/console.h>

/* line completion handler found in getline_lua_complete.c */
TERMUI_GETLINE_FCN_COMPLETE(lua_complete);

static TERMUI_GETLINE_FCN_PROMPT(prompt)
{
  return termui_term_printf(tm, "[%31Alua%A] ");
}

void main()
{
  struct termui_term_s tm;
  struct termui_term_behavior_s	bhv;
  lua_State			*luast;

  assert(device_check_accessor(&console_dev));

  /* create lua state */
  luast = luaL_newstate();
  luaL_openlibs(luast);

  termui_dev_io_init(&tm, &console_dev, "xterm");
  termui_getline_init(&tm, &bhv, 256);
  termui_term_set_private(&tm, luast);

  termui_term_printf(&tm, "libtermui lua getline example, use Ctrl-D to quit\n\n");

  termui_getline_history_init(&bhv, 64); /* 64 entries max */
  termui_getline_complete_init(&bhv, lua_complete);
  termui_getline_setprompt(&bhv, prompt);

  while (1)
    {
      int oldtop;
      const char *line;

      if (!(line = termui_getline_process(&bhv)))
	break;

      /* skip blank line */
      if (!*(line += strspn(line, "\n\r\t ")))
	continue;

      termui_getline_history_addlast(&bhv);

      if (luaL_loadbuffer(luast, line, strlen(line), ""))
	{
	  termui_term_printf(&tm, "%91AParse error:%A %s\n", lua_tostring(luast, -1));
	  lua_pop(luast, 1);
	  continue;
	}

      oldtop = lua_gettop(luast);

      if (lua_pcall(luast, 0, LUA_MULTRET, 0))
	{
	  termui_term_printf(&tm, "%91AExecution error:%A %s\n", lua_tostring(luast, -1));
	  lua_pop(luast, 1);
	  continue;
	}

      lua_pop(luast, lua_gettop(luast) - oldtop + 1);

      termui_term_printf(&tm, "\n");
    }

  lua_close(luast);

  /* free resources allocated by getline behavior */
  termui_getline_cleanup(&bhv);

  /* free resources and restore terminal attributes */
  termui_termio_cleanup(&tm);
}


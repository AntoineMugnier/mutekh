
#include <lua/lauxlib.h>
#include <lua/lua.h>

#include <pthread.h>

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

void term_lcd_init(lua_State *st);
void term_block_init(lua_State *st);

extern struct device_s *console_dev;

void* lua_main(void *unused)
{
  struct term_s			*tm;
  struct term_behavior_s	*bhv;
  lua_State			*luast;

  /* create lua state */
  luast = luaL_newstate();
  luaL_openlibs(luast);

  lua_register(luast, "print", cmd_print);

  term_lcd_init(luast);
  term_i2c_init(luast);
  term_block_init(luast);


  /* initialize terminal */
  if (!(tm = term_alloc(console_dev, console_dev, luast)))
    return -1;

  /* set capabilities */
  term_set(tm, "vt102");

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

void *joystick_main(void *unused);

void app_start()
{
	cpu_interrupt_enable();

	static pthread_t a;
	pthread_create(&a, NULL, lua_main, NULL);

	static pthread_t b;
	pthread_create(&b, NULL, joystick_main, NULL);
}

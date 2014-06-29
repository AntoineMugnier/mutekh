
#include <stdlib.h>

#include <termui/term.h>
#include <termui/getline.h>
#include <termui/mutekh.h>
#include <termui/console.h>

#include <mutek/console.h>

static TERMUI_CON_COMMAND_PROTOTYPE(foo_command)
{
  int i;

  for (i = 0; i < argc; i++)
    termui_con_printf(con, "Argument %i : '%s'\n", i, argv[i]);

  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(bar_command)
{
  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(test_command)
{
  return 0;
}

/* console commands descriptors array */
static TERMUI_CON_GROUP_DECL(root_group) =
{
  /* Some useful commands provided by the library */
  TERMUI_CON_BUILTIN_HELP(-1),
  TERMUI_CON_BUILTIN_LIST(-1),
  TERMUI_CON_BUILTIN_QUIT(-1),

  /* Bare simple command */
  TERMUI_CON_ENTRY(bar_command, "bar"),

  /* Simple command with short help and allowed argc range */
  TERMUI_CON_ENTRY(foo_command, "foo",
		   /* Help entry */
		   TERMUI_CON_HELP("The foo command", NULL)
		   /* Valid argc range is [0, 10] */
		   TERMUI_CON_ARGS(0, 10)
		   ),

  /* Simple command with short and long help message */
  TERMUI_CON_ENTRY(test_command, "test",
		   TERMUI_CON_HELP("The test command",
				   "This is the long help text")
		   ),

  TERMUI_CON_LIST_END
};

void main()
{
  struct termui_term_s tm;
  struct termui_console_s con;

  assert(device_check_accessor(&console_dev));

  termui_dev_io_init(&tm, &console_dev, "xterm");
  termui_con_init(&con, &tm, root_group);

  termui_con_set_prompt(&con, "[%31Aconsole%A] ");

  termui_term_printf(&tm, "libtermui console ui example. You may type `list' and `help'.\n\n");

  /* process user commands */
  error_t res;
  do {
    res = termui_con_process(&con);
  } while (res != -EIO && res != -ECANCELED);

  /* free allocated resources */
  termui_con_cleanup(&con);
  termui_termio_cleanup(&tm);
}


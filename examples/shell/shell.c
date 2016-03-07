/*
  This example shows how to run a custom libtermui shell on a char
  device.

  More examples of declarations related to command line and options
  parsing are provided with the libtermui source code.

  In order to simply run the regular mutek shell thread in your
  existing application, simply define the CONFIG_MUTEK_CONSOLE and
  CONFIG_MUTEK_SHELL_THREAD tokens. You can also register your own
  commands in the regular shell by using the MUTEK_SHELL_ROOT_GROUP
  macro.
*/

#include <assert.h>

#include <mutek/console.h>
#include <mutek/shell.h>
#include <mutek/startup.h>
#include <mutek/thread.h>

#include <device/driver.h>
#include <device/device.h>
#include <device/class/char.h>

static TERMUI_CON_COMMAND_PROTOTYPE(foo_command)
{
  void *my_private_data = termui_con_get_private(con);

  int i;
  for (i = 0; i < argc; i++)
    termui_con_printf(con, "Argument %i : '%s'\n", i, argv[i]);

  return 0;
}

static TERMUI_CON_GROUP_DECL(root_group) =
{
  TERMUI_CON_BUILTIN_HELP(-1)
  TERMUI_CON_BUILTIN_LIST(-1)
  TERMUI_CON_BUILTIN_QUIT(-1)

  /* Simple command with short help and allowed argc range */
  TERMUI_CON_ENTRY(foo_command, "foo",
		   /* Help entry */
		   TERMUI_CON_HELP("The foo command", NULL)
		   /* Valid argc range is [0, 10] */
		   TERMUI_CON_ARGS(0, 10)
		   )

  TERMUI_CON_LIST_END
};

static CONTEXT_ENTRY(shell_thread)
{
  void *my_private_data = NULL;

#if defined(CONFIG_MUTEK_CONSOLE) && !defined(CONFIG_MUTEK_SHELL_THREAD)
  assert(device_check_accessor(&console_dev));

  /* start a shell on the mutekh console char device */
  mutek_shell_start(&console_dev, "xterm", root_group, "$ ", my_private_data);
#else

  struct device_char_s chardev;
  if (device_get_accessor_by_path(&chardev, NULL,
        "console* tty* uart*", DRIVER_CLASS_CHAR))
    abort();

  /* start a shell on a char device */
  mutek_shell_start(&chardev, "xterm", root_group, "$ ", my_private_data);
#endif
}

void app_start()
{
  thread_create(shell_thread, NULL, NULL);
}


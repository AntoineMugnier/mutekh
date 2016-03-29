
#include <stdlib.h>

#include <termui/term.h>
#include <termui/getline.h>
#include <termui/mutekh.h>

#include <mutek/console.h>

static TERMUI_GETLINE_FCN_PROMPT(prompt)
{
  return termui_term_printf(tm, "[%31AMutekH%A] ");
}

struct termui_term_s tm;
struct termui_term_behavior_s	bhv;

void main()
{
  assert(device_check_accessor(&console_dev.base));

  termui_dev_io_init(&tm, &console_dev, "xterm");
  termui_getline_init(&tm, &bhv, 256);
  termui_term_set_private(&tm, NULL);

  termui_term_printf(&tm, "libtermui getline example, use Ctrl-D to quit\n\n");

  termui_getline_history_init(&bhv, 64); /* 64 entries max */
  termui_getline_setprompt(&bhv, prompt);

  while (1)
    {
      const char *line;

      if (!(line = termui_getline_process(&bhv)))
	break;

      /* skip blank line */
      if (!*(line += strspn(line, "\n\r\t ")))
	continue;

      termui_getline_history_addlast(&bhv);

      termui_term_printf(&tm, "entered line is: `%s'\n\n", line);
    }

  /* free resources allocated by getline behavior */
  termui_getline_cleanup(&bhv);

  /* free resources and restore terminal attributes */
  termui_termio_cleanup(&tm);

  while (1)
    ;
}



#include <termui/term.h>
#include <termui/getline.h>

extern struct device_s *console_dev;

static GETLINE_FCN_PROMPT(prompt)
{
  return term_printf(tm, "[%31AMutekH%A] ");
}

void app_start()
{
  struct term_s			*tm;
  struct term_behavior_s	*bhv;

  /* initialize terminal */
  if (!(tm = term_alloc(console_dev, console_dev, NULL)))
    abort();

  /* set capabilities */
  term_set(tm, "xterm");

  term_printf(tm, "libtermui getline example, use Ctrl-D to quit\n\n");

  /* initialize getline behavior according to term capabilities */
  if (!(bhv = getline_alloc(tm, 256)))	/* max line len = 256 */
    abort();

  getline_history_init(bhv, 64); /* 64 entries max */
  getline_setprompt(bhv, prompt);

  while (1)
    {
      const char *line;

      if (!(line = getline_process(bhv)))
	break;

      /* skip blank line */
      if (!*(line += strspn(line, "\n\r\t ")))
	continue;

      getline_history_addlast(bhv);

      term_printf(tm, "entered line is: `%s'\n\n", line);
    }

  /* free resources allocated by getline behavior */
  getline_free(bhv);

  /* free resources and restore terminal attributes */
  term_free(tm);

  while (1)
    ;
}


/* timeout -- run a command with bounded time
   Copyright (C) 2008-2011 Free Software Foundation, Inc.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.  */


/* timeout - Start a command, and kill it if the specified timeout expires

   We try to behave like a shell starting a single (foreground) job,
   and will kill the job if we receive the alarm signal we setup.
   The exit status of the job is returned, or one of these errors:
     EXIT_TIMEDOUT      124      job timed out
     EXIT_CANCELED      125      internal error
     EXIT_CANNOT_INVOKE 126      error executing job
     EXIT_ENOENT        127      couldn't find job to exec

   Caveats:
     If user specifies the KILL (9) signal is to be sent on timeout,
     the monitor is killed and so exits with 128+9 rather than 124.

     If you start a command in the background, which reads from the tty
     and so is immediately sent SIGTTIN to stop, then the timeout
     process will ignore this so it can timeout the command as expected.
     This can be seen with `timeout 10 dd&` for example.
     However if one brings this group to the foreground with the `fg`
     command before the timer expires, the command will remain
     in the sTop state as the shell doesn't send a SIGCONT
     because the timeout process (group leader) is already running.
     To get the command running again one can Ctrl-Z, and do fg again.
     Note one can Ctrl-C the whole job when in this state.
     I think this could be fixed but I'm not sure the extra
     complication is justified for this scenario.

   Written by Pádraig Brady.  

   Modified by Alexandre Becoulet for use in MutekH testsuite.

*/

#include <getopt.h>
#include <stdio.h>
#include <sys/types.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#include <stdlib.h>
#include <limits.h>
#include <errno.h>

#define PROGRAM_NAME "testwrap"

#define AUTHORS proper_name_utf8 ("Padraig Brady", "P\303\241draig Brady")

static int timed_out;
static int term_signal = SIGTERM;  /* same default as kill command.  */
static int monitored_pid;
static int sigs_to_ignore[NSIG];   /* so monitor can ignore sigs it resends.  */
static unsigned long kill_after;

static struct option const long_options[] =
{
  {"sig-after", required_argument, NULL, 'a'},
  {"kill-after", required_argument, NULL, 'k'},
  {"signal", required_argument, NULL, 's'},
  {"ignore-status", no_argument, NULL, 'i'},
  {"invert-status", no_argument, NULL, 'n'},
  {NULL, 0, NULL, 0}
};

/* send sig to group but not ourselves.
 * FIXME: Is there a better way to achieve this?  */
static int
send_sig (int where, int sig)
{
  sigs_to_ignore[sig] = 1;
  return kill (where, sig);
}

static void
cleanup (int sig)
{
  if (sig == SIGALRM)
    {
      timed_out = 1;
      sig = term_signal;
    }
  if (monitored_pid)
    {
      if (sigs_to_ignore[sig])
        {
          sigs_to_ignore[sig] = 0;
          return;
        }
      if (kill_after)
        {
          /* Start a new timeout after which we'll send SIGKILL.  */
          term_signal = SIGKILL;
          alarm (kill_after);
          kill_after = 0; /* Don't let later signals reset kill alarm.  */
        }
      send_sig (0, sig);
      if (sig != SIGKILL && sig != SIGCONT)
        send_sig (0, SIGCONT);
    }
  else /* we're the child or the child is not exec'd yet.  */
    _exit (128 + sig);
}

void
usage (int status)
{
  if (status != 0)
    {
      printf (("\
Usage: %s [OPTION] COMMAND [ARG]...\n"), PROGRAM_NAME);

      fputs (("\
Start COMMAND, and kill it if still running after DURATION.\n\
\n\
Mandatory arguments to long options are mandatory for short options too.\n\
"), stdout);
      fputs (("\
  -i, --ignore-status\n\
                   ignore command error code.\n\
  -n, --invert-status\n\
                   invert command error code.\n\
  -a, --sig-after=DURATION\n\
                   send signal if COMMAND is still running on timeout\n\
  -k, --kill-after=DURATION\n\
                   also send a KILL signal if COMMAND is still running\n\
                   this long after the initial signal was sent.\n\
  -s, --signal=SIGNAL\n\
                   specify the signal to be sent on timeout.\n\
                   SIGNAL may be a name like `HUP' or a number.\n\
                   See `kill -l` for a list of signals\n"), stdout);

      fputs (("\n\
DURATION is an integer with an optional suffix:\n\
`s' for seconds(the default), `m' for minutes, `h' for hours or `d' for days.\n\
"), stdout);

      fputs (("\n\
If the command times out, then exit with status 124.  Otherwise, exit\n\
with the status of COMMAND.  If no signal is specified, send the TERM\n\
signal upon timeout.  The TERM signal kills any process that does not\n\
block or catch that signal.  For other processes, it may be necessary to\n\
use the KILL (9) signal, since this signal cannot be caught.\n"), stdout);
    }
  exit (status);
}

/* Given a long integer value *X, and a suffix character, SUFFIX_CHAR,
   scale *X by the multiplier implied by SUFFIX_CHAR.  SUFFIX_CHAR may
   be the NUL byte or `s' to denote seconds, `m' for minutes, `h' for
   hours, or `d' for days.  If SUFFIX_CHAR is invalid, don't modify *X
   and return false.  If *X would overflow an integer, don't modify *X
   and return false. Otherwise return true.  */

static int
apply_time_suffix (unsigned long *x, char suffix_char)
{
  unsigned int multiplier = 1;

  switch (suffix_char)
    {
    case 0:
    case 's':
      return 1;
    case 'd':
      multiplier *= 24;
    case 'h':
      multiplier *= 60;
    case 'm':
      if (multiplier > UINT_MAX / 60) /* 16 bit overflow */
        return 0;
      multiplier *= 60;
      break;
    default:
      return 0;
    }

  if (*x > UINT_MAX / multiplier)
    return 0;

  *x *= multiplier;

  return 1;
}

static unsigned long
parse_duration (const char* str)
{
  unsigned long duration;
  char *ep;

  duration = strtoul (str, &ep, 10);
  if ( *ep
      /* Invalid interval. Note 0 disables timeout  */
      || (duration > UINT_MAX)
      /* Extra chars after the number and an optional s,m,h,d char.  */
      || (*ep && *(ep + 1))
      /* Check any suffix char and update timeout based on the suffix.  */
      || !apply_time_suffix (&duration, *ep))
    {
      fprintf(stderr, "invalid time interval %s", str);
      usage (-1);
    }

  return duration;
}

static void
install_signal_handlers (int sigterm)
{
  struct sigaction sa;
  sigemptyset (&sa.sa_mask);  /* Allow concurrent calls to handler */
  sa.sa_handler = cleanup;
  sa.sa_flags = SA_RESTART;  /* restart syscalls (like wait() below) */

  sigaction (SIGALRM, &sa, NULL); /* our timeout.  */
  sigaction (SIGINT, &sa, NULL);  /* Ctrl-C at terminal for example.  */
  sigaction (SIGQUIT, &sa, NULL); /* Ctrl-\ at terminal for example.  */
  sigaction (SIGHUP, &sa, NULL);  /* terminal closed for example.  */
  sigaction (SIGTERM, &sa, NULL); /* if we're killed, stop monitored proc.  */
  sigaction (sigterm, &sa, NULL); /* user specified termination signal.  */
}

int
main (int argc, char **argv)
{
  unsigned long timeout = 0;
  int status_and = 1;
  int status_xor = 0;
  int c;

  //  initialize_main (&argc, &argv);
  //  set_program_name (argv[0]);
  //  setlocale (LC_ALL, "");
  //  bindtextdomain (PACKAGE, LOCALEDIR);
  //  textdomain (PACKAGE);

  //  initialize_exit_failure (EXIT_CANCELED);
  //  atexit (close_stdout);

  while ((c = getopt_long (argc, argv, "+k:s:a:ni", long_options, NULL)) != -1)
    {
      switch (c)
        {
        case 'k':
          kill_after = parse_duration (optarg);
          break;

        case 'a':
          timeout = parse_duration (optarg);

        case 'i':
	  status_and = 0;
	  break;

        case 'n':
	  status_xor = 1;
          break;

        case 's':
          term_signal = atoi(optarg);
          if (term_signal == -1)
            usage (-1);
          break;

        default:
          usage (-1);
          break;
        }
    }

  if (argc - optind < 1)
    usage (-1);

  argv += optind;

  /* Ensure we're in our own group so all subprocesses can be killed.
     Note we don't just put the child in a separate group as
     then we would need to worry about foreground and background groups
     and propagating signals between them.  */
  setpgid (0, 0);

  /* Setup handlers before fork() so that we
     handle any signals caused by child, without races.  */
  install_signal_handlers (term_signal);
  signal (SIGTTIN, SIG_IGN);    /* don't sTop if background child needs tty.  */
  signal (SIGTTOU, SIG_IGN);    /* don't sTop if background child needs tty.  */
  signal (SIGCHLD, SIG_DFL);    /* Don't inherit CHLD handling from parent.   */

  monitored_pid = fork ();
  if (monitored_pid == -1)
    {
      fprintf(stderr, "fork system call failed");
      return -1;
    }
  else if (monitored_pid == 0)
    {                           /* child */
      int exit_status;

      /* exec doesn't reset SIG_IGN -> SIG_DFL.  */
      signal (SIGTTIN, SIG_DFL);
      signal (SIGTTOU, SIG_DFL);

      close(0);
      execvp (argv[0], argv);   /* FIXME: should we use "sh -c" ... here?  */

      /* exit like sh, env, nohup, ...  */
      exit_status = -1;
      fprintf(stderr, "failed to run command %s", argv[0]);
      return exit_status;
    }
  else
    {
      int status;

      if (timeout)
        alarm (timeout);

      /* We're just waiting for a single process here, so wait() suffices.
         Note the signal() calls above on GNU/Linux and BSD at least,
         essentially call the lower level sigaction() with the SA_RESTART flag
         set, which ensures the following wait call will only return if the
         child exits, not on this process receiving a signal. Also we're not
         passing WUNTRACED | WCONTINUED to a waitpid() call and so will not get
         indication that the child has stopped or continued.  */
      if (wait (&status) == -1)
        {
          /* shouldn't happen.  */
          fprintf(stderr, "error waiting for command");
          status = -1;
        }
      else
        {
	  status = (!!status ^ status_xor) & status_and;
        }

      if (timed_out)
        return -1;
      else
        return status;
    }
}

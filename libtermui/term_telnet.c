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

#include <termui/bhv.h>
#include "term_pv.h"

#define IAC 255
#define TELOPT_ECHO 1
#define TELOPT_SGA 3
#define TELOPT_LFLOW 33
#define DO 253
#define WONT 252
#define WILL 251

static error_t term_iac_send(term_stream_t out, char cmd, char opt)
{
  const char iac[3] = { IAC, cmd, opt };

  return stream_write(out, iac, 3) != 3;
}

error_t term_telnet_send_setup(struct term_s *tm)
{
  return term_iac_send(tm->out, DO, TELOPT_ECHO)
    || term_iac_send(tm->out, DO, TELOPT_LFLOW)
    || term_iac_send(tm->out, WILL, TELOPT_ECHO)
    || term_iac_send(tm->out, WILL, TELOPT_SGA);
}

static TERM_FCN_KEYEVENT(bhv_key_iac)
{
  char iac[3] = { key };  

  if (stream_read(bhv->tm->in, iac + 1, 2) != 2)
    return TERM_RET_IOERROR;

  /* process IAC here */

  return TERM_RET_CONTINUE;
}

/* add telnet protocol handling to current behavior */

error_t term_telnet_bhv_init(struct term_behavior_s *bhv)
{
  bhv->keyevent[IAC] = bhv_key_iac;

  return 0;
}


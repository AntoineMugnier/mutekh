/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2016
*/

#ifndef SYSTEMVIEW_EVENT_H_
#define SYSTEMVIEW_EVENT_H_

#include <hexo/decls.h>

enum systemview_event_id_e
{
  // Without length
  SYSTEMVIEW_EVENT_NOP              = 0,
  SYSTEMVIEW_EVENT_OVERFLOW         = 1,
  SYSTEMVIEW_EVENT_ISR_ENTER        = 2,
  SYSTEMVIEW_EVENT_ISR_EXIT         = 3,
  SYSTEMVIEW_EVENT_TASK_START_EXEC  = 4,
  SYSTEMVIEW_EVENT_TASK_STOP_EXEC   = 5,
  SYSTEMVIEW_EVENT_TASK_START_READY = 6,
  SYSTEMVIEW_EVENT_TASK_STOP_READY  = 7,
  SYSTEMVIEW_EVENT_TASK_CREATE      = 8,
  SYSTEMVIEW_EVENT_TASK_INFO        = 9,
  SYSTEMVIEW_EVENT_TRACE_START      = 10,
  SYSTEMVIEW_EVENT_TRACE_STOP       = 11,
  SYSTEMVIEW_EVENT_SYSTIME_CYCLES   = 12,
  SYSTEMVIEW_EVENT_SYSTIME_US       = 13,
  SYSTEMVIEW_EVENT_SYSDESC          = 14,
  SYSTEMVIEW_EVENT_USER_START       = 15,
  SYSTEMVIEW_EVENT_USER_STOP        = 16,
  SYSTEMVIEW_EVENT_IDLE             = 17,
  SYSTEMVIEW_EVENT_ISR_TO_SCHEDULER = 18,
  SYSTEMVIEW_EVENT_TIMER_ENTER      = 19,
  SYSTEMVIEW_EVENT_TIMER_EXIT       = 20,
  SYSTEMVIEW_EVENT_STACK_INFO       = 21,
  SYSTEMVIEW_EVENT_MODULEDESC       = 22,
  SYSTEMVIEW_EVENT_KNOWN_LAST       = 23,

  // With length
  SYSTEMVIEW_EVENT_INIT             = 24,
  SYSTEMVIEW_EVENT_NAME_RESOURCE    = 25,
  SYSTEMVIEW_EVENT_PRINT_FORMATTED  = 26,
  SYSTEMVIEW_EVENT_NUMMODULES       = 27,

  // MutekH-specific
  SYSTEMVIEW_MUTEKH_KROUTINE_INIT = 33,
  SYSTEMVIEW_MUTEKH_KROUTINE_EXEC = 34,
  SYSTEMVIEW_MUTEKH_KROUTINE_BEGIN = 35,
  SYSTEMVIEW_MUTEKH_KROUTINE_END = 36,

  SYSTEMVIEW_MUTEKH_MEM_ALLOC = 37,
  SYSTEMVIEW_MUTEKH_MEM_FREE = 38,

  SYSTEMVIEW_MUTEKH_DEVICE_OP_BEGIN = 39,
  SYSTEMVIEW_MUTEKH_DEVICE_OP_END = 40,

  SYSTEMVIEW_MUTEKH_BYTECODE_VM_OP = 41,
};

enum systemview_command_id_e
{
  SYSTEMVIEW_COMMAND_START = 1,
  SYSTEMVIEW_COMMAND_STOP,
  SYSTEMVIEW_COMMAND_GET_SYSTIME,
  SYSTEMVIEW_COMMAND_GET_TASKLIST,
  SYSTEMVIEW_COMMAND_GET_SYSDESC,
  SYSTEMVIEW_COMMAND_GET_NUMMODULES,
  SYSTEMVIEW_COMMAND_GET_MODULEDESC,

  // With a second parameter
  SYSTEMVIEW_COMMAND_GET_MODULE = 128,
};

#endif

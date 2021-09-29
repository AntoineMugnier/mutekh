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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2016
*/

#include <device/valio/hwclock.h>
#include <device/class/valio.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/shell.h>

#include <mutek/mem_alloc.h>
#include <mutek/console.h>
#include <mutek/shell.h>
#include <hexo/enum.h>

enum hwclock_opts_e
{
  HWCLOCK_OPT_DEV  = 0x01,
  HWCLOCK_OPT_DATE = 0x02,
  HWCLOCK_OPT_TIME = 0x04,
  HWCLOCK_OPT_DOW  = 0x08,
  HWCLOCK_OPT_UNIX = 0x10,
};

struct termui_optctx_hwclock_opts_s
{
  struct device_valio_s accessor;
  struct termui_con_string_s date;
  struct termui_con_string_s time;
  enum valio_hwclock_dow_e dow;
};

static TERMUI_CON_ARGS_CLEANUP_PROTOTYPE(hwclock_opts_cleanup)
{
  struct termui_optctx_hwclock_opts_s *c = ctx;

  if (device_check_accessor(&c->accessor.base))
    device_put_accessor(&c->accessor.base);
}

static uint32_t sized_atoi(const char *s, size_t size)
{
  uint32_t ret = 0;

  for (size_t i = 0; i < size; ++i) {
    ret *= 10;
    ret += s[i] - '0';
  }

  return ret;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_hwclock_set)
{
  struct termui_optctx_hwclock_opts_s *c = ctx;
  error_t err = 0;
  struct valio_hwclock_s date;

  if ((used & (HWCLOCK_OPT_DATE | HWCLOCK_OPT_TIME | HWCLOCK_OPT_DOW))
      != (HWCLOCK_OPT_DATE | HWCLOCK_OPT_TIME | HWCLOCK_OPT_DOW)) {
    err = dev_valio_wait_op(DEVICE_VALIO_READ, &c->accessor, VALIO_HWCLOCK_DATE, &date);
    if (err) {
      termui_con_printf(con, "Cannot read current date: %d\n", err);

      return err;
    }
  }

  if (used & HWCLOCK_OPT_DATE) {
    if (c->date.len != 10
        || c->date.str[4] != '-'
        || c->date.str[7] != '-') {
      termui_con_printf(con, "Expecting date as YYYY-MM-DD format\n");
      return -EINVAL;
    }

    date.year = sized_atoi(c->date.str, 4);
    date.month = sized_atoi(c->date.str + 5, 2);
    date.day = sized_atoi(c->date.str + 8, 2);
  }

  if (used & HWCLOCK_OPT_TIME) {
    if (c->time.len != 8
        || c->time.str[2] != ':'
        || c->time.str[5] != ':') {
      termui_con_printf(con, "Expecting time as HH:MM:SS format\n");
      return -EINVAL;
    }

    date.hour = sized_atoi(c->time.str, 2);
    date.min = sized_atoi(c->time.str + 3, 2);
    date.sec = sized_atoi(c->time.str + 6, 2);
  }

  if (used & HWCLOCK_OPT_DOW)
    date.dow = c->dow;

  err = dev_valio_wait_op(DEVICE_VALIO_WRITE, &c->accessor, VALIO_HWCLOCK_DATE, &date);
  if (err)
    termui_con_printf(con, "Request failed with error: %d\n", err);

  return err;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_hwclock_get)
{
  struct termui_optctx_hwclock_opts_s *c = ctx;
  error_t err = 0;
  struct valio_hwclock_s date;

  err = dev_valio_wait_op(DEVICE_VALIO_READ, &c->accessor, VALIO_HWCLOCK_DATE, &date);
  if (err) {
    termui_con_printf(con, "Cannot read current date: %d\n", err);

    return err;
  }

  if (used & HWCLOCK_OPT_UNIX) {
    termui_con_printf(con, "timestamp: %d\n",
                      valio_hwclock_to_epoch(&date));
  } else {
    termui_con_printf(con, "%N %04d-%02d-%02d %02d:%02d:%02d\n",
                      date.dow, ENUM_DESC_VALIO_HWCLOCK_DOW_E,
                      date.year, date.month, date.day,
                      date.hour, date.min, date.sec);
  }

  return err;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_hwclock_utc)
{
  struct valio_hwclock_s date;

  valio_hwclock_from_epoch(&date, atoi(argv[0]));

  termui_con_printf(con, "%N %04d-%02d-%02d %02d:%02d:%02d\n",
                    date.dow, ENUM_DESC_VALIO_HWCLOCK_DOW_E,
                    date.year, date.month, date.day,
                    date.hour, date.min, date.sec);

  return 0;
}

static TERMUI_CON_OPT_DECL(hwclock_opts_s) =
{
  TERMUI_CON_OPT_DEV_ACCESSOR_ENTRY("-d", "--dev", HWCLOCK_OPT_DEV,
                                    struct termui_optctx_hwclock_opts_s, accessor, DRIVER_CLASS_VALIO,
                                    TERMUI_CON_OPT_CONSTRAINTS(HWCLOCK_OPT_DEV, 0)
                                    )

  TERMUI_CON_OPT_ENTRY("-u", "--unix", HWCLOCK_OPT_UNIX,
                       TERMUI_CON_OPT_CONSTRAINTS(HWCLOCK_OPT_UNIX, HWCLOCK_OPT_UNIX))

  TERMUI_CON_OPT_STRING_ENTRY("-D", "--date", HWCLOCK_OPT_DATE, struct termui_optctx_hwclock_opts_s, date, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(HWCLOCK_OPT_DATE, HWCLOCK_OPT_DATE | HWCLOCK_OPT_DEV))

  TERMUI_CON_OPT_STRING_ENTRY("-T", "--time", HWCLOCK_OPT_TIME, struct termui_optctx_hwclock_opts_s, time, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(HWCLOCK_OPT_TIME, HWCLOCK_OPT_TIME | HWCLOCK_OPT_DATE | HWCLOCK_OPT_DEV))

  TERMUI_CON_OPT_ENUM_ENTRY("-W", "--dow", HWCLOCK_OPT_DOW, struct termui_optctx_hwclock_opts_s, dow, ENUM_DESC_VALIO_HWCLOCK_DOW_E,
                            TERMUI_CON_OPT_CONSTRAINTS(HWCLOCK_OPT_DOW, HWCLOCK_OPT_DOW | HWCLOCK_OPT_TIME | HWCLOCK_OPT_DATE | HWCLOCK_OPT_DEV))

  TERMUI_CON_LIST_END
};

TERMUI_CON_GROUP_DECL(dev_shell_hwclock_group) =
{
  TERMUI_CON_ENTRY(shell_hwclock_get, "get",
    TERMUI_CON_OPTS_CTX(hwclock_opts_s,
                        HWCLOCK_OPT_DEV,
                        HWCLOCK_OPT_DEV | HWCLOCK_OPT_UNIX,
                        hwclock_opts_cleanup)
  )

  TERMUI_CON_ENTRY(shell_hwclock_utc, "utc",
    TERMUI_CON_ARGS(1, 1)
  )

  TERMUI_CON_ENTRY(shell_hwclock_set, "set",
    TERMUI_CON_OPTS_CTX(hwclock_opts_s,
                        HWCLOCK_OPT_DEV | HWCLOCK_OPT_DATE | HWCLOCK_OPT_TIME | HWCLOCK_OPT_DOW,
                        HWCLOCK_OPT_DEV | HWCLOCK_OPT_DATE | HWCLOCK_OPT_TIME | HWCLOCK_OPT_DOW,
                        hwclock_opts_cleanup)
  )

  TERMUI_CON_LIST_END
};

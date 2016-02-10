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

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/interrupt.h>
#include <string.h>
#include <mutek/startup.h>
#include <mutek/printk.h>
#include <hexo/ordering.h>
#include <hexo/cpu.h>

#include <drivers/rtt/rtt.h>

#include <systemview/event.h>
#include <systemview/log.h>

#define SV_HEADER_SIZE 4
#define SV_UINT32_SIZE 5
#define SV_FOOTER_SIZE SV_UINT32_SIZE
#define SV_MIN_SIZE (SV_HEADER_SIZE + SV_FOOTER_SIZE)

static uint8_t sv_tx_buffer[CONFIG_SYSTEMVIEW_TX_BUFFER_SIZE];
static uint8_t sv_rx_buffer[CONFIG_SYSTEMVIEW_RX_BUFFER_SIZE];
static struct rtt_channel_s *sv_tx, *sv_rx;
static uint64_t sv_last_write_ts = 0;
static uint32_t sv_dropped_count = 0;
static bool_t sv_enabled = 0;

#define _STRINGIFY(x) #x
#define STRINGIFY(x) _STRINGIFY(x)

static
void systemview_start(void)
{
    sv_enabled = 1;
    systemview_log(SYSTEMVIEW_EVENT_TRACE_START);
    systemview_log_4(SYSTEMVIEW_EVENT_INIT, 16000000, 32768, 0x20000000, __builtin_ctz(CONFIG_MUTEK_MEMALLOC_ALIGN));
    systemview_log_sysdesc("N="STRINGIFY(BUILD_OUTPUT_NAME)",O=MutekH,D="STRINGIFY(BUILD_BUILD_NAME));
    systemview_log_1(SYSTEMVIEW_EVENT_SYSTIME_CYCLES, systemview_timestamp_get());
    systemview_log_module_count(0);
}

void init_systemview(void)
{
  sv_tx = rtt_channel_init(
    RTT_CHANNEL_TX_ID(CONFIG_SYSTEMVIEW_INSTRUMENTATION_CHANNEL_FIRST),
    "SysView",
    sv_tx_buffer, CONFIG_SYSTEMVIEW_TX_BUFFER_SIZE,
    RTT_CHANNEL_MODE_SKIP);

  sv_rx = rtt_channel_init(
    RTT_CHANNEL_RX_ID(CONFIG_SYSTEMVIEW_INSTRUMENTATION_CHANNEL_FIRST),
    "SysView",
    sv_rx_buffer, CONFIG_SYSTEMVIEW_RX_BUFFER_SIZE,
    RTT_CHANNEL_MODE_SKIP);
}

static uint8_t *sv_uint32_put(uint8_t *ptr, uint32_t val)
{
  do {
    uint8_t byte = val;

    val >>= 7;
    if (val)
      byte |= 0x80;

    *(ptr++) = byte;
  } while (val);

  return ptr;
}

static uint8_t *sv_string_put(uint8_t *ptr, const char *val, uint32_t maxsize)
{
  uint32_t len = val ? strlen(val) : 0;

  if (len >= maxsize)
    len = maxsize;

  if (len == 0) {
    *ptr++ = 1;
    *ptr++ = '-';
    return ptr;
  }

  ptr = sv_uint32_put(ptr, len);
  memcpy(ptr, val, len);

  return ptr + len;
}

static uint8_t *sv_uint16_put_rev(uint8_t *ptr, uint16_t val)
{
  uint8_t hi = val >> 7;
  uint8_t lo = val;

  if (hi) {
    ptr[-2] = lo | 0x80;
    ptr[-1] = hi;
    return ptr - 2;
  } else {
    ptr[-1] = lo;
    return ptr - 1;
  }
}

static
bool_t systemview_packet_send(uint32_t id, uint8_t *begin, uint8_t *end)
{
  static uint32_t cmd_count = 0;

  if (id <= SYSTEMVIEW_EVENT_KNOWN_LAST) {
    --begin;
    *begin = id;
  } else {
    begin = sv_uint16_put_rev(begin, end - begin);
    begin = sv_uint16_put_rev(begin, id);
  }

  uint64_t now = systemview_timestamp_get();
  uint32_t ts_delta = now - sv_last_write_ts;
  end = sv_uint32_put(end, ts_delta);

  ++cmd_count;

  //  printk("SV Packet %d @%d %P\n", cmd_count, sv_tx->write_ptr, begin, end - begin);

  uint32_t written;

  written = rtt_channel_write(sv_tx, begin, end - begin);

  if (!written) {
    sv_dropped_count++;
    return 1;
  }

  sv_dropped_count = 0;
  sv_last_write_ts = now;
  return 0;
}

static
bool_t systemview_overflow_send(void)
{
  uint8_t buffer[SV_MIN_SIZE + SV_UINT32_SIZE];
  uint8_t *begin, *end;
  begin = end = buffer + SV_HEADER_SIZE;

  end = sv_uint32_put(end, sv_dropped_count);
  return systemview_packet_send(SYSTEMVIEW_EVENT_OVERFLOW, begin, end);
}

static void systemview_handle_command(void)
{
  uint8_t cmd;
  size_t count;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  count = rtt_channel_read(sv_rx, &cmd, 1);
  CPU_INTERRUPT_RESTORESTATE;
  if (!count)
    return;

  switch (cmd) {
  case SYSTEMVIEW_COMMAND_START:
    systemview_start();
    break;

  case SYSTEMVIEW_COMMAND_STOP:
    sv_enabled = 0;
    break;

  case SYSTEMVIEW_COMMAND_GET_SYSTIME:
    systemview_log_1(SYSTEMVIEW_EVENT_SYSTIME_CYCLES, systemview_timestamp_get());
    break;

  case SYSTEMVIEW_COMMAND_GET_TASKLIST:
    break;

  case SYSTEMVIEW_COMMAND_GET_SYSDESC:
    systemview_log_4(SYSTEMVIEW_EVENT_INIT, 16000000, 32768, 0x20000000, __builtin_ctz(CONFIG_MUTEK_MEMALLOC_ALIGN));
    break;

  case SYSTEMVIEW_COMMAND_GET_NUMMODULES:
    systemview_log_module_count(0);
    break;

  case SYSTEMVIEW_COMMAND_GET_MODULEDESC:
    break;

  case SYSTEMVIEW_COMMAND_GET_MODULE:
    CPU_INTERRUPT_SAVESTATE_DISABLE;
    rtt_channel_read(sv_rx, &cmd, 1);
    CPU_INTERRUPT_RESTORESTATE;
    break;

  default:
    if (cmd >= 128) {
      CPU_INTERRUPT_SAVESTATE_DISABLE;
      rtt_channel_read(sv_rx, &cmd, 1);
      CPU_INTERRUPT_RESTORESTATE;
    }
    break;
  }
}

static
void systemview_log_append(uint32_t id, uint8_t *begin, uint8_t *end)
{
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  if (sv_enabled) {
    if (!(sv_dropped_count && systemview_overflow_send()))
      systemview_packet_send(id, begin, end);
  }
  CPU_INTERRUPT_RESTORESTATE;

  if (rtt_channel_has_data(sv_rx))
    systemview_handle_command();
}

void init_systemview_start(void)
{
#if defined(CONFIG_SYSTEMVIEW_WAIT)
  while (!sv_enabled) {
    if (rtt_channel_has_data(sv_rx))
      systemview_handle_command();
    order_compiler_mem();
  }
#else
  systemview_start();
#endif
}

void systemview_log(uint32_t id)
{
  uint8_t buffer[SV_MIN_SIZE];
  uint8_t *begin, *end;
  begin = end = buffer + SV_HEADER_SIZE;

  systemview_log_append(id, begin, end);
}

void systemview_log_1(uint32_t id, uint32_t a0)
{
  uint8_t buffer[SV_MIN_SIZE + SV_UINT32_SIZE];
  uint8_t *begin, *end;
  begin = end = buffer + SV_HEADER_SIZE;

  end = sv_uint32_put(end, a0);
  systemview_log_append(id, begin, end);
}

void systemview_log_2(uint32_t id, uint32_t a0, uint32_t a1)
{
  uint8_t buffer[SV_MIN_SIZE + SV_UINT32_SIZE * 2];
  uint8_t *begin, *end;
  begin = end = buffer + SV_HEADER_SIZE;

  end = sv_uint32_put(end, a0);
  end = sv_uint32_put(end, a1);
  systemview_log_append(id, begin, end);
}

void systemview_log_3(uint32_t id, uint32_t a0, uint32_t a1, uint32_t a2)
{
  uint8_t buffer[SV_MIN_SIZE + SV_UINT32_SIZE * 3];
  uint8_t *begin, *end;
  begin = end = buffer + SV_HEADER_SIZE;

  end = sv_uint32_put(end, a0);
  end = sv_uint32_put(end, a1);
  end = sv_uint32_put(end, a2);
  systemview_log_append(id, begin, end);
}

void systemview_log_4(uint32_t id, uint32_t a0, uint32_t a1, uint32_t a2, uint32_t a3)
{
  uint8_t buffer[SV_MIN_SIZE + SV_UINT32_SIZE * 4];
  uint8_t *begin, *end;
  begin = end = buffer + SV_HEADER_SIZE;

  end = sv_uint32_put(end, a0);
  end = sv_uint32_put(end, a1);
  end = sv_uint32_put(end, a2);
  end = sv_uint32_put(end, a3);
  systemview_log_append(id, begin, end);
}

void systemview_log_sysdesc(const char *desc)
{
  uint8_t buffer[SV_MIN_SIZE + 32 + 1];
  uint8_t *begin, *end;
  begin = end = buffer + SV_HEADER_SIZE;

  end = sv_string_put(end, desc, 32);
  systemview_log_append(SYSTEMVIEW_EVENT_SYSDESC, begin, end);
}

void systemview_log_task_info(uint32_t id, const char *name, uint32_t prio)
{
  uint8_t buffer[SV_MIN_SIZE + SV_UINT32_SIZE * 2 + 32 + 1];
  uint8_t *begin, *end;
  begin = end = buffer + SV_HEADER_SIZE;

  end = sv_uint32_put(end, SYSTEMVIEW_ID_SHRINK(id));
  end = sv_uint32_put(end, prio);
  end = sv_string_put(end, name, 32);
  systemview_log_append(SYSTEMVIEW_EVENT_TASK_INFO, begin, end);
}

void systemview_log_module_desc(uint32_t id, uint32_t first_event, const char *desc)
{
  uint8_t buffer[SV_MIN_SIZE + SV_UINT32_SIZE * 2 + 32 + 1];
  uint8_t *begin, *end;
  begin = end = buffer + SV_HEADER_SIZE;

  end = sv_uint32_put(end, id);
  end = sv_uint32_put(end, first_event);
  end = sv_string_put(end, desc, 32);
  systemview_log_append(SYSTEMVIEW_EVENT_MODULEDESC, begin, end);
}

void systemview_log_name_resource(uint32_t id, const char *name)
{
  uint8_t buffer[SV_MIN_SIZE + SV_UINT32_SIZE + 32 + 1];
  uint8_t *begin, *end;
  begin = end = buffer + SV_HEADER_SIZE;

  end = sv_uint32_put(end, SYSTEMVIEW_ID_SHRINK(id));
  end = sv_string_put(end, name, 32);
  systemview_log_append(SYSTEMVIEW_EVENT_NAME_RESOURCE, begin, end);
}


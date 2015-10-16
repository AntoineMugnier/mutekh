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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2014
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <string.h>

#include <mutek/printk.h>

#include <drivers/rtt/rtt.h>

static uint8_t rtt_printk_ringbuffer[CONFIG_RTT_PRINTK_RINGBUFFER_SIZE];

static PRINTF_OUTPUT_FUNC(rtt_printk_out)
{
  struct rtt_channel_s *chan = ctx;
  uint32_t done = 0;
  bool_t blocking = !!(chan->flags & RTT_CHANNEL_MODE_BLOCKING);

  while (done < len && blocking)
    done += rtt_channel_write(chan, (const uint8_t *)str + done, len - done);
}

void rtt_printk_init(void);
void rtt_printk_init(void)
{
  struct rtt_channel_s *chan = rtt_channel_init(
    RTT_CHANNEL_TX_ID(CONFIG_DRIVER_RTT_PRINTK_OUT_FIRST),
    "Printk Output",
    rtt_printk_ringbuffer, CONFIG_RTT_PRINTK_RINGBUFFER_SIZE,
    RTT_CHANNEL_MODE_BLOCKING);

  printk_set_output(rtt_printk_out, chan);
}

/*
  Get rtt_printk_control address with objdump, then configure term
  input in JLink.

  This can be done from JLinkExe:
  - Run JLinkExe
  - telnet localhost 19021
  - type "exec SetRTTAddr <control_addr>" in JLink prompt

  This can also be done from JLinkGDBServer:
  - Run JLinkGDBServer
  - telnet localhost 19021
  - type "monitor exec SetRTTAddr <control_addr>" in gdb prompt


  Example:

  $ telnet localhost 19021
  Connected to localhost.
  Escape character is '^]'.
  SEGGER J-Link V4.95c (beta) - Real time terminal output
  Process: JLinkExe
  ...

  $ JLinkExe
  ...
  J-Link>speed 4000
  J-Link>exec SetRTTAddr 0x20000050
  J-Link>r
  Reset delay: 0 ms
  Reset type NORMAL: Resets core & peripherals via SYSRESETREQ & VECTRESET bit.
  J-Link>g


  "SetRTTAddr <address>" can be replaced with "SetRTTSearchRanges
  <base address> <end address>", this will search for control block
  token in memory.
*/

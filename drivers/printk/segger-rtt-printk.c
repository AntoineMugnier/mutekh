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

#include "segger-rtt.h"

static uint8_t rtt_printk_ringbuffer[CONFIG_RTT_PRINTK_RINGBUFFER_SIZE];

static struct {
  struct rtt_s control;
  struct rtt_ringbuffer_s ring;
} rtt_printk_control = {
  .control = RTT_INITIALIZER("", 1, 0),
  .ring = RTT_RINGBUFFER_INITIALIZER("Terminal", rtt_printk_ringbuffer,
                                     RTT_RINGBUFFER_MODE_BLOCKING),
};

static PRINTF_OUTPUT_FUNC(rtt_printk_out)
{
  struct rtt_s *rtt = ctx;
  size_t done = 0;
  bool_t blocking = !!(rtt->buffer[0].flags & RTT_RINGBUFFER_MODE_BLOCKING);

  while (done < len && blocking)
    done += rtt_ringbuffer_write(&rtt->buffer[0],
                                 (const uint8_t *)str + done,
                                 len - done);
}

void rtt_printk_init()
{
  // Avoid having one instance of the token in .rodata
  strcpy(rtt_printk_control.control.id, "SEGGER");
  strcat(rtt_printk_control.control.id, " RTT");
  printk_set_output(rtt_printk_out, (void*)&rtt_printk_control.control);
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

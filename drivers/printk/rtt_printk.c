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

  rtt_channel_write(chan, (const uint8_t *)str, len);
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

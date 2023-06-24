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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2023
*/

#include <drivers/arm/identification.h>
#include <hexo/iospace.h>
#include <mutek/printk.h>

#define DEVID_OFFSET 0x00000fc0
#define PID1_OFFSET 0x00000fd0
#define PID0_OFFSET 0x00000fe0
#define CID_OFFSET 0x00000ff0

static uint32_t multi_word_get(uintptr_t base)
{
    uint32_t ret = 0;
    for (uintptr_t addr = base + 0xc; addr >= base; addr -= 4) {
        ret <<= 8;
        ret |= cpu_mem_read_32(addr) & 0xff;
    }
    return ret;
}

void arm_identification_read(struct arm_identification_s *id, uintptr_t base_addr)
{
    base_addr &= ~0xfff;

    id->cid = multi_word_get(base_addr | CID_OFFSET);
    id->pid0 = multi_word_get(base_addr | PID0_OFFSET);
    id->pid1 = multi_word_get(base_addr | PID1_OFFSET);
    id->devid = multi_word_get(base_addr | DEVID_OFFSET);
}

void arm_identification_dump(const struct arm_identification_s *id)
{
    logk("CID: %08x, PID0: %08x, PID1: %08x, DEVID: %08x",
           id->cid, id->pid0, id->pid1, id->devid);
}

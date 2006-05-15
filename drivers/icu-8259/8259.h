/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/


#ifndef PIC_8259_H_
#define PIC_8259_H_

#define PIC_ICW1_INIT		0x10
#define PIC_ICW1_HAS_ICW4	0x01
#define PIC_ICW1_NO_SLAVE	0x02
#define PIC_ICW1_TRIGGER_MODE	0x08

//      PIC_ICW2 is irq line 0 int vector number

#define PIC_ICW3_CHAIN_LINE(n) (1 << (n))

#define PIC_ICW4_X86_MODE	0x01
#define PIC_ICW4_AUTO_IRQ_END	0x02

#define PIC_OCW1_DISABLED(n)	(1 << (n))

#define PIC_OCW2_LINE(n)	(n) & 0x07)
#define PIC_OCW2_EOI		0x20

#define PIC_OCW2_PRIO_NOP	0x40


#define PIC_SLAVE_CHAINED_IRQ	2 /* IRQ line on master used for slave */

#define PIC_DEFAULT_MASTER_MASK	(0xff ^ PIC_OCW1_DISABLED(PIC_SLAVE_CHAINED_IRQ))

static inline void
pic_8259_setmask(uintptr_t pic, uint8_t mask)
{
  cpu_io_write_8(pic + 1, mask);
}

static inline uint8_t
pic_8259_getmask(uintptr_t pic)
{
  return cpu_io_read_8(pic + 1);
}

static inline void
pic_8259_enable(uintptr_t pic, uint_fast8_t line)
{
  uint8_t	x = cpu_io_read_8(pic + 1);
  cpu_io_write_8(pic + 1, x & ~(1 << line));
}


static inline void
pic_8259_disable(uintptr_t pic, uint_fast8_t line)
{
  uint8_t	x = cpu_io_read_8(pic + 1);
  cpu_io_write_8(pic + 1, x | (1 << line));
}

static inline void
pic_8259_irqend(uintptr_t pic, uint_fast8_t line)
{
  cpu_io_write_8(pic, PIC_OCW2_EOI | PIC_OCW2_PRIO_NOP | line);
}

static inline void
pic_8259_irqend_slave(uintptr_t master, uintptr_t slave, uint_fast8_t line)
{
  pic_8259_irqend(slave, line);
  pic_8259_irqend(master, PIC_SLAVE_CHAINED_IRQ);
}

static inline void
pic_8259_irqend_master(uintptr_t master, uint_fast8_t line)
{
  pic_8259_irqend(master, line);
}

static inline void
pic_8259_init(uintptr_t master, uintptr_t slave, uint8_t base)
{
  /* master init */
  cpu_io_write_8(master    , PIC_ICW1_INIT | PIC_ICW1_HAS_ICW4);
  cpu_io_write_8(master + 1, base);
  cpu_io_write_8(master + 1, PIC_ICW3_CHAIN_LINE(PIC_SLAVE_CHAINED_IRQ));
  cpu_io_write_8(master + 1, PIC_ICW4_X86_MODE);

  pic_8259_setmask(master, PIC_DEFAULT_MASTER_MASK);
  pic_8259_irqend_master(master, PIC_SLAVE_CHAINED_IRQ);

  /* slave init */
  cpu_io_write_8(slave    , PIC_ICW1_INIT | PIC_ICW1_HAS_ICW4);
  cpu_io_write_8(slave + 1, base + 8);
  cpu_io_write_8(slave + 1, PIC_SLAVE_CHAINED_IRQ);
  cpu_io_write_8(slave + 1, PIC_ICW4_X86_MODE);

  pic_8259_setmask(slave, 0xff);
}

#endif


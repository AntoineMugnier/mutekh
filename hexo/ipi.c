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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2009
    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2009

*/

#include <device/icu.h>
#include <device/driver.h>
#include <device/device.h>
#include <hexo/ipi.h>

static CPU_LOCAL ipi_queue_root_t ipi_fifo = CONTAINER_ROOT_INITIALIZER(ipi_queue, DLIST);
CPU_LOCAL struct device_s *ipi_icu_dev = 0;
CPU_LOCAL void *ipi_cpu_id;

error_t ipi_post(void *cpu_cls)
{
  struct device_s *icu = *CPU_LOCAL_CLS_ADDR(cpu_cls, ipi_icu_dev);

  if (!icu)
    return -EOPNOTSUPP;

  return dev_icu_sendipi(icu, *CPU_LOCAL_CLS_ADDR(cpu_cls, ipi_cpu_id));
}

error_t ipi_post_rq(void *cpu_cls, struct ipi_request_s *rq)
{
  if (ipi_queue_pushback(CPU_LOCAL_CLS_ADDR(cpu_cls, ipi_fifo), rq))
    return ipi_post(cpu_cls);

  return -ENOMEM;
}

void ipi_process_rq()
{
  struct ipi_request_s *rq;

  while ((rq = ipi_queue_pop(CPU_LOCAL_ADDR(ipi_fifo))))
    rq->func(rq->private);
}

void ipi_hook_cpu(void *cpu_cls,
				  struct device_s *ipi_icudev,
				  void *privdata)
{
	struct device_s **icu = CPU_LOCAL_CLS_ADDR(cpu_cls, ipi_icu_dev);
	void ** priv = CPU_LOCAL_CLS_ADDR(cpu_cls, ipi_cpu_id);

	*icu = ipi_icudev;
	*priv = privdata;
}

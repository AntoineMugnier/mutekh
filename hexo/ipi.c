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

CONTAINER_FUNC(ipi_queue, DLIST, static inline, ipi_queue);

CPU_LOCAL struct ipi_endpoint_s ipi_endpoint = {};

error_t ipi_post(struct ipi_endpoint_s *endpoint)
{
    struct device_s *icu = endpoint->icu_dev;

    if (!icu)
        return -EOPNOTSUPP;

    return dev_icu_sendipi(icu, endpoint->priv);
}

error_t ipi_post_rq(struct ipi_endpoint_s *endpoint, struct ipi_request_s *rq)
{
    if (ipi_queue_pushback(&endpoint->ipi_fifo, rq))
        return ipi_post(endpoint);

    return -ENOMEM;
}

void ipi_process_rq()
{
    struct ipi_request_s *rq;
    ipi_queue_root_t *fifo = &(CPU_LOCAL_ADDR(ipi_endpoint)->ipi_fifo);

    while ((rq = ipi_queue_pop(fifo)))
        rq->func(rq->private);
}

error_t ipi_hook_endpoint(struct ipi_endpoint_s *endpoint,
                          struct device_s *ipi_dev,
                          uint_fast8_t ipi_no)
{
    void *foo = dev_icu_setupipi(ipi_dev, ipi_no);
    endpoint->icu_dev = ipi_dev;
    endpoint->priv = foo;

    return 0;
}

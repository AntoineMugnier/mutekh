/*
 * This file is part of MutekH.
 * 
 * MutekH is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation; version 2.1 of the License.
 * 
 * MutekH is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with MutekH; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * Copyright (c) UPMC, Lip6, SoC
 *         Nicolas Pouillon <nipo@ssji.net>, 2008
 */

#include <hexo/scheduler.h>
#include <hexo/error.h>
#include <assert.h>
#include <srl/srl_public_types.h>
#include <srl_private_types.h>

void srl_barrier_wait(srl_barrier_t barrier)
{
	CPU_INTERRUPT_SAVESTATE_DISABLE;
	sched_queue_wrlock(&barrier->wait);

	assert(barrier->count >= 1);

	if (barrier->count == 1)
    {
		while (sched_wake(&barrier->wait) != NULL)
			barrier->count++;
		sched_queue_unlock(&barrier->wait);
    }
	else
    {
		barrier->count--;
		sched_wait_unlock(&barrier->wait);
    }

	CPU_INTERRUPT_RESTORESTATE;
}


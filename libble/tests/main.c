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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2015
*/

#include <assert.h>

#include <mutek/thread.h>
#include <mutek/startup.h>

void cmac_test(void);
void sm_test(void);
void ccm_test(void);
void speed_test(void);

static CONTEXT_ENTRY(tests)
{
  sm_test();
  cmac_test();
  ccm_test();
  speed_test();
}

void app_start()
{
    thread_create(tests, 0, NULL);
}

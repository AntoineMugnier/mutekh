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

  Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2016
*/

static inline
bool_t psoc4_blerd_is_powered(void)
{
  return !!(cpu_mem_read_32(BLESS + BLESS_RF_CONFIG_ADDR)
            & BLESS_RF_CONFIG_RF_ENABLE);
}

static inline
void psoc4_blerd_power_enable(void)
{
  error_t err;
  uint32_t tmp;

  assert(psoc4_lfclk_is_running());

  // Enable RF power
  tmp = cpu_mem_read_32(BLESS + BLESS_RF_CONFIG_ADDR);
  tmp |= BLESS_RF_CONFIG_RF_ENABLE;
  cpu_mem_write_32(BLESS + BLESS_RF_CONFIG_ADDR, tmp);
}

static inline
void psoc4_blerd_power_disable(void)
{
  uint32_t tmp;

  assert(psoc4_lfclk_is_running());
  assert(!psoc4_blerd_eco_is_running());

  dprintk("%s\n", __FUNCTION__);

  // Disable RF power
  tmp = cpu_mem_read_32(BLESS + BLESS_RF_CONFIG_ADDR);
  tmp |= BLESS_RF_CONFIG_RF_ENABLE;
  cpu_mem_write_32(BLESS + BLESS_RF_CONFIG_ADDR, tmp);
}

static
bool_t psoc4_clock_eco_start(void)
{
  if (!psoc4_lfclk_is_running()) {
    return 0;
  }

  if (!psoc4_blerd_is_powered())
    psoc4_blerd_power_enable();

  if (psoc4_clock_eco_is_running())
    return 1;

  
  return 1;
}

static
bool_t psoc4_clock_wco_start(void)
{
}

static inline
bool_t psoc4_blerd_eco_is_running(void)
{
  return !!(cpu_mem_read_32(BLERD + BLERD_DBUS_ADDR) & BLERD_DBUS_XTAL_ENABLE);
}

static inline
void psoc4_ble_eco_irq_clear(void)
{
  uint32_t tmp;

  dprintk("%s\n", __FUNCTION__);

  // Disable IRQ
  tmp = cpu_mem_read_32(BLESS + BLESS_LL_DSM_CTRL_ADDR);
  tmp &= ~BLESS_LL_DSM_CTRL_XTAL_ON_INTR_EN;
  cpu_mem_write_32(BLESS + BLESS_LL_DSM_CTRL_ADDR, tmp);

  // Clear IRQ
  cpu_mem_write_32(BLESS + BLESS_LL_DSM_INTR_STAT_ADDR,
                   BLESS_LL_DSM_INTR_STAT_XTAL_ON_INTR);
}

static inline
void psoc4_ble_eco_start(void)
{
  uint32_t dbus;
  uint32_t tmp;
  error_t err;

  dprintk("%s\n", __FUNCTION__);

  assert(psoc4_lfclk_is_running());
  assert(psoc4_blerd_is_powered());

  // Clear IRQ
  cpu_mem_write_32(BLESS + BLESS_LL_DSM_INTR_STAT_ADDR,
                   BLESS_LL_DSM_INTR_STAT_XTAL_ON_INTR);

  // Enable IRQ
  tmp = cpu_mem_read_32(BLESS + BLESS_LL_DSM_CTRL_ADDR);
  tmp |= BLESS_LL_DSM_CTRL_XTAL_ON_INTR_EN;
  cpu_mem_write_32(BLESS + BLESS_LL_DSM_CTRL_ADDR, tmp);

  // Enable xtal
  dbus = cpu_mem_read_32(BLERD + BLERD_DBUS_ADDR);
  dbus |= BLERD_DBUS_XTAL_ENABLE;
  cpu_mem_write_32(BLERD + BLERD_DBUS_ADDR, dbus);
}

static inline
void psoc4_ble_eco_stop(void)
{
  uint32_t dbus;

  assert(psoc4_lfclk_is_running());
  assert(psoc4_blerd_is_powered());

  dprintk("%s\n", __FUNCTION__);

  // Disable xtal
  dbus = cpu_mem_read_32(BLERD + BLERD_DBUS_ADDR);
  dbus &= ~BLERD_DBUS_XTAL_ENABLE;
  cpu_mem_write_32(BLERD + BLERD_DBUS_ADDR, dbus);
}

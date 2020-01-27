#define LOGK_MODULE_ID "meas"

#include <mutek/printk.h>
#include <enums.h>
#include <hexo/enum.h>
#include <stdlib.h>
#include <stdio.h>
#include <ble/profile/fluke/measurement.h>

const char fluke_reading_state_e[] = ENUM_DESC_FLUKE_READING_STATE_E;
const char fluke_reading_magnitude_e[] = ENUM_DESC_FLUKE_READING_MAGNITUDE_E;
const char fluke_reading_unit_e[] = ENUM_DESC_FLUKE_READING_UNIT_E;
const char fluke_reading_func_e[] = ENUM_DESC_FLUKE_READING_FUNC_E;
const char fluke_reading_attrib_rng_decade_e[] = ENUM_DESC_FLUKE_READING_ATTRIB_RNG_DECADE_E;
const char fluke_reading_attrib_e[] = ENUM_DESC_FLUKE_READING_ATTRIB_E;

struct fluke_unit_desc_s {
  const char *label;
  int8_t base_exp;
};

static const struct fluke_unit_desc_s units[] = {
  [FLUKE_READING_UNIT_NONE] = { "", 0, },
  [FLUKE_READING_UNIT_VAC] = { "V AC", 0, },
  [FLUKE_READING_UNIT_VDC] = { "V DC", 0, },
  [FLUKE_READING_UNIT_AAC] = { "A AC", 0, },
  [FLUKE_READING_UNIT_ADC] = { "A DC", 0, },
  [FLUKE_READING_UNIT_HZ] = { "Hz", 0, },
  [FLUKE_READING_UNIT_RH_PCT] = { "%Rh", 0, },
  [FLUKE_READING_UNIT_DEGC] = { "°C", 0, },
  [FLUKE_READING_UNIT_DEGF] = { "°F", 0, },
  [FLUKE_READING_UNIT_DEG_RANKINE] = { "°R", 0, },
  [FLUKE_READING_UNIT_KELVIN] = { "K", 0, },
  [FLUKE_READING_UNIT_OHMS] = { "Ω", 0, },
  [FLUKE_READING_UNIT_SIEMENS] = { "S", 0, },
  [FLUKE_READING_UNIT_DUTY_PCT] = { "%", 0, },
  [FLUKE_READING_UNIT_SECONDS] = { "s", 0, },
  [FLUKE_READING_UNIT_FARADS] = { "F", 0, },
  [FLUKE_READING_UNIT_DB] = { "dB", 0, },
  [FLUKE_READING_UNIT_DBM] = { "dBm", 0, },
  [FLUKE_READING_UNIT_WATTS] = { "W", 0, },
  [FLUKE_READING_UNIT_JOULE] = { "J", 0, },
  [FLUKE_READING_UNIT_HENRY] = { "H", 0, },
  [FLUKE_READING_UNIT_PSI] = { "PSI", 0, },
  [FLUKE_READING_UNIT_HG_METERS] = { "mHG", 0, },
  [FLUKE_READING_UNIT_HG_INCHES] = { "inHG", 0, },
  [FLUKE_READING_UNIT_H2O_FEET] = { "ftH2O", 0, },
  [FLUKE_READING_UNIT_H2O_METERS] = { "mH2O", 0, },
  [FLUKE_READING_UNIT_H2O_INCHES] = { "inH2O", 0, },
  [FLUKE_READING_UNIT_H2O_60F_INCHES] = { "60FinH2O", 0, },
  [FLUKE_READING_UNIT_BAR] = { "BAR", 0, },
  [FLUKE_READING_UNIT_PASCALS] = { "Pa", 0, },
  [FLUKE_READING_UNIT_G_PER_CM_SQUARED] = { "g/cm2", 0, },
  [FLUKE_READING_UNIT_DBV] = { "dBV", 0, },
  [FLUKE_READING_UNIT_CREST_FACTOR] = { "Crest Factor", 0, },
  [FLUKE_READING_UNIT_VAC_PLUS_DC] = { "V AC+DC", 0, },
  [FLUKE_READING_UNIT_AAC_PLUS_DC] = { "A AC+DC", 0, },
  [FLUKE_READING_UNIT_PERCENT] = { "%", 0, },
  [FLUKE_READING_UNIT_VAC_OVER_HZ] = { "V/Hz AC", 0, },
  [FLUKE_READING_UNIT_ACCELERATION_G] = { "G", 0, },
  [FLUKE_READING_UNIT_ACCELERATION_MPS2] = { "m/s2", 0, },
  [FLUKE_READING_UNIT_VELOCITY_IPS] = { "in/s", 0, },
  [FLUKE_READING_UNIT_VELOCITY_MMPS] = { "m/s", -3, },
  [FLUKE_READING_UNIT_DISPLACEMENT_MILS] = { "mils", 0, },
  [FLUKE_READING_UNIT_DISPLACEMENT_MICRON] = { "m", -9, },
  [FLUKE_READING_UNIT_UNKNOWN] = { "Unknown", 0, },
  [FLUKE_READING_UNIT_TERA_OHMS] = { "Ω", 12, },
};

static const int8_t decade_exp[] = {
  [FLUKE_READING_ATTRIB_RNG_DECADE_NONE] = 0,
  [FLUKE_READING_ATTRIB_RNG_DECADE_TENS] = 1,
  [FLUKE_READING_ATTRIB_RNG_DECADE_HUNDREDS] = 2,
  [FLUKE_READING_ATTRIB_RNG_DECADE_THOUSANDS] = 3,
  [FLUKE_READING_ATTRIB_RNG_DECADE_MILLI] = -3,
  [FLUKE_READING_ATTRIB_RNG_DECADE_CENTI] = -2,
  [FLUKE_READING_ATTRIB_RNG_DECADE_DECI] = -1,
};

static const int8_t magnitude_exp[] = {
  [FLUKE_READING_MAGNITUDE_NONE] = 0,
  [FLUKE_READING_MAGNITUDE_GIGA] = 9,
  [FLUKE_READING_MAGNITUDE_MEGA] = 6,
  [FLUKE_READING_MAGNITUDE_KILO] = 3,
  [FLUKE_READING_MAGNITUDE_MILLI] = -3,
  [FLUKE_READING_MAGNITUDE_MICRO] = -6,
  [FLUKE_READING_MAGNITUDE_NANO] = -9,
  [FLUKE_READING_MAGNITUDE_PICO] = -12,
  [FLUKE_READING_MAGNITUDE_TERA] = 12,
  [FLUKE_READING_MAGNITUDE_PETA] = 15,
  [FLUKE_READING_MAGNITUDE_EXA] = 18,
  [FLUKE_READING_MAGNITUDE_ATTO] = -18,
  [FLUKE_READING_MAGNITUDE_FEMTO] = -15,
};

const char *mult_dec[] = {
  "",
  "k", "m",
  "M", "µ",
  "G", "n",
  "T", "p",
  "P", "f",
  "E", "a",
};

void fluke_measurement_value_to_string(const struct fluke_measurement_s *m,
                                       char buffer[static 32])
{
  if (m->reading_state == FLUKE_READING_STATE_OL_RANGE) {
    snprintf(buffer, 32, "OL");
    return;
  }

  const struct fluke_unit_desc_s *unit = &units[m->unit];

  if (m->value == 0x1fffff) {
    snprintf(buffer, 32, "- %s", unit->label);
    return;
  }

  int8_t exp = unit->base_exp - m->decimal_places + magnitude_exp[m->magnitude];
  snprintf(buffer, 16, "%de%d %s", m->value, exp, unit->label);
}

void fluke_measurement_from_blob(struct fluke_measurement_s *m,
                                 const uint8_t blob[static 8])
{
  uint64_t raw_low = endian_le32_na_load(blob);
  uint64_t raw_high = endian_le32_na_load(blob + 4);

  uint32_t abs_value = bit_get_mask(raw_low, 0, 21);
  m->reading_state = bit_get_mask(raw_low, 21, 4);
  m->decimal_places = bit_get_mask(raw_low, 25, 3);
  m->magnitude = bit_get_mask(raw_low, 28, 3);
  uint32_t neg = bit_get_mask(raw_low, 31, 1);
  m->unit = bit_get_mask(raw_high, 0, 8);
  m->function = bit_get_mask(raw_high, 8, 8);
  m->value_range = bit_get_mask(raw_high, 16, 7);
  m->decade = bit_get_mask(raw_high, 23, 3);
  m->attrib = bit_get_mask(raw_high, 26, 5);
  m->capture_flag = bit_get_mask(raw_high, 31, 1);
  m->value = neg ? -abs_value : abs_value;
}

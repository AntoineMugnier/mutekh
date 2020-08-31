
struct _efr32_pa_curve_s {
    int16_t max_pwr_dbm;
    uint16_t slope;
    int32_t offset;
};

struct _efr32_pa_data_s {
    uint8_t data_size;
    const struct _efr32_pa_curve_s *data;
};

static const struct _efr32_pa_curve_s _2g4_vbat[] = {
    { 160, 2776, -300026 },
    { 126, 1335, -73192 },
    { 94, 772, -7179 },
    { 59, 441, 17309 },
    { 43, 343, 22520 },
    { -1, 209, 22360 },
    { -30, 124, 18896 },
    { -74, 40, 10519 }
};

static const struct _efr32_pa_curve_s _2g4_dcdc[] = {
    { 128, 4306, -391604 },
    { 94, 1435, -52495 },
    { 63, 610, 13579 },
    { 31, 331, 24456 },
    { -2, 224, 23902 },
    { -34, 140, 20330 },
    { -74, 37, 10371 }
};

static const struct _efr32_pa_curve_s _sg_vbat[] = {
    { 160, 2757, -319913 },
    { 127, 1173, -64900 },
    { 94, 694, -8378 },
    { 61, 429, 12097 },
    { 30, 263, 18309 },
    { -1, 167, 18071 },
    { -33, 103, 15386 },
    { -70, 34, 9064 }
};

static const struct _efr32_pa_curve_s _sg_dcdc[] = {
    { 128, 9069, -1171644 },
    { 121, 3826, -378994 },
    { 99, 932, -22748 },
    { 63, 470, 13485 },
    { 31, 304, 19712 },
    { -2, 192, 19146 },
    { -31, 110, 15607 },
    { -72, 31, 8239 }
};

#ifdef CONFIG_DRIVER_EFR32_RADIO_VBAT

static struct _efr32_pa_data_s efr32_radio_sg_pa_data = {
    ARRAY_SIZE(_sg_vbat),
    _sg_vbat,
};

// static struct _efr32_pa_data_s efr32_radio_2g4_pa_data = {
//     ARRAY_SIZE(_2g4_vbat),
//     _2g4_vbat,
// };

#else

static struct _efr32_pa_data_s efr32_radio_sg_pa_data = {
    ARRAY_SIZE(_sg_dcdc),
    _sg_dcdc,
};

// static struct _efr32_pa_data_s efr32_radio_2g4_pa_data = {
//     ARRAY_SIZE(_2g4_dcdc),
//     _2g4_dcdc,
// };

#endif
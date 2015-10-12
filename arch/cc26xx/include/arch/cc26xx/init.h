#ifndef _CC26XX_INIT_DEFS_
#define _CC26XX_INIT_DEFS_

#define BIT_BAND_ADDR(x, b)                                                    \
        ((uint32_t)(x) & 0xF0000000) | 0x02000000 |                            \
        (((uint32_t)(x) & 0x000FFFFF) << 5) | ((b) << 2)


// Offset for 4-bit masked access.
// Data bit[n] is written if mask bit[n] is set ('1').
// Bits 7:4 are mask. Bits 3:0 are data.
// Requires 'byte' write.
#define CC26XX_DDI_O_MASK4B     0x00000100

// Offset for 8-bit masked access.
// Data bit[n] is written if mask bit[n] is set ('1').
// Bits 15:8 are mask. Bits 7:0 are data.
// Requires 'short' write.
#define CC26XX_DDI_O_MASK8B     0x00000180

// Offset for 16-bit masked access.
// Data bit[n] is written if mask bit[n] is set ('1').
// Bits 31:16 are mask. Bits 15:0 are data.
// Requires 'long' write.
#define CC26XX_DDI_O_MASK16B    0x00000200

// Offset for 4-bit masked access.
// Data bit[n] is written if mask
// bit[n] is set ('1').
// Bits 7:4 are mask. Bits 3:0 are data.
// Requires 'byte' write.
#define CC26XX_ADI_O_MASK4B     0x00000040

// Offset for 8-bit masked access.
// Data bit[n] is written if mask
// bit[n] is set ('1'). Bits 15:8 are
// mask. Bits 7:0 are data. Requires
// 'short' write.
#define CC26XX_ADI_O_MASK8B     0x00000060

// Offset for 16-bit masked access.
// Data bit[n] is written if mask
// bit[n] is set ('1'). Bits 31:16
// are mask. Bits 15:0 are data.
// Requires 'long' write.
#define CC26XX_ADI_O_MASK16B    0x00000080


#define CC26XX_CCFG_MODE_CONF_VDDR_TRIM_SLEEP_DELTA_WIDTH    4

#endif


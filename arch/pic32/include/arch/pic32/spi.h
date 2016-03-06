/***************************************
* Auto generated by BFGen, do not edit *
***************************************/

/*
   bfgen -o cdefs -O arch/pic32/include/arch/pic32/spi.h cdefs_use_reg_mask=1  \
     cdefs_use_field_setval=1 -I /opt/bfgen/defs/pic32/pic32_spi.bf
*/

#ifndef _PIC32_SPI_BFGEN_DEFS_
#define _PIC32_SPI_BFGEN_DEFS_

#define PIC32_SPI_CON_ADDR                           0x00000000
#define PIC32_SPI_CON_MASK                           0xff83bfff
  #define PIC32_SPI_CON_SRXISEL(v)                 ((PIC32_SPI_CON_SRXISEL_##v) << 0)
  #define PIC32_SPI_CON_SRXISEL_SET(x, v)          do { (x) = (((x) & ~0x3) | ((PIC32_SPI_CON_SRXISEL_##v) << 0)); } while(0)
  #define PIC32_SPI_CON_SRXISEL_SETVAL(x, v)       do { (x) = (((x) & ~0x3) | ((v) << 0)); } while(0)
  #define PIC32_SPI_CON_SRXISEL_GET(x)             (((x) >> 0) & 0x3)
    #define PIC32_SPI_CON_SRXISEL_EMPTY              0x00000000
    #define PIC32_SPI_CON_SRXISEL_NOT_EMPTY          0x00000001
    #define PIC32_SPI_CON_SRXISEL_HALF_FULL          0x00000002
    #define PIC32_SPI_CON_SRXISEL_FULL               0x00000003
  #define PIC32_SPI_CON_STXISEL(v)                 ((PIC32_SPI_CON_STXISEL_##v) << 2)
  #define PIC32_SPI_CON_STXISEL_SET(x, v)          do { (x) = (((x) & ~0xc) | ((PIC32_SPI_CON_STXISEL_##v) << 2)); } while(0)
  #define PIC32_SPI_CON_STXISEL_SETVAL(x, v)       do { (x) = (((x) & ~0xc) | ((v) << 2)); } while(0)
  #define PIC32_SPI_CON_STXISEL_GET(x)             (((x) >> 2) & 0x3)
    #define PIC32_SPI_CON_STXISEL_COMPLETE           0x00000000
    #define PIC32_SPI_CON_STXISEL_EMPTY              0x00000001
    #define PIC32_SPI_CON_STXISEL_HALF_EMPTY         0x00000002
    #define PIC32_SPI_CON_STXISEL_NOT_FULL           0x00000003
  #define PIC32_SPI_CON_DISSDI                     0x00000010
  #define PIC32_SPI_CON_MSTEN                      0x00000020
  #define PIC32_SPI_CON_CKP                        0x00000040
  #define PIC32_SPI_CON_SSEN                       0x00000080
  #define PIC32_SPI_CON_CKE                        0x00000100
  #define PIC32_SPI_CON_SMP                        0x00000200
  #define PIC32_SPI_CON_MODE(v)                    ((v) << 10)
  #define PIC32_SPI_CON_MODE_SET(x, v)             do { (x) = (((x) & ~0xc00) | ((v) << 10)); } while(0)
  #define PIC32_SPI_CON_MODE_GET(x)                (((x) >> 10) & 0x3)
  #define PIC32_SPI_CON_DISSDO                     0x00001000
  #define PIC32_SPI_CON_SIDL                       0x00002000
  #define PIC32_SPI_CON_ON                         0x00008000
  #define PIC32_SPI_CON_ENHBUF                     0x00010000
  #define PIC32_SPI_CON_SPIFE                      0x00020000
  #define PIC32_SPI_CON_MCLKSEL                    0x00800000
  #define PIC32_SPI_CON_FRMCNT(v)                  ((PIC32_SPI_CON_FRMCNT_##v) << 24)
  #define PIC32_SPI_CON_FRMCNT_SET(x, v)           do { (x) = (((x) & ~0x7000000) | ((PIC32_SPI_CON_FRMCNT_##v) << 24)); } while(0)
  #define PIC32_SPI_CON_FRMCNT_SETVAL(x, v)        do { (x) = (((x) & ~0x7000000) | ((v) << 24)); } while(0)
  #define PIC32_SPI_CON_FRMCNT_GET(x)              (((x) >> 24) & 0x7)
    #define PIC31_SPI_CON_FRMCNT_EVERY1              0x00000000
    #define PIC32_SPI_CON_FRMCNT_EVERY2              0x00000001
    #define PIC32_SPI_CON_FRMCNT_EVERY4              0x00000002
    #define PIC32_SPI_CON_FRMCNT_EVERY8              0x00000003
    #define PIC32_SPI_CON_FRMCNT_EVERY16             0x00000004
    #define PIC32_SPI_CON_FRMCNT_EVERY32             0x00000005
  #define PIC32_SPI_CON_FRMSYPW                    0x08000000
  #define PIC32_SPI_CON_MSSEN                      0x10000000
  #define PIC32_SPI_CON_FRMPOL                     0x20000000
  #define PIC32_SPI_CON_FRMSYNC                    0x40000000
  #define PIC32_SPI_CON_FRMEN                      0x80000000

#define PIC32_SPI_CON_CLR_ADDR                       0x00000004
#define PIC32_SPI_CON_CLR_MASK                       0xff83bfff
  #define PIC32_SPI_CON_CLR_SRXISEL(v)             ((PIC32_SPI_CON_CLR_SRXISEL_##v) << 0)
  #define PIC32_SPI_CON_CLR_SRXISEL_SET(x, v)      do { (x) = (((x) & ~0x3) | ((PIC32_SPI_CON_CLR_SRXISEL_##v) << 0)); } while(0)
  #define PIC32_SPI_CON_CLR_SRXISEL_SETVAL(x, v)   do { (x) = (((x) & ~0x3) | ((v) << 0)); } while(0)
  #define PIC32_SPI_CON_CLR_SRXISEL_GET(x)         (((x) >> 0) & 0x3)
    #define PIC32_SPI_CON_CLR_SRXISEL_EMPTY          0x00000000
    #define PIC32_SPI_CON_CLR_SRXISEL_NOT_EMPTY      0x00000001
    #define PIC32_SPI_CON_CLR_SRXISEL_HALF_FULL      0x00000002
    #define PIC32_SPI_CON_CLR_SRXISEL_FULL           0x00000003
  #define PIC32_SPI_CON_CLR_STXISEL(v)             ((PIC32_SPI_CON_CLR_STXISEL_##v) << 2)
  #define PIC32_SPI_CON_CLR_STXISEL_SET(x, v)      do { (x) = (((x) & ~0xc) | ((PIC32_SPI_CON_CLR_STXISEL_##v) << 2)); } while(0)
  #define PIC32_SPI_CON_CLR_STXISEL_SETVAL(x, v)   do { (x) = (((x) & ~0xc) | ((v) << 2)); } while(0)
  #define PIC32_SPI_CON_CLR_STXISEL_GET(x)         (((x) >> 2) & 0x3)
    #define PIC32_SPI_CON_CLR_STXISEL_COMPLETE       0x00000000
    #define PIC32_SPI_CON_CLR_STXISEL_EMPTY          0x00000001
    #define PIC32_SPI_CON_CLR_STXISEL_HALF_EMPTY     0x00000002
    #define PIC32_SPI_CON_CLR_STXISEL_NOT_FULL       0x00000003
  #define PIC32_SPI_CON_CLR_DISSDI                 0x00000010
  #define PIC32_SPI_CON_CLR_MSTEN                  0x00000020
  #define PIC32_SPI_CON_CLR_CKP                    0x00000040
  #define PIC32_SPI_CON_CLR_SSEN                   0x00000080
  #define PIC32_SPI_CON_CLR_CKE                    0x00000100
  #define PIC32_SPI_CON_CLR_SMP                    0x00000200
  #define PIC32_SPI_CON_CLR_MODE(v)                ((v) << 10)
  #define PIC32_SPI_CON_CLR_MODE_SET(x, v)         do { (x) = (((x) & ~0xc00) | ((v) << 10)); } while(0)
  #define PIC32_SPI_CON_CLR_MODE_GET(x)            (((x) >> 10) & 0x3)
  #define PIC32_SPI_CON_CLR_DISSDO                 0x00001000
  #define PIC32_SPI_CON_CLR_SIDL                   0x00002000
  #define PIC32_SPI_CON_CLR_ON                     0x00008000
  #define PIC32_SPI_CON_CLR_ENHBUF                 0x00010000
  #define PIC32_SPI_CON_CLR_SPIFE                  0x00020000
  #define PIC32_SPI_CON_CLR_MCLKSEL                0x00800000
  #define PIC32_SPI_CON_CLR_FRMCNT(v)              ((PIC32_SPI_CON_CLR_FRMCNT_##v) << 24)
  #define PIC32_SPI_CON_CLR_FRMCNT_SET(x, v)       do { (x) = (((x) & ~0x7000000) | ((PIC32_SPI_CON_CLR_FRMCNT_##v) << 24)); } while(0)
  #define PIC32_SPI_CON_CLR_FRMCNT_SETVAL(x, v)    do { (x) = (((x) & ~0x7000000) | ((v) << 24)); } while(0)
  #define PIC32_SPI_CON_CLR_FRMCNT_GET(x)          (((x) >> 24) & 0x7)
    #define PIC32_SPI_CON_CLR_FRMCNT_EVERY1          0x00000000
    #define PIC32_SPI_CON_CLR_FRMCNT_EVERY2          0x00000001
    #define PIC32_SPI_CON_CLR_FRMCNT_EVERY4          0x00000002
    #define PIC32_SPI_CON_CLR_FRMCNT_EVERY8          0x00000003
    #define PIC32_SPI_CON_CLR_FRMCNT_EVERY16         0x00000004
    #define PIC32_SPI_CON_CLR_FRMCNT_EVERY32         0x00000005
  #define PIC32_SPI_CON_CLR_FRMSYPW                0x08000000
  #define PIC32_SPI_CON_CLR_MSSEN                  0x10000000
  #define PIC32_SPI_CON_CLR_FRMPOL                 0x20000000
  #define PIC32_SPI_CON_CLR_FRMSYNC                0x40000000
  #define PIC32_SPI_CON_CLR_FRMEN                  0x80000000

#define PIC32_SPI_CON_SET_ADDR                       0x00000008
#define PIC32_SPI_CON_SET_MASK                       0xff83bfff
  #define PIC32_SPI_CON_SET_SRXISEL(v)             ((PIC32_SPI_CON_SET_SRXISEL_##v) << 0)
  #define PIC32_SPI_CON_SET_SRXISEL_SET(x, v)      do { (x) = (((x) & ~0x3) | ((PIC32_SPI_CON_SET_SRXISEL_##v) << 0)); } while(0)
  #define PIC32_SPI_CON_SET_SRXISEL_SETVAL(x, v)   do { (x) = (((x) & ~0x3) | ((v) << 0)); } while(0)
  #define PIC32_SPI_CON_SET_SRXISEL_GET(x)         (((x) >> 0) & 0x3)
    #define PIC32_SPI_CON_SET_SRXISEL_EMPTY          0x00000000
    #define PIC32_SPI_CON_SET_SRXISEL_NOT_EMPTY      0x00000001
    #define PIC32_SPI_CON_SET_SRXISEL_HALF_FULL      0x00000002
    #define PIC32_SPI_CON_SET_SRXISEL_FULL           0x00000003
  #define PIC32_SPI_CON_SET_STXISEL(v)             ((PIC32_SPI_CON_SET_STXISEL_##v) << 2)
  #define PIC32_SPI_CON_SET_STXISEL_SET(x, v)      do { (x) = (((x) & ~0xc) | ((PIC32_SPI_CON_SET_STXISEL_##v) << 2)); } while(0)
  #define PIC32_SPI_CON_SET_STXISEL_SETVAL(x, v)   do { (x) = (((x) & ~0xc) | ((v) << 2)); } while(0)
  #define PIC32_SPI_CON_SET_STXISEL_GET(x)         (((x) >> 2) & 0x3)
    #define PIC32_SPI_CON_SET_STXISEL_COMPLETE       0x00000000
    #define PIC32_SPI_CON_SET_STXISEL_EMPTY          0x00000001
    #define PIC32_SPI_CON_SET_STXISEL_HALF_EMPTY     0x00000002
    #define PIC32_SPI_CON_SET_STXISEL_NOT_FULL       0x00000003
  #define PIC32_SPI_CON_SET_DISSDI                 0x00000010
  #define PIC32_SPI_CON_SET_MSTEN                  0x00000020
  #define PIC32_SPI_CON_SET_CKP                    0x00000040
  #define PIC32_SPI_CON_SET_SSEN                   0x00000080
  #define PIC32_SPI_CON_SET_CKE                    0x00000100
  #define PIC32_SPI_CON_SET_SMP                    0x00000200
  #define PIC32_SPI_CON_SET_MODE(v)                ((v) << 10)
  #define PIC32_SPI_CON_SET_MODE_SET(x, v)         do { (x) = (((x) & ~0xc00) | ((v) << 10)); } while(0)
  #define PIC32_SPI_CON_SET_MODE_GET(x)            (((x) >> 10) & 0x3)
  #define PIC32_SPI_CON_SET_DISSDO                 0x00001000
  #define PIC32_SPI_CON_SET_SIDL                   0x00002000
  #define PIC32_SPI_CON_SET_ON                     0x00008000
  #define PIC32_SPI_CON_SET_ENHBUF                 0x00010000
  #define PIC32_SPI_CON_SET_SPIFE                  0x00020000
  #define PIC32_SPI_CON_SET_MCLKSEL                0x00800000
  #define PIC32_SPI_CON_SET_FRMCNT(v)              ((PIC32_SPI_CON_SET_FRMCNT_##v) << 24)
  #define PIC32_SPI_CON_SET_FRMCNT_SET(x, v)       do { (x) = (((x) & ~0x7000000) | ((PIC32_SPI_CON_SET_FRMCNT_##v) << 24)); } while(0)
  #define PIC32_SPI_CON_SET_FRMCNT_SETVAL(x, v)    do { (x) = (((x) & ~0x7000000) | ((v) << 24)); } while(0)
  #define PIC32_SPI_CON_SET_FRMCNT_GET(x)          (((x) >> 24) & 0x7)
    #define PIC32_SPI_CON_SET_FRMCNT_EVERY1          0x00000000
    #define PIC32_SPI_CON_SET_FRMCNT_EVERY2          0x00000001
    #define PIC32_SPI_CON_SET_FRMCNT_EVERY4          0x00000002
    #define PIC32_SPI_CON_SET_FRMCNT_EVERY8          0x00000003
    #define PIC32_SPI_CON_SET_FRMCNT_EVERY16         0x00000004
    #define PIC32_SPI_CON_SET_FRMCNT_EVERY32         0x00000005
  #define PIC32_SPI_CON_SET_FRMSYPW                0x08000000
  #define PIC32_SPI_CON_SET_MSSEN                  0x10000000
  #define PIC32_SPI_CON_SET_FRMPOL                 0x20000000
  #define PIC32_SPI_CON_SET_FRMSYNC                0x40000000
  #define PIC32_SPI_CON_SET_FRMEN                  0x80000000

#define PIC32_SPI_STATUS_ADDR                        0x00000010
#define PIC32_SPI_STATUS_MASK                        0x1f1f19eb
  #define PIC32_SPI_STATUS_RBF                     0x00000001
  #define PIC32_SPI_STATUS_TBF                     0x00000002
  #define PIC32_SPI_STATUS_TBE                     0x00000008
  #define PIC32_SPI_STATUS_RBE                     0x00000020
  #define PIC32_SPI_STATUS_ROV                     0x00000040
  #define PIC32_SPI_STATUS_SRMT                    0x00000080
  #define PIC32_SPI_STATUS_TUR                     0x00000100
  #define PIC32_SPI_STATUS_BUSY                    0x00000800
  #define PIC32_SPI_STATUS_FRMERR                  0x00001000
  #define PIC32_SPI_STATUS_TXBUFELM(v)             ((v) << 16)
  #define PIC32_SPI_STATUS_TXBUFELM_SET(x, v)      do { (x) = (((x) & ~0x1f0000) | ((v) << 16)); } while(0)
  #define PIC32_SPI_STATUS_TXBUFELM_GET(x)         (((x) >> 16) & 0x1f)
  #define PIC32_SPI_STATUS_RXBUFELM(v)             ((v) << 24)
  #define PIC32_SPI_STATUS_RXBUFELM_SET(x, v)      do { (x) = (((x) & ~0x1f000000) | ((v) << 24)); } while(0)
  #define PIC32_SPI_STATUS_RXBUFELM_GET(x)         (((x) >> 24) & 0x1f)

#define PIC32_SPI_STATUS_CLR_ADDR                    0x00000014
#define PIC32_SPI_STATUS_CLR_MASK                    0x1f1f19eb
  #define PIC32_SPI_STATUS_CLR_RBF                 0x00000001
  #define PIC32_SPI_STATUS_CLR_TBF                 0x00000002
  #define PIC32_SPI_STATUS_CLR_TBE                 0x00000008
  #define PIC32_SPI_STATUS_CLR_RBE                 0x00000020
  #define PIC32_SPI_STATUS_CLR_ROV                 0x00000040
  #define PIC32_SPI_STATUS_CLR_SRMT                0x00000080
  #define PIC32_SPI_STATUS_CLR_TUR                 0x00000100
  #define PIC32_SPI_STATUS_CLR_BUSY                0x00000800
  #define PIC32_SPI_STATUS_CLR_FRMERR              0x00001000
  #define PIC32_SPI_STATUS_CLR_TXBUFELM(v)         ((v) << 16)
  #define PIC32_SPI_STATUS_CLR_TXBUFELM_SET(x, v)  do { (x) = (((x) & ~0x1f0000) | ((v) << 16)); } while(0)
  #define PIC32_SPI_STATUS_CLR_TXBUFELM_GET(x)     (((x) >> 16) & 0x1f)
  #define PIC32_SPI_STATUS_CLR_RXBUFELM(v)         ((v) << 24)
  #define PIC32_SPI_STATUS_CLR_RXBUFELM_SET(x, v)  do { (x) = (((x) & ~0x1f000000) | ((v) << 24)); } while(0)
  #define PIC32_SPI_STATUS_CLR_RXBUFELM_GET(x)     (((x) >> 24) & 0x1f)

#define PIC32_SPI_STATUS_SET_ADDR                    0x00000018
#define PIC32_SPI_STATUS_SET_MASK                    0x1f1f19eb
  #define PIC32_SPI_STATUS_SET_RBF                 0x00000001
  #define PIC32_SPI_STATUS_SET_TBF                 0x00000002
  #define PIC32_SPI_STATUS_SET_TBE                 0x00000008
  #define PIC32_SPI_STATUS_SET_RBE                 0x00000020
  #define PIC32_SPI_STATUS_SET_ROV                 0x00000040
  #define PIC32_SPI_STATUS_SET_SRMT                0x00000080
  #define PIC32_SPI_STATUS_SET_TUR                 0x00000100
  #define PIC32_SPI_STATUS_SET_BUSY                0x00000800
  #define PIC32_SPI_STATUS_SET_FRMERR              0x00001000
  #define PIC32_SPI_STATUS_SET_TXBUFELM(v)         ((v) << 16)
  #define PIC32_SPI_STATUS_SET_TXBUFELM_SET(x, v)  do { (x) = (((x) & ~0x1f0000) | ((v) << 16)); } while(0)
  #define PIC32_SPI_STATUS_SET_TXBUFELM_GET(x)     (((x) >> 16) & 0x1f)
  #define PIC32_SPI_STATUS_SET_RXBUFELM(v)         ((v) << 24)
  #define PIC32_SPI_STATUS_SET_RXBUFELM_SET(x, v)  do { (x) = (((x) & ~0x1f000000) | ((v) << 24)); } while(0)
  #define PIC32_SPI_STATUS_SET_RXBUFELM_GET(x)     (((x) >> 24) & 0x1f)

#define PIC32_SPI_BUF_ADDR                           0x00000020
#define PIC32_SPI_BUF_MASK                           0xffffffff
  #define PIC32_SPI_BUF_DATA(v)                    ((v) << 0)
  #define PIC32_SPI_BUF_DATA_SET(x, v)             do { (x) = (((x) & ~0xffffffff) | ((v) << 0)); } while(0)
  #define PIC32_SPI_BUF_DATA_GET(x)                (((x) >> 0) & 0xffffffff)

#define PIC32_SPI_BRG_ADDR                           0x00000030
#define PIC32_SPI_BRG_MASK                           0x000001ff
  #define PIC32_SPI_BRG_VAL(v)                     ((v) << 0)
  #define PIC32_SPI_BRG_VAL_SET(x, v)              do { (x) = (((x) & ~0x1ff) | ((v) << 0)); } while(0)
  #define PIC32_SPI_BRG_VAL_GET(x)                 (((x) >> 0) & 0x1ff)

#define PIC32_SPI_CON2_ADDR                          0x00000040
#define PIC32_SPI_CON2_MASK                          0x00009f8b
  #define PIC32_SPI_CON2_AUDMOD(v)                 ((PIC32_SPI_CON2_AUDMOD_##v) << 0)
  #define PIC32_SPI_CON2_AUDMOD_SET(x, v)          do { (x) = (((x) & ~0x3) | ((PIC32_SPI_CON2_AUDMOD_##v) << 0)); } while(0)
  #define PIC32_SPI_CON2_AUDMOD_SETVAL(x, v)       do { (x) = (((x) & ~0x3) | ((v) << 0)); } while(0)
  #define PIC32_SPI_CON2_AUDMOD_GET(x)             (((x) >> 0) & 0x3)
    #define PIC32_SPI_CON2_AUDMOD_I2S                0x00000000
    #define PIC32_SPI_CON2_AUDMOD_LEFTJUSTIFIED      0x00000001
    #define PIC32_SPI_CON2_AUDMOD_RIGHTJUSTIFIED     0x00000002
    #define PIC32_SPI_CON2_AUDMOD_PCMDSP             0x00000003
  #define PIC32_SPI_CON2_AUDMONO                   0x00000008
  #define PIC32_SPI_CON2_AUDEN                     0x00000080
  #define PIC32_SPI_CON2_IGNTUR                    0x00000100
  #define PIC32_SPI_CON2_IGNROV                    0x00000200
  #define PIC32_SPI_CON2_SPITUREN                  0x00000400
  #define PIC32_SPI_CON2_SPIROVEN                  0x00000800
  #define PIC32_SPI_CON2_FRMERREN                  0x00001000
  #define PIC32_SPI_CON2_SPISGNEXT                 0x00008000

#endif

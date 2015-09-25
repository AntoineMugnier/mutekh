/***************************************
* Auto generated by BFGen, do not edit *
***************************************/

/*
   bfgen -o cdefs -O arch/pic32/include/arch/pic32_dma.h cdefs_use_reg_mask=1  \
     cdefs_use_field_setval=1 -I /opt/bfgen/defs/pic32/pic32_dma.bf
*/

#ifndef _PIC32_DMA_BFGEN_DEFS_
#define _PIC32_DMA_BFGEN_DEFS_

#define PIC32_DMA_CTRL_ADDR                          0x00000000
#define PIC32_DMA_CTRL_MASK                          0x00009800
  #define PIC32_DMA_CTRL_BUSY                      0x00000800
  #define PIC32_DMA_CTRL_SUSPEND                   0x00001000
  #define PIC32_DMA_CTRL_ON                        0x00008000

#define PIC32_DMA_STATUS_ADDR                        0x00000010
#define PIC32_DMA_STATUS_MASK                        0x0000000f
  #define PIC32_DMA_STATUS_CH(v)                   ((v) << 0)
  #define PIC32_DMA_STATUS_CH_SET(x, v)            do { (x) = (((x) & ~0x7) | ((v) << 0)); } while(0)
  #define PIC32_DMA_STATUS_CH_GET(x)               (((x) >> 0) & 0x7)
  #define PIC32_DMA_STATUS_RDWR                    0x00000008

#define PIC32_DMA_ADDR_ADDR                          0x00000020
#define PIC32_DMA_ADDR_MASK                          0xffffffff
  #define PIC32_DMA_ADDR_V(v)                      ((v) << 0)
  #define PIC32_DMA_ADDR_V_SET(x, v)               do { (x) = (((x) & ~0xffffffff) | ((v) << 0)); } while(0)
  #define PIC32_DMA_ADDR_V_GET(x)                  (((x) >> 0) & 0xffffffff)

#define PIC32_DMA_DCRCCTRL_ADDR                      0x00000030
#define PIC32_DMA_DCRCCTRL_MASK                      0x39001fe7
  #define PIC32_DMA_DCRCCTRL_CHAN(v)               ((v) << 0)
  #define PIC32_DMA_DCRCCTRL_CHAN_SET(x, v)        do { (x) = (((x) & ~0x7) | ((v) << 0)); } while(0)
  #define PIC32_DMA_DCRCCTRL_CHAN_GET(x)           (((x) >> 0) & 0x7)
  #define PIC32_DMA_DCRCCTRL_TYPE                  0x00000020
  #define PIC32_DMA_DCRCCTRL_APP                   0x00000040
  #define PIC32_DMA_DCRCCTRL_EN                    0x00000080
  #define PIC32_DMA_DCRCCTRL_LEN(v)                ((v) << 8)
  #define PIC32_DMA_DCRCCTRL_LEN_SET(x, v)         do { (x) = (((x) & ~0x1f00) | ((v) << 8)); } while(0)
  #define PIC32_DMA_DCRCCTRL_LEN_GET(x)            (((x) >> 8) & 0x1f)
  #define PIC32_DMA_DCRCCTRL_BITO                  0x01000000
  #define PIC32_DMA_DCRCCTRL_WBO                   0x08000000
  #define PIC32_DMA_DCRCCTRL_BYTO(v)               ((v) << 28)
  #define PIC32_DMA_DCRCCTRL_BYTO_SET(x, v)        do { (x) = (((x) & ~0x30000000) | ((v) << 28)); } while(0)
  #define PIC32_DMA_DCRCCTRL_BYTO_GET(x)           (((x) >> 28) & 0x3)

#define PIC32_DMA_DCRCDATA_ADDR                      0x00000040
#define PIC32_DMA_DCRCDATA_MASK                      0xffffffff
  #define PIC32_DMA_DCRCDATA_V(v)                  ((v) << 0)
  #define PIC32_DMA_DCRCDATA_V_SET(x, v)           do { (x) = (((x) & ~0xffffffff) | ((v) << 0)); } while(0)
  #define PIC32_DMA_DCRCDATA_V_GET(x)              (((x) >> 0) & 0xffffffff)

#define PIC32_DMA_DCRCXOR_ADDR                       0x00000050
#define PIC32_DMA_DCRCXOR_MASK                       0xffffffff
  #define PIC32_DMA_DCRCXOR_V(v)                   ((v) << 0)
  #define PIC32_DMA_DCRCXOR_V_SET(x, v)            do { (x) = (((x) & ~0xffffffff) | ((v) << 0)); } while(0)
  #define PIC32_DMA_DCRCXOR_V_GET(x)               (((x) >> 0) & 0xffffffff)

#define PIC32_DMA_DCHCTRL_ADDR(ridx)                 (0x00000060 + (ridx) * 192)
#define PIC32_DMA_DCHCTRL_COUNT                      8
#define PIC32_DMA_DCHCTRL_MASK                       0xff00a9f7
  #define PIC32_DMA_DCHCTRL_PRI(v)                 ((v) << 0)
  #define PIC32_DMA_DCHCTRL_PRI_SET(x, v)          do { (x) = (((x) & ~0x3) | ((v) << 0)); } while(0)
  #define PIC32_DMA_DCHCTRL_PRI_GET(x)             (((x) >> 0) & 0x3)
  #define PIC32_DMA_DCHCTRL_EDET                   0x00000004
  #define PIC32_DMA_DCHCTRL_AEN                    0x00000010
  #define PIC32_DMA_DCHCTRL_CHN                    0x00000020
  #define PIC32_DMA_DCHCTRL_AED                    0x00000040
  #define PIC32_DMA_DCHCTRL_EN                     0x00000080
  #define PIC32_DMA_DCHCTRL_CHNS                   0x00000100
  #define PIC32_DMA_DCHCTRL_PATLEN                 0x00000800
  #define PIC32_DMA_DCHCTRL_PIGNEN                 0x00002000
  #define PIC32_DMA_DCHCTRL_BUSY                   0x00008000
  #define PIC32_DMA_DCHCTRL_PIGN(v)                ((v) << 24)
  #define PIC32_DMA_DCHCTRL_PIGN_SET(x, v)         do { (x) = (((x) & ~0xff000000) | ((v) << 24)); } while(0)
  #define PIC32_DMA_DCHCTRL_PIGN_GET(x)            (((x) >> 24) & 0xff)

#define PIC32_DMA_DCHCTRL_CLR_ADDR(ridx)             (0x00000064 + (ridx) * 192)
#define PIC32_DMA_DCHCTRL_CLR_COUNT                  8
#define PIC32_DMA_DCHCTRL_CLR_MASK                   0xff00a9f7
  #define PIC32_DMA_DCHCTRL_CLR_PRI(v)             ((v) << 0)
  #define PIC32_DMA_DCHCTRL_CLR_PRI_SET(x, v)      do { (x) = (((x) & ~0x3) | ((v) << 0)); } while(0)
  #define PIC32_DMA_DCHCTRL_CLR_PRI_GET(x)         (((x) >> 0) & 0x3)
  #define PIC32_DMA_DCHCTRL_CLR_EDET               0x00000004
  #define PIC32_DMA_DCHCTRL_CLR_AEN                0x00000010
  #define PIC32_DMA_DCHCTRL_CLR_CHN                0x00000020
  #define PIC32_DMA_DCHCTRL_CLR_AED                0x00000040
  #define PIC32_DMA_DCHCTRL_CLR_EN                 0x00000080
  #define PIC32_DMA_DCHCTRL_CLR_CHNS               0x00000100
  #define PIC32_DMA_DCHCTRL_CLR_PATLEN             0x00000800
  #define PIC32_DMA_DCHCTRL_CLR_PIGNEN             0x00002000
  #define PIC32_DMA_DCHCTRL_CLR_BUSY               0x00008000
  #define PIC32_DMA_DCHCTRL_CLR_PIGN(v)            ((v) << 24)
  #define PIC32_DMA_DCHCTRL_CLR_PIGN_SET(x, v)     do { (x) = (((x) & ~0xff000000) | ((v) << 24)); } while(0)
  #define PIC32_DMA_DCHCTRL_CLR_PIGN_GET(x)        (((x) >> 24) & 0xff)

#define PIC32_DMA_DCHCTRL_SET_ADDR(ridx)             (0x00000068 + (ridx) * 192)
#define PIC32_DMA_DCHCTRL_SET_COUNT                  8
#define PIC32_DMA_DCHCTRL_SET_MASK                   0xff00a9f7
  #define PIC32_DMA_DCHCTRL_SET_PRI(v)             ((v) << 0)
  #define PIC32_DMA_DCHCTRL_SET_PRI_SET(x, v)      do { (x) = (((x) & ~0x3) | ((v) << 0)); } while(0)
  #define PIC32_DMA_DCHCTRL_SET_PRI_GET(x)         (((x) >> 0) & 0x3)
  #define PIC32_DMA_DCHCTRL_SET_EDET               0x00000004
  #define PIC32_DMA_DCHCTRL_SET_AEN                0x00000010
  #define PIC32_DMA_DCHCTRL_SET_CHN                0x00000020
  #define PIC32_DMA_DCHCTRL_SET_AED                0x00000040
  #define PIC32_DMA_DCHCTRL_SET_EN                 0x00000080
  #define PIC32_DMA_DCHCTRL_SET_CHNS               0x00000100
  #define PIC32_DMA_DCHCTRL_SET_PATLEN             0x00000800
  #define PIC32_DMA_DCHCTRL_SET_PIGNEN             0x00002000
  #define PIC32_DMA_DCHCTRL_SET_BUSY               0x00008000
  #define PIC32_DMA_DCHCTRL_SET_PIGN(v)            ((v) << 24)
  #define PIC32_DMA_DCHCTRL_SET_PIGN_SET(x, v)     do { (x) = (((x) & ~0xff000000) | ((v) << 24)); } while(0)
  #define PIC32_DMA_DCHCTRL_SET_PIGN_GET(x)        (((x) >> 24) & 0xff)

#define PIC32_DMA_DCHECTRL_ADDR(ridx)                (0x00000070 + (ridx) * 192)
#define PIC32_DMA_DCHECTRL_COUNT                     8
#define PIC32_DMA_DCHECTRL_MASK                      0x00fffff8
  #define PIC32_DMA_DCHECTRL_AIRQEN                0x00000008
  #define PIC32_DMA_DCHECTRL_SIRQEN                0x00000010
  #define PIC32_DMA_DCHECTRL_PATEN                 0x00000020
  #define PIC32_DMA_DCHECTRL_CABORT                0x00000040
  #define PIC32_DMA_DCHECTRL_CFORCE                0x00000080
  #define PIC32_DMA_DCHECTRL_SIRQ(v)               ((v) << 8)
  #define PIC32_DMA_DCHECTRL_SIRQ_SET(x, v)        do { (x) = (((x) & ~0xff00) | ((v) << 8)); } while(0)
  #define PIC32_DMA_DCHECTRL_SIRQ_GET(x)           (((x) >> 8) & 0xff)
  #define PIC32_DMA_DCHECTRL_AIRQ(v)               ((v) << 16)
  #define PIC32_DMA_DCHECTRL_AIRQ_SET(x, v)        do { (x) = (((x) & ~0xff0000) | ((v) << 16)); } while(0)
  #define PIC32_DMA_DCHECTRL_AIRQ_GET(x)           (((x) >> 16) & 0xff)

#define PIC32_DMA_DCHINT_ADDR(ridx)                  (0x00000080 + (ridx) * 192)
#define PIC32_DMA_DCHINT_COUNT                       8
#define PIC32_DMA_DCHINT_MASK                        0x00ff00ff
  #define PIC32_DMA_DCHINT_CHERIF                  0x00000001
  #define PIC32_DMA_DCHINT_CHTAIF                  0x00000002
  #define PIC32_DMA_DCHINT_CHCCIF                  0x00000004
  #define PIC32_DMA_DCHINT_CHBCIF                  0x00000008
  #define PIC32_DMA_DCHINT_CHDHIF                  0x00000010
  #define PIC32_DMA_DCHINT_CHDDIF                  0x00000020
  #define PIC32_DMA_DCHINT_CHSHIF                  0x00000040
  #define PIC32_DMA_DCHINT_CHSDIF                  0x00000080
  #define PIC32_DMA_DCHINT_CHERIE                  0x00010000
  #define PIC32_DMA_DCHINT_CHTAIE                  0x00020000
  #define PIC32_DMA_DCHINT_CHCCIE                  0x00040000
  #define PIC32_DMA_DCHINT_CHBCIE                  0x00080000
  #define PIC32_DMA_DCHINT_CHDHIE                  0x00100000
  #define PIC32_DMA_DCHINT_CHDDIE                  0x00200000
  #define PIC32_DMA_DCHINT_CHSHIE                  0x00400000
  #define PIC32_DMA_DCHINT_CHSDIE                  0x00800000

#define PIC32_DMA_DCHSSA_ADDR(ridx)                  (0x00000090 + (ridx) * 192)
#define PIC32_DMA_DCHSSA_COUNT                       8
#define PIC32_DMA_DCHSSA_MASK                        0xffffffff
  #define PIC32_DMA_DCHSSA_V(v)                    ((v) << 0)
  #define PIC32_DMA_DCHSSA_V_SET(x, v)             do { (x) = (((x) & ~0xffffffff) | ((v) << 0)); } while(0)
  #define PIC32_DMA_DCHSSA_V_GET(x)                (((x) >> 0) & 0xffffffff)

#define PIC32_DMA_DCHDSA_ADDR(ridx)                  (0x000000a0 + (ridx) * 192)
#define PIC32_DMA_DCHDSA_COUNT                       8
#define PIC32_DMA_DCHDSA_MASK                        0xffffffff
  #define PIC32_DMA_DCHDSA_V(v)                    ((v) << 0)
  #define PIC32_DMA_DCHDSA_V_SET(x, v)             do { (x) = (((x) & ~0xffffffff) | ((v) << 0)); } while(0)
  #define PIC32_DMA_DCHDSA_V_GET(x)                (((x) >> 0) & 0xffffffff)

#define PIC32_DMA_DCHSSIZ_ADDR(ridx)                 (0x000000b0 + (ridx) * 192)
#define PIC32_DMA_DCHSSIZ_COUNT                      8
#define PIC32_DMA_DCHSSIZ_MASK                       0x0000ffff
  #define PIC32_DMA_DCHSSIZ_V(v)                   ((v) << 0)
  #define PIC32_DMA_DCHSSIZ_V_SET(x, v)            do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define PIC32_DMA_DCHSSIZ_V_GET(x)               (((x) >> 0) & 0xffff)

#define PIC32_DMA_DCHDSIZ_ADDR(ridx)                 (0x000000c0 + (ridx) * 192)
#define PIC32_DMA_DCHDSIZ_COUNT                      8
#define PIC32_DMA_DCHDSIZ_MASK                       0x0000ffff
  #define PIC32_DMA_DCHDSIZ_V(v)                   ((v) << 0)
  #define PIC32_DMA_DCHDSIZ_V_SET(x, v)            do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define PIC32_DMA_DCHDSIZ_V_GET(x)               (((x) >> 0) & 0xffff)

#define PIC32_DMA_DCHSPTR_ADDR(ridx)                 (0x000000d0 + (ridx) * 192)
#define PIC32_DMA_DCHSPTR_COUNT                      8
#define PIC32_DMA_DCHSPTR_MASK                       0x0000ffff
  #define PIC32_DMA_DCHSPTR_V(v)                   ((v) << 0)
  #define PIC32_DMA_DCHSPTR_V_SET(x, v)            do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define PIC32_DMA_DCHSPTR_V_GET(x)               (((x) >> 0) & 0xffff)

#define PIC32_DMA_DCHDDPTR_ADDR(ridx)                (0x000000e0 + (ridx) * 192)
#define PIC32_DMA_DCHDDPTR_COUNT                     8
#define PIC32_DMA_DCHDDPTR_MASK                      0x0000ffff
  #define PIC32_DMA_DCHDDPTR_V(v)                  ((v) << 0)
  #define PIC32_DMA_DCHDDPTR_V_SET(x, v)           do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define PIC32_DMA_DCHDDPTR_V_GET(x)              (((x) >> 0) & 0xffff)

#define PIC32_DMA_DCHCSIZ_ADDR(ridx)                 (0x000000f0 + (ridx) * 192)
#define PIC32_DMA_DCHCSIZ_COUNT                      8
#define PIC32_DMA_DCHCSIZ_MASK                       0x0000ffff
  #define PIC32_DMA_DCHCSIZ_V(v)                   ((v) << 0)
  #define PIC32_DMA_DCHCSIZ_V_SET(x, v)            do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define PIC32_DMA_DCHCSIZ_V_GET(x)               (((x) >> 0) & 0xffff)

#define PIC32_DMA_DCHCPTR_ADDR(ridx)                 (0x00000100 + (ridx) * 192)
#define PIC32_DMA_DCHCPTR_COUNT                      8
#define PIC32_DMA_DCHCPTR_MASK                       0x0000ffff
  #define PIC32_DMA_DCHCPTR_V(v)                   ((v) << 0)
  #define PIC32_DMA_DCHCPTR_V_SET(x, v)            do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define PIC32_DMA_DCHCPTR_V_GET(x)               (((x) >> 0) & 0xffff)

#define PIC32_DMA_DCHDAT_ADDR(ridx)                  (0x00000110 + (ridx) * 192)
#define PIC32_DMA_DCHDAT_COUNT                       8
#define PIC32_DMA_DCHDAT_MASK                        0x0000ffff
  #define PIC32_DMA_DCHDAT_V(v)                    ((v) << 0)
  #define PIC32_DMA_DCHDAT_V_SET(x, v)             do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define PIC32_DMA_DCHDAT_V_GET(x)                (((x) >> 0) & 0xffff)

#endif


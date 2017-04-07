#ifndef PIC32_FUSES_H_
#define PIC32_FUSES_H_

#include <hexo/types.h>
#include <arch/pic32/config.h>

#define _PIC32_ATTR_FUSE_FIELD(name_, attr_, mask_, value_)             \
  const uint32_t pic32_fuse_##name_                                     \
  __attribute__((section(".pic32_fuses."#name_)))                       \
       attr_                                                            \
       = ~(mask_) | (value_)

#define _PIC32_FUSE_FIELD(name_, mask_, value_)         \
  _PIC32_ATTR_FUSE_FIELD(name_, , mask_, value_)

#define PIC32_ADEVCFG0(x) _PIC32_FUSE_FIELD(adevcfg0, PIC32_CONFIG_DEVCFG0_MASK, x)
#define PIC32_ADEVCFG1(x) _PIC32_FUSE_FIELD(adevcfg1, PIC32_CONFIG_DEVCFG1_MASK, x)
#define PIC32_ADEVCFG2(x) _PIC32_FUSE_FIELD(adevcfg2, PIC32_CONFIG_DEVCFG2_MASK, x)
#define PIC32_ADEVCFG3(x) _PIC32_FUSE_FIELD(adevcfg3, PIC32_CONFIG_DEVCFG3_MASK, x)
#define PIC32_ADEVCP0(x) _PIC32_FUSE_FIELD(adevcp0, 0, x)
#define PIC32_ADEVCP1(x) _PIC32_FUSE_FIELD(adevcp1, 0, x)
#define PIC32_ADEVCP2(x) _PIC32_FUSE_FIELD(adevcp2, 0, x)
#define PIC32_ADEVCP3(x) _PIC32_FUSE_FIELD(adevcp3, 0, x)
#define PIC32_ADEVSIGN0(x) _PIC32_FUSE_FIELD(adevsign0, 0, x)
#define PIC32_ADEVSIGN1(x) _PIC32_FUSE_FIELD(adevsign1, 0, x)
#define PIC32_ADEVSIGN2(x) _PIC32_FUSE_FIELD(adevsign2, 0, x)
#define PIC32_ADEVSIGN3(x) _PIC32_FUSE_FIELD(adevsign3, 0, x)

#define PIC32_DEVCFG0(x) _PIC32_FUSE_FIELD(devcfg0, PIC32_CONFIG_DEVCFG0_MASK, x)
#define PIC32_DEVCFG1(x) _PIC32_FUSE_FIELD(devcfg1, PIC32_CONFIG_DEVCFG1_MASK, x)
#define PIC32_DEVCFG2(x) _PIC32_FUSE_FIELD(devcfg2, PIC32_CONFIG_DEVCFG2_MASK, x)
#define PIC32_DEVCFG3(x) _PIC32_FUSE_FIELD(devcfg3, PIC32_CONFIG_DEVCFG3_MASK, x)
#define PIC32_DEVCP0(x) _PIC32_FUSE_FIELD(devcp0, 0, x)
#define PIC32_DEVCP1(x) _PIC32_FUSE_FIELD(devcp1, 0, x)
#define PIC32_DEVCP2(x) _PIC32_FUSE_FIELD(devcp2, 0, x)
#define PIC32_DEVCP3(x) _PIC32_FUSE_FIELD(devcp3, 0, x)
#define PIC32_DEVSIGN0(x) _PIC32_FUSE_FIELD(devsign0, 0, x)
#define PIC32_DEVSIGN1(x) _PIC32_FUSE_FIELD(devsign1, 0, x)
#define PIC32_DEVSIGN2(x) _PIC32_FUSE_FIELD(devsign2, 0, x)
#define PIC32_DEVSIGN3(x) _PIC32_FUSE_FIELD(devsign3, 0, x)

#endif

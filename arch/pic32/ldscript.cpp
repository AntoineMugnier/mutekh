
#define _PIC32_FUSE_STR(x) #x
#define PIC32_FUSE_STR(x) _PIC32_FUSE_STR(x)
#define PIC32_FUSE(addr, name, default)                                \
    pic32_fuse_##name = .;                                             \
    KEEP(*(.pic32_fuses.name));                                        \
    *(.pic32_fuses.name);                                              \
    FILL(default);                                                     \
    . += (NEXT(.) == (addr + 4)) ? 0 : 4;                              \
    ASSERT(NEXT(4) == (addr + 4), PIC32_FUSE_STR(PIC32 fuse name not defined exactly once))

#define ARCH_LDSCRIPT_SECTIONS                                          \
    .pic32fuses 0x9fc0ff40 : AT(0x9fc0ff40) {                           \
        PIC32_FUSE(0x9fc0ff40, adevcfg3, 0xffffffff);                   \
        PIC32_FUSE(0x9fc0ff44, adevcfg2, 0xffffffff);                   \
        PIC32_FUSE(0x9fc0ff48, adevcfg1, 0xffffffff);                   \
        PIC32_FUSE(0x9fc0ff4c, adevcfg0, 0xffffffff);                   \
        PIC32_FUSE(0x9fc0ff50, adevcp3, 0xffffffff);                    \
        PIC32_FUSE(0x9fc0ff54, adevcp2, 0xffffffff);                    \
        PIC32_FUSE(0x9fc0ff58, adevcp1, 0xffffffff);                    \
        PIC32_FUSE(0x9fc0ff5c, adevcp0, 0xffffffff);                    \
        PIC32_FUSE(0x9fc0ff60, adevsign3, 0xffffffff);                  \
        PIC32_FUSE(0x9fc0ff64, adevsign2, 0xffffffff);                  \
        PIC32_FUSE(0x9fc0ff68, adevsign1, 0xffffffff);                  \
        PIC32_FUSE(0x9fc0ff6c, adevsign0, 0xffffffff);                  \
        . += 0x50;                                                      \
        PIC32_FUSE(0x9fc0ffc0, devcfg3, 0x87ffdefa);                    \
        PIC32_FUSE(0x9fc0ffc4, devcfg2, 0x7ff9b1a8);                    \
        PIC32_FUSE(0x9fc0ffc8, devcfg1, 0x7f733f39);                    \
        PIC32_FUSE(0x9fc0ffcc, devcfg0, 0xffffffdf);                    \
        PIC32_FUSE(0x9fc0ffd0, devcp3, 0xffffffff);                     \
        PIC32_FUSE(0x9fc0ffd4, devcp2, 0xffffffff);                     \
        PIC32_FUSE(0x9fc0ffd8, devcp1, 0xffffffff);                     \
        PIC32_FUSE(0x9fc0ffdc, devcp0, 0xffffffff);                     \
        PIC32_FUSE(0x9fc0ffe0, devsign3, 0xffffffff);                   \
        PIC32_FUSE(0x9fc0ffe4, devsign2, 0xffffffff);                   \
        PIC32_FUSE(0x9fc0ffe8, devsign1, 0xffffffff);                   \
        PIC32_FUSE(0x9fc0ffec, devsign0, 0xffffffff);                   \
}

#include "../common/ldscript.cpp"

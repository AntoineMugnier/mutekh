
#define ARCH_LDSCRIPT_SECTIONS                  \
.pic32fuses 0x9fc0ff40 : AT(0x9fc0ff40) {       \
          KEEP(*(.pic32fuses))                  \
}

#include "../common/ldscript.cpp"


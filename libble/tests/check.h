
#define CHECK(label, o, x...) {                         \
    const uint8_t tmp[] = {x};                          \
                                                        \
    printk("Testing %s...", label);                     \
                                                        \
    if (memcmp(o, tmp, sizeof(tmp))) {                  \
      printk(" failed:\n");                             \
      printk("  expected %P\n", tmp, sizeof(tmp));      \
      printk("  got      %P\n", o, sizeof(tmp));        \
    } else {                                            \
      printk(" OK\n");                                  \
    }                                                   \
  }

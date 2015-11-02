
#define CHECK_DATA(label, o, ref, len) {                \
    printk("Testing %s...", label);                     \
                                                        \
    if (memcmp(o, ref, len)) {                          \
      printk(" failed:\n");                             \
      printk("  expected %P\n", ref, len);              \
      printk("  got      %P\n", o, len);                \
      abort();                                          \
    } else {                                            \
      printk(" OK\n");                                  \
    }                                                   \
  }

#define CHECK(label, o, x...) {                         \
    const uint8_t tmp[] = {x};                          \
    CHECK_DATA(label, o, tmp, sizeof(tmp));             \
  }

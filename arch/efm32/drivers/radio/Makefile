
objs = common.o protimer.o sequencer.o

objs-$(CONFIG_DRIVER_EFR32_RFPACKET) += rfpacket.o radio_config.o
objs-$(CONFIG_DRIVER_EFR32_BLE_ADVERTISE) += ble_advertise.o

$(LOCAL_OBJ_DIR)/%.o: $(LOCAL_SRC_DIR)/%.seq $(OBJ_DIR)/config.h
	@echo 'SEQ     $(notdir $@)'
	$(CC) -E -x c $(CFLAGS) $(CPUCFLAGS) $(ARCHCFLAGS) $(INCS) -I $(LOCAL_SRC_DIR) $< -o $@.i
	perl $(MUTEK_SRC_DIR)/scripts/decl_filter.pl --parse-decl $(CC) \
                $(CFLAGS) $(CPUCFLAGS) $(ARCHCFLAGS) $(INCS) -I $(LOCAL_SRC_DIR) < $@.i > $@.ii
	perl -I $(MUTEK_SRC_DIR)/arch/efm32 $(MUTEK_SRC_DIR)/arch/efm32/seq_asm.pl $@.ii -o $@.bin
	perl $(MUTEK_SRC_DIR)/scripts/blob2c.pl $@.bin -n seqcode -S seqcode_size -D 4 > $@.c
	$(CC) -x c -c $(CFLAGS) $(CPUCFLAGS) $(ARCHCFLAGS) $(INCS) $@.c -o $@

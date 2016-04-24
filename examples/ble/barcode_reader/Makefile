objs = main.o dis.o hids.o bas.o gap.o keymap.o keymap_fr.o keymap_fr_mac.o keymap_us.o keymap_iterator.o mcr12.o led.o poweroff.o scps.o config.o

$(LOCAL_OBJ_DIR)/hids.o: $(LOCAL_OBJ_DIR)/report_descriptor.h

$(LOCAL_OBJ_DIR)/report_descriptor.h: $(LOCAL_SRC_DIR)/report_descriptor.py
	mkdir -p $(dir $@)
	python $< -o code $@

hids.o_CFLAGS = -iquote $(LOCAL_OBJ_DIR)

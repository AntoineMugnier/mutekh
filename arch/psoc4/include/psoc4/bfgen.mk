
HEADERS = scb.h hsiom.h
BFGEN_CDEFS = -o cdefs cdefs_use_reg_mask=1 cdefs_use_field_setval=1

all: $(HEADERS) $(HEADERS:.h=.html)

clean:
	rm -f $(HEADERS)

%.h: %.bf
	bfgen $(BFGEN_CDEFS) -O $@ -I $<

%.html: %.bf
	bfgen -o html -O $@ -I $<

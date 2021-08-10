
#MUTEKH=~/mutekh/
CSRC=bytecode.c ../gfx.c ../bytecode.c
TSRC=../include/gfx/gfx.t ../include/gfx/pixel.t ../include/gfx/line.t ../include/gfx/rect.t ../include/gfx/circle.t ../include/gfx/arc.t ../include/gfx/blit.t
HSRC=../include/gfx/bytecode.h ../include/gfx/math.h

all: test_tile.out test_circle.out test_line.out test_rect.out test_arc.out test_blit.out test_scroll.out test_arith.out test

%.h: %.t
	perl backslash.pl $< $@

%.out: %.bc bc_custom_gfx.pm $(TSRC:.t=.h) $(HSRC)
	gcc -x c -o - -E -I ../include $< | perl $(MUTEKH)scripts/decl_filter.pl | perl $(MUTEKH)scripts/bc_asm.pl -w 2 -b blob -p . -o $@

test: test.c $(CSRC) $(TSRC:.t=.h) $(HSRC)
	gcc -Wall -Wsign-compare -include bytecode.h -I ../include -ggdb -O test.c $(CSRC) $(shell pkg-config --cflags --libs sdl) -o test

clean:
	rm -f *.out *.out.adr test $(TSRC:.t=.h)

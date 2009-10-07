CXX=g++
CXXFLAGS=-m32 -Wall -I$(ELFPP)/elfpp -I$(ELFPP)
ASFLAGS=-arch i386
LDFLAGS=-L$(ELFPP)/.libs -m32 -lelfpp

all: elfrunner

elfrunner: elfrunner.o
	$(CXX) -o $@ $^ $(LDFLAGS) $(CXXFLAGS)

%.o: %.cc
	$(CXX) -c -o $@ $< $(CXXFLAGS)

%.o: %.s
	$(AS) -o $@ $< $(ASFLAGS)

clean:
	rm -f elfrunner *.o

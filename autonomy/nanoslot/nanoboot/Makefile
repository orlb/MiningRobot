OPTS=-O -g
CFLAGS=-I../../include -std=c++17 $(OPTS)
PROGS=nanoboot

all: $(PROGS)

nanoboot: nanoboot.cpp
	g++ $(CFLAGS) $< -o $@

clean:
	- rm $(PROGS)

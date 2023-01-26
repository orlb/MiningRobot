OPTS=-O -g
CFLAGS=-I../../include -I../include -std=c++17 $(OPTS)

all: $(PROG)

$(PROG): *.cpp
	g++ $(CFLAGS) $< -o $@

clean:
	- rm $(PROG)

OPTS=-O -g -Wall
CFLAGS=-I../../include -std=c++17 $(OPTS)

all: $(PROG)

$(PROG): *.cpp
	g++ $(CFLAGS) $< -o $@

clean:
	- rm $(PROG)

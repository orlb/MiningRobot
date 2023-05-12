include ../../Makefile.inc
OPTS=-g
INC=../../include
CFLAGS=-I$(INC)  -Wall  -std=c++17  $(OPTS) $(CVCFLAGS)
LIBS=$(CVLINK)
PROGS=ik_test

all: $(PROGS)

ik_test: ik_test.cpp $(INC)/*/*
	g++ $(OPTS) $(CFLAGS) $< -o $@ $(LIBS)

clean:
	- rm $(PROGS)

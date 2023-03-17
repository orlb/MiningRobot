include ../../Makefile.inc
OPTS=-O4
INC=../../include
CFLAGS=-I$(INC)  -Wall  -std=c++17  $(OPTS) $(CVCFLAGS)
LIBS=$(CVLINK)
PROGS=dump_joints

all: $(PROGS)

dump_joints: dump_joints.cpp $(INC)/*/*
	g++ $(OPTS) $(CFLAGS) $< -o $@ $(LIBS)

clean:
	- rm $(PROGS)

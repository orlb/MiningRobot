include ../Makefile.inc
OPTS=-O4
LD = g++
CXX = g++
CFLAGS=-I../include  -Wall  -std=c++17 -DGL_SILENCE_DEPRECATION  $(OPTS) $(CVCFLAGS)
LDFLAGS =  -lpqxx -lpq
LIBS=$(CVLINK)
PROGS=lunacapture lunaview

all: $(PROGS)

lunaview: lunaview.cpp
	g++ $(CFLAGS) $< -o $@ $(LIBS)

lunacapture: lunacapture.cpp
	g++ $(CFLAGS) $< -o $@

clean:
	- rm $(PROGS)

include ../Makefile.inc

OPTS=-g -O3 -std=c++17
ARUCO=/usr/local
CFLAGS=$(OPTS) $(CVCFLAGS) -I../include -I$(ARUCO)/include/aruco 
LFLAGS=$(OPTS) -L$(ARUCO)/lib -laruco  $(CVLINK)

all: camera

camera: main.cpp
	g++ $< -o $@ $(CFLAGS) $(LFLAGS)

clean:
	- rm camera


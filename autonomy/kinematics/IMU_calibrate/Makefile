OPTS=-O4
INC=../../include
CFLAGS=-Wall -I$(INC) -std=c++17 $(OPTS)
PROGS=IMU_calibrate

all: $(PROGS)

IMU_calibrate: main.cpp $(INC)/aurora/* $(INC)/nanoslot/*
	g++ $(CFLAGS) $< -o $@

clean:
	- rm $(PROGS)


include ../Makefile.inc

OPTS=-O4
CFLAGS=-Wall -I../include -std=c++11 $(OPTS) $(CVCFLAGS)
LIBS=-laruco -lrealsense2 $(CVLINK)
PROGS=vision vision_mining vision_capture

all: $(PROGS)

vision: vision.cpp ../include/*/*
	g++ $(CFLAGS) $< -o $@ $(LIBS)


calibration: calibration.cpp ../include/*/*
	g++ $(CFLAGS) $< -o $@ $(LIBS)

vision_mining: vision_mining.cpp ../include/*/*
	g++ $(CFLAGS) $< -o $@ $(LIBS)

vision_capture: vision_capture.cpp ../include/*/*
	g++ $(CFLAGS) $< -o $@ $(LIBS)

clean:
	- rm $(PROGS)


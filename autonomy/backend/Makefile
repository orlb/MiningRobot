COMPILER=g++ -std=c++11

DIRS=-I/usr/local/include -L/usr/local/lib -I../include

ifeq ($(OS),Windows_NT)
	LIBS+=-lWs2_32
	#ADD LIBS FOR GL/GLUT on windows...
else
	LIB+=-lpthread

	ifeq ($(shell uname),Darwin)
		LIB+=-framework OpenGL -framework GLUT -framework IOKit -framework CoreFoundation
	else
		LIB+=-lGL -lGLU -lglut
	endif
endif

OPTS=-g -O4
CFLAGS=$(OPTS) -Wall -Wno-psabi -Wno-deprecated-declarations -Wno-char-subscripts

all: backend

backend: main.cpp
	$(COMPILER) $^ $(LIB) $(CFLAGS) $(DIRS) -o $@

clean:
	rm -f backend backend.exe

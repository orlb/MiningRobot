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

CFLAGS=-O -Wall -Wno-deprecated-declarations -Wno-char-subscripts

all: frontend

frontend: main.cpp 
	$(COMPILER) $^ $(LIB) $(CFLAGS) $(DIRS) -o $@

frontend_msl: main.cpp 
	$(COMPILER) $^ $(LIB) $(CFLAGS) $(DIRS) -o $@ -D MSL -std=c++11

clean:
	rm -f frontend frontend.exe frontend_msl frontend_msl.exe

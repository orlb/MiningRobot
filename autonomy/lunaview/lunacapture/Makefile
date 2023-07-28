# Note, Makefile requires a tab char to start a recipe, not 4 spaces!
TARGET = lunacapture
SOURCES = $(wildcard *.cpp)
HEADERS = $(wildcard *.h*)
OBJECTS = $(SOURCES:%.cpp=%.o)
LD = g++
CXX = g++
CXXFLAGS = -std=c++17 -g -Wall -I../../include -DGL_SILENCE_DEPRECATION
LDFLAGS = -lpqxx -lpq

.PHONY: all clean

all: $(TARGET)

clean:
	$(RM) main.o src/*.o
	$(RM) $(TARGET)

$(TARGET): $(OBJECTS)
	$(LD) -o $@ $(OBJECTS) $(LDFLAGS)

%.o: %.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

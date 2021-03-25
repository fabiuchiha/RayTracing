
# - inc/
#     - *.h
# - MyRayTracer/
#     - *.c
#     - *.cpp
# - obj/
#     - *.o
# - main

TARGET := main
SOURCES := $(wildcard MyRayTracer/*.c MyRayTracer/*.cpp)
OBJECTS := $(patsubst MyRayTracer%,obj%, $(patsubst %.c,%.o, $(patsubst %.cpp,%.o,$(SOURCES))))

INCLUDE := -I. -IMyRayTracer/Dependencies/conio
LIBPATH :=
LIBS := -lglut -lGL -lGLU -lGLEW -lm -lIL

FLAGS := -Wall
CCFLAGS := $(FLAGS) -std=c11
CXXFLAGS := $(FLAGS)

CC := gcc
Cxx := g++

all: $(OBJECTS)
	$(Cxx) $(CCFLAGS) $(INCLUDE) $(OBJECTS) -o $(TARGET) $(LIBPATH) $(LIBS)

%.o: ../MyRayTracer/%.c
	$(CC) $(CCFLAGS) $(INCLUDE) -c $< -o $@

%.o: ../MyRayTracer/%.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE) -c $< -o $@

clean:
	rm -rf obj/*
	rm -f $(TARGET)
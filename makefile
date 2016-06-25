#---------------------------------------------------
# Target file to be compiled by default
#---------------------------------------------------
MAIN = quadrotors
#---------------------------------------------------
# Source files
#---------------------------------------------------
SOURCES = $(wildcard *.c)
OBJECTS = $(patsubst %.c, %.o, $(SOURCES))
#---------------------------------------------------
# CC will be the compiler to use
#---------------------------------------------------
CC = gcc
#---------------------------------------------------
# CFLAGS will be the options passed to the compiler
#---------------------------------------------------
CFLAGS = -Wall
FLAGS = -lm `pkg-config --cflags --libs allegro`
#---------------------------------------------------
# Dependencies
#---------------------------------------------------
$(MAIN): $(OBJECTS)
	$(CC) -o $(MAIN) $^ $(FLAGS)
clean:
	rm *.o

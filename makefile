#---------------------------------------------------
# Target file to be compiled by default
#---------------------------------------------------
MAIN = quadrotors
#---------------------------------------------------
# Source files, object files
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
FLAGS = -Wall -lm -lpthread `pkg-config --cflags --libs allegro`
#---------------------------------------------------
# Default goal first
#---------------------------------------------------
$(MAIN): $(OBJECTS)
	$(CC) -o $(MAIN) $^ $(FLAGS)
#---------------------------------------------------
# Clean goal
#---------------------------------------------------
clean:
	rm -f *.o *.d
#---------------------------------------------------
# Generate prerequisites
# line continuation (; \) assures that $$ is preserved
# in every call
#---------------------------------------------------
%.d: %.c
	rm -f $@; \
	$(CC) -MM $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$
#---------------------------------------------------
# Include prerequisites
# this is after main goal in order to avoid random main goals
#---------------------------------------------------
include $(SOURCES:.c=.d)


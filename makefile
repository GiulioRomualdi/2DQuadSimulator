#---------------------------------------------------
# Target file to be compiled by default
#---------------------------------------------------
MAIN = quadrotors
#---------------------------------------------------
# CC will be the compiler to use
#---------------------------------------------------
CC = gcc
#---------------------------------------------------
# CFLAGS will be the options passed to the compiler
#---------------------------------------------------
CFLAGS = -Wall
#---------------------------------------------------
# Dependencies
#---------------------------------------------------
$(MAIN): $(MAIN).o
	$(CC) $(CFLAGS) -o $(MAIN) $(MAIN).o
$(MAIN).o: $(MAIN).c
	$(CC) -c $(MAIN).c
simulator.o: simulator.c
	$(CC) -c simulator.c

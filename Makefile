#  Makefile for P1: File Sys Project;  CEG 4350/6350, Matt Piekenbrock

CFLAGS = -g -Wall -pedantic -std=c++11 -I/usr/local/include -L/usr/local/lib
CC = g++

.SUFFIXES: .cpp .o .C

.C.o:
	$(CC) $(CFLAGS) -c $<

.cpp.o:
	$(CC) $(CFLAGS) -c $<


FILES= main.cpp

OBJFILES = main.o

PROJECT = track

$(PROJECT): $(OBJFILES)
	g++ -o $(PROJECT) $(CFLAGS) $(OBJFILES)

test:   $(PROJECT)	
	rm -fr D?.bin
	./$(PROJECT)

$(OBJFILES):  Makefile

indent:
	indent -kr -i2 -pmt *.C *.h

tar archive: clean
	(cd ..; tar cvvfj ./$(PROJECT).tbz $(PROJECT); ls -l $(PROJECT).tbz)

clean:
	rm -fr *.o *~ *.out $(PROJECT) D?.dsk *.f33 d1.txt


# -eof-

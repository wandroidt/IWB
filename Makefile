#  Makefile for IWB Project;  Senior Design, Matt Piekenbrock

CFLAGS = -g -Wall -pedantic -std=c++11 -I/usr/local/include -L/usr/local/lib
CC = g++

OPENCV_LIBS_pc = `pkg-config --libs opencv`
OPENCV_LIBS_man = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_flann
.SUFFIXES: .cpp .o .C

.C.o:
	$(CC) $(CFLAGS) -c $<

.cpp.o:
	$(CC) $(CFLAGS) -c $<


FILES= main.cpp

OBJFILES = main.o

PROJECT = track

$(PROJECT): $(OBJFILES)
	g++ -o $(PROJECT) $(CFLAGS) $(OBJFILES) $(OPENCV_LIBS_man)

$(OBJFILES):  Makefile

indent:
	indent -i2 -pmt *.C *.h

tar archive: clean
	(cd ..; tar cvvfj ./$(PROJECT).tbz $(PROJECT); ls -l $(PROJECT).tbz)

clean:
	rm -fr *.o *~ *.out $(PROJECT)


# -eof-
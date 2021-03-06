#  Makefile for IWB Project;  Senior Design, Matt Piekenbrock

PROJECT = IWB
FILES= main.cpp calibratePiCamera.h calibration.cpp README.md LICENSE
OBJFILES = calibration.o
EXE = calibrateCamera
CC = g++
CFLAGS = -g -Wall -pedantic -std=c++11 -stdlib=libstdc++

OPENCV_LIBS_pkg = `pkg-config --libs opencv`
OPENCV_LIBS = -L/usr/local/lib \
-lopencv_core \
-lopencv_highgui \
-lopencv_imgproc \
-lopencv_calib3d \
-lopencv_features2d \
-lopencv_nonfree

%.o : %.C
	$(CC) -c $< $(CFLAGS)

%.o : %.cpp
	$(CC) -c $< $(CFLAGS)

$(PROJECT): $(OBJFILES)
	$(CC) $(OBJFILES) -o $(EXE) $(CFLAGS) $(OPENCV_LIBS)

indent:
	indent -i2 -pmt *.C *.cpp *.h

tar: clean
	tar --version; tar cfvz ./archives/$(PROJECT)_$(shell date +"%m-%d-%y").tar.gz $(FILES); ls -l ./archives/$(PROJECT)_$(shell date +"%m-%d-%y").tar.gz;
gzip: clean
	tar --version; GZIP=-9 tar cfvz ./archives/$(PROJECT)_$(shell date +"%m-%d-%y").tar.gz $(FILES); ls -l ./archives/$(PROJECT)_$(shell date +"%m-%d-%y").tar.gz;
bzip2: clean
	tar --version; BZIP=-9 tar cfvy ./archives/$(PROJECT)_$(shell date +"%m-%d-%y").tar.bz2 $(FILES); ls -l ./archives/$(PROJECT)_$(shell date +"%m-%d-%y").tar.bz2;
xz: clean
	tar --version; XZ_OPT=-9 tar cfvJ ./archives/$(PROJECT)_$(shell date +"%m-%d-%y").tar.xz $(FILES); ls -l ./archives/$(PROJECT)_$(shell date +"%m-%d-%y").tar.xz;

clean:
	rm -fr *.o *~ *.out $(EXE)


# -eof-

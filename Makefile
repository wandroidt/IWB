#  Makefile for IWB Project;  Senior Design, Matt Piekenbrock

PROJECT = IWB
FILES= main.cpp calibratePiCamera.h README.md LICENSE
OBJFILES = main.o
EXE = track
CC = g++
CFLAGS = -g -Wall -pedantic -std=c++11 -stdlib=libstdc++

OPENCV_LIBS_pkg = `pkg-config --libs opencv`
OPENCV_LIBS = \
-lopencv_core \
-lopencv_highgui \
-lopencv_features2d \
-lopencv_imgproc \
-lopencv_ml \
-lopencv_video \
-lopencv_calib3d \
-lopencv_objdetect \
-lopencv_contrib \
-lopencv_legacy \
-lopencv_flann \
-lopencv_nonfree

%.o : %.C
	$(CC) -c $< $(CFLAGS)

%.o : %.cpp
	$(CC) -c $< $(CFLAGS)

$(PROJECT): $(OBJFILES)
	g++ $(OBJFILES) -o $(EXE) $(CFLAGS) $(OPENCV_LIBS)

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
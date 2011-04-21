#
# Makefile for blobs library for OpenCV
#

CFLAGS=`pkg-config --cflags opencv` -I.
LDFLAGS=`pkg-config --libs opencv` 
CXX=g++

CPPFILES= \
	Blob.cpp \
        BlobContour.cpp \
	BlobOperators.cpp \
	BlobResult.cpp \
        ComponentLabeling.cpp


.SUFFIXES: .cpp.o
.cpp.o:	; echo 'Compiling $*.cpp' ; $(CXX) $(CFLAGS) -c $*.cpp

.SILENT:

libcvblobs.a: $(CPPFILES:.cpp=.o)
	ar ru libcvblobs.a $(CPPFILES:.cpp=.o) 2> /dev/null
	ranlib libcvblobs.a
	@echo Copy include files...
#
	@echo Cleaning objects...
	rm -f $(CPPFILES:.cpp=.o)

example:
	$(CXX) testBlobs/main.cpp $(CFLAGS) $(LDFLAGS) -L. libcvblobs.a -o example

all: clean

clean:
	@echo Cleaning...
	rm -f $(CPPFILES:.cpp=.o)
	rm -f libcvblobs.a
	rm -f testBlobs/main.o
	rm -f example
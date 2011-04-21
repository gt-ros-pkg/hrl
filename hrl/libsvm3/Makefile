CXX ?= g++
CFLAGS = -Wall -Wconversion -O3 -fPIC
SHVER = 2

all: svm-train svm-predict svm-scale lib

lib: svm.o
	$(CXX) -shared -dynamiclib svm.o -o libsvm.so.$(SHVER)
	ln -s libsvm.so.$(SHVER) ./libsvm.so --force

svm-predict: svm-predict.c svm.o
	$(CXX) $(CFLAGS) svm-predict.c svm.o -o svm-predict -lm
svm-train: svm-train.c svm.o
	$(CXX) $(CFLAGS) svm-train.c svm.o -o svm-train -lm
svm-scale: svm-scale.c
	$(CXX) $(CFLAGS) svm-scale.c -o svm-scale
svm.o: svm.cpp svm.h
	$(CXX) $(CFLAGS) -c svm.cpp
clean:
	rm -f *~ svm.o svm-train svm-predict svm-scale libsvm.so.$(SHVER)

install:
	cp libsvm.so.$(SHVER) /usr/lib/libsvm.so.$(SHVER)
	cp python/svm.py /usr/lib/python2.6/svm.py
	cp python/svmutil.py /usr/lib/python2.6/svmutil.py

uninstall:
	rm /usr/lib/libsvm.so.$(SHVER)
	rm /usr/lib/python2.6/svm.py
	rm /usr/lib/python2.6/svmutil.py

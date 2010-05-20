INCLUDE_FLAGS =`pkg-config --cflags opencv`
LINK_FLAGS    =`pkg-config --libs opencv`
CMD           = g++ -Wall

all: videreo swigify
	$(CMD) $(INCLUDE_FLAGS) $(LINK_FLAGS) videre.o sample.cpp -o sample

videreo:
	$(CMD) $(INCLUDE_FLAGS) videre.cpp -c

swigify: videreo
	swig -c++ -python videre.i
	g++ $(INCLUDE_FLAGS) -I/usr/include/python2.5 -c videre.cpp videre_wrap.cxx
	ld $(LINK_FLAGS) -shared videre.o videre_wrap.o -o _videre.so

clean:
	rm -f *.o
	rm -f *.so
	rm -f sample

 

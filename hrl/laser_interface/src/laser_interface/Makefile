
all: 
	cd videre_interface; $(MAKE) $(MFLAGS)
	ln -fs videre_interface/_videre.so
	ln -fs videre_interface/videre.py

clean:
	cd videre_interface; $(MAKE) $(MFLAGS) clean
	rm _videre.so
	rm videre.py

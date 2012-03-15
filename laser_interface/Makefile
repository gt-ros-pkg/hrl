all: gen_msgs
	ln -fs `rospack find wg_robot_description`/pr2/cameras.xml .
	cd src; $(MAKE) $(MFLAGS)

clean: clean_msgs
	rm cameras.xml
	cd src; $(MAKE) $(MFLAGS) clean

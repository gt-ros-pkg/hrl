PACKAGE_NAME = sample_pkg

all:
	`rospack find roslib`/scripts/gensrv.py srv/*.srv
	`rospack find roslib`/scripts/genmsg.py msg/*.msg

clean:
	rm -rf build lib msg/cpp msg/oct msg/lisp srv/cpp srv/oct srv/lisp
	rm -rf src/${PACKAGE_NAME}/msg src/${PACKAGE_NAME}/srv




PACKAGE_NAME = force_torque

all:
	`rospack find roslib`/scripts/gensrv srv/*.srv
	`rospack find roslib`/scripts/genmsg msg/*.msg

clean:
	rm -rf bin build lib msg/cpp msg/oct msg/lisp srv/cpp srv/oct srv/lisp
	rm -rf src/${PACKAGE_NAME}/msg src/${PACKAGE_NAME}/srv




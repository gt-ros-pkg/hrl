import robotis_servo as rs

for i in xrange(255):
    try:
        s = rs.robotis_servo('/dev/ttyUSB0',i)
        print '\n\n FOUND YOUR SERVO ID @ %d\n\n' % i
    except:
        pass

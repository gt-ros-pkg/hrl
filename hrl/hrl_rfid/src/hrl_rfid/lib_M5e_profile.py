#
# Copyright (c) 2009, Georgia Tech Research Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

#  \author Travis Deyle (Healthcare Robotics Lab, Georgia Tech.)


##  Thing Magic M5e and M5e-Compact Python Interface
##  Written by Travis Deyle please remember me if you become rich and/or famous, and please  
##    refer to our related work(s) when writing papers.  Thanks.

import sys, serial, time, string
from threading import Thread

# A nice functional print function (Norvig)
def prin(x): print x


# This class is more general in that it allows custum antenna functions (if
# using custom hardware) in addition to callbacks.

class M5e_Poller(Thread):
    QUERY_MODE = 'query'
    TRACK_MODE = 'track'

    def __init__(self, M5e, antfuncs=[], callbacks=[]):
        Thread.__init__(self)
        self.M5e = M5e
        self.antfuncs = antfuncs
        self.callbacks = callbacks
        self.mode = ''
        self.should_run = True
        
        print 'Creating M5e Polling Thread'
        self.start()

    def pause_poller(self):
        self.mode = ''

    def query_mode(self):
        self.mode = self.QUERY_MODE

    def track_mode(self, tag_id):
        self.mode         = self.TRACK_MODE
        self.tag_to_track = tag_id
    
    def run(self):
        while self.should_run:
            if self.mode == self.QUERY_MODE:
                for aF in self.antfuncs:
                    antennaName = aF(self.M5e)    # let current antFunc make appropriate changes
                    results = self.M5e.QueryEnvironment()
                    for tagid, rssi in results:
                        datum = [antennaName, tagid, rssi]
                        [cF(datum) for cF in self.callbacks]
            elif self.mode == self.TRACK_MODE:
                for aF in self.antfuncs:
                    antennaName = aF(self.M5e)    # let current antFunc make appropriate changes
                    tagid = self.tag_to_track
                    rssi = self.M5e.TrackSingleTag(tagid)
                    #if rssi != -1:
                    datum = [antennaName, tagid, rssi]
                    [cF(datum) for cF in self.callbacks]
            else:
                time.sleep(0.005)
        
    def stop(self):
        self.should_run = False
        self.join(3)
        if (self.isAlive()): 
            raise RuntimeError("RFID M5e Poller: unable to stop thread")
    
class M5e:
    "Interface to Mercury M5e and M5e-Compact"
    def __init__(self, portSTR='/dev/robot/RFIDreader', baudrate=9600, 
        TXport=1, RXport=1, readPwr=2300, protocol='GEN2', compact=True, verbosity_func=prin):

        # verbosity_func should alawys take string as an input.  Lets us choose a selective 
        #   print function (like rospy.logout) instead of built-in print.

        try:
            self.port = string.atoi( portSTR ) # stores the serial port as 0-based integer for Windows
        except:
            self.port = portSTR # stores it as a /dev-mapped string for Linux / Mac
        
        self.baudrate = baudrate    # should be 9600 for M5e by default.  May want to up the baud for faster reading (after bootup)
        self.TXport = TXport        # Initialized transmit antenna
        self.RXport = RXport        # Initialized receive antenna
        self.readPwr = readPwr      # Initialized read TX power in centi-dBm
        self.protocol = protocol    # Initialized protocol
        self.compact = compact
        self.prin = verbosity_func
        self.ser = None
        
        self.prin( 'Initializing M5e (or M5e-Compact)' )
        
        # Setup Serial port  (kinda Hacky, but it works and I don't want to waste time on it)
        #    Read/Write timeouts should be appropriate for data sizes being dealt with.  Keep this in mind
        
        failed = False
        try:
            self.prin( '\tAttempting 230400 bps' )
            self.ser = serial.Serial(self.port, 230400, timeout=2, writeTimeout=2)
            # Check if BootLoader is running by issuing "Get BootLoader/Firmware Version"        
            self.TransmitCommand('\x00\x03')
            self.ReceiveResponse()      # automatically makes sure returned status = 0x0000
            self.prin( '\tSuccessful at 230400 bps' )
        except:
            self.ser = None
            failed = True
            self.prin( '\tFailed @ 230400 bps' )
            pass
        
        if failed:
            self.prin( '\tAttempting 9600 bps' )
            try:
                self.ser = serial.Serial(self.port, 9600, timeout=2, writeTimeout=2)
                self.TransmitCommand('\x00\x03')
                self.ReceiveResponse()      # automatically makes sure returned status = 0x0000
                self.prin( '\tSuccessful @ 9600 bps' )
            except:
                self.prin( '\tFailed 9600 bps' )
                self.ser = None
                raise M5e_SerialPortError('Could not open serial port %s at baudrate %d or %d.' % (str(self.port),230400,9600))
            
            # Change baud to 230400 bps (instead of 9600 default)
            self.prin( '\tSwitching to 230400 bps' )
            self.TransmitCommand('\x04\x06\x00\x03\x84\x00')
            self.ReceiveResponse()    
            self.ser = None
            try:
                self.prin( '\tAttempting to reconnect @ 230400 bps' )
                self.ser = serial.Serial(self.port, 230400, timeout=2, writeTimeout=2)
                # Check if BootLoader is running by issuing "Get BootLoader/Firmware Version"        
                self.TransmitCommand('\x00\x03')
                self.ReceiveResponse()      # automatically makes sure returned status = 0x0000
                self.prin( '\tSuccessfully reconnected at 230400 bps' )
            except:
                self.ser = None
                self.prin( '\tFailed @ 230400 bps' )
                pass
        
        if self.ser == None:
            raise M5e_SerialPortError('Could not open serial port %s at baudrate %d or %d.' % (str(self.port),230400,9600))
        
        # Check if BootLoader is running by issuing "Get BootLoader/Firmware Version"        
        self.TransmitCommand('\x00\x03')
        self.ReceiveResponse()      # automatically makes sure returned status = 0x0000
        
        # Boot into Firmware
        self.TransmitCommand('\x00\x04')
        try:
            self.ReceiveResponse()
        except M5e_CommandStatusError, inst:
            # Non-Zero Response will be received if the reader has already booted into firmware
            #   This occurs when you've already powered-up & previously configured the reader.  
            #   Can safely ignore this problem and continue initialization
                if inst[1] == '\x01\x01':       # This actually means "invalid opcode"
                    pass
                else:
                    raise
        
        self.ChangeAntennaPorts(self.TXport, self.RXport)
        self.ChangeTXReadPower(self.readPwr)
                
        # Set Current Tag Protocol to GEN2
        if self.protocol != 'GEN2':
            raise M5e_error('Sorry, GEN2 is only protocol supported at this time')
        self.TransmitCommand('\x02\x93\x00\x05')
        self.ReceiveResponse()
        
        # Set Region (we're only going to deal with North America)
        self.TransmitCommand('\x01\x97\x01')
        self.ReceiveResponse()
        
        # Set Power Mode (we'll just use default of "full power mode").
        # Use remaining defaults
        
        
        
    def ChangeAntennaPorts(self, TXport, RXport):
        "Changes TX and RX ports"
        self.TXport = TXport
        self.RXport = RXport
        self.TransmitCommand('\x02\x91' + chr(self.TXport) + chr(self.RXport))
        self.ReceiveResponse()
        
    def ChangeTXReadPower(self, readPwr):
        "Sets the Read TX Power based on current value of readPwr (in centi-dBm)"
        self.readPwr = readPwr
        readTXPwrHighByte = chr((self.readPwr & 0xFFFF) >> 8)
        readTXPwrLowByte = chr(self.readPwr & 0x00FF)
        self.TransmitCommand('\x02\x92'+ readTXPwrHighByte + readTXPwrLowByte)
        self.ReceiveResponse()
        
    def CalculateCRC(self, msg):
        "Implements CCITT CRC-16 defined in Mercury Embedded Module Dev Guide."
        crcResult = 0xFFFF
        for x in range(len(msg)):
            currChar = ord(msg[x])
            v = 0x80
            for y in range(8):
                xor_flag = 0
                if (crcResult & 0x8000):
                    xor_flag = 1
                crcResult = crcResult << 1
                if (currChar & v):
                    crcResult = crcResult + 1
                if (xor_flag):
                    crcResult = crcResult ^ 0x1021
                v = v >> 1
                crcResult = crcResult & 0xFFFF
            #print hex(currChar)
        return chr((crcResult >> 8) & 0xFF) + chr(crcResult & 0xFF)
        # return the 16-bit result in the form of 2 characters

    def ReturnHexString(self, hexStr):
        "Helper function to visualize a hex string (such as a hexCommand)"
        result = ''
        for i in range(len(hexStr)):
            result = result + hex(ord(hexStr[i])) + ' '
        return result

    def ConstructCommand(self, hexCommand):
        "Helper function to attach start byte and CRC to a command string"
        return '\xFF' + hexCommand + self.CalculateCRC(hexCommand)

    def TransmitCommand(self, command):
        "Transmits a command.  Should call ReceiveResponse before calling again."
        try:
            self.ser.write(self.ConstructCommand(command))
        except:
            raise M5e_TransmitTimeoutExceeded('Something happened (probably power failure) to disable serial transmission')

    def QuickTrackResponse( self ):
        "Does almost no error checking, and can hang..."

        #slcs = self.ser.read( 5 ) # start, length, command, status x 2
        slcs = self.fr( 5 )
        #print 'slcs: ', self.ReturnHexString( slcs )
        if slcs[3:] == '\x04\x00':  # Tag not found, return rssi -1
            #dc = self.ser.read( ord(slcs[1]) + 2 ) # finish reading response
            dc = self.sec( ord(slcs[1]) + 2 )
            
            #print 'dc: ', self.ReturnHexString( dc )
            return -1

        #dc = self.ser.read( 20 ) # data (18) and crc (2) -- rssi is dc[3]
        dc = self.sec( 20 )
        #print 'dc: ', self.ReturnHexString( dc )
        # not going to check crc!  (I've never seen a failed check anyway...)
        return ord( dc[3] )

    def fr( self, chars ):
        return self.ser.read( chars )

    def sec( self, chars ):
        return self.ser.read( chars )
        
    def ReceiveResponse(self):
        "Receives a single command's response"
        # Get Start byte, disregard anything else...
        timeoutsToWait = 5
        timeoutsWaited = 0

        while timeoutsWaited < timeoutsToWait:
            start = self.ser.read()	    # this is non-blocking.  Will wait for timeout duration, then return
            if start == '\xFF':
                break
            timeoutsWaited += 1
            
        if start != '\xFF':
            time.sleep(5)
            self.ser.flushInput()
            raise M5e_ReceiveError('Error in receive stream (start byte).  Waited 5 seconds then flushed input.  Try reissueing command and receive response.')
            
        length = self.ser.read()        # non blocking, returns after timeout
        if len(length) != 1:
            time.sleep(5)
            self.ser.flushInput()
            raise M5e_ReceiveError('Error in receive stream (length byte).  Waited 5 seconds then flushed input.  Try reissueing command and receive response.')

        command = self.ser.read()
        if len(command) != 1:
            time.sleep(5)
            self.ser.flushInput()
            raise M5e_ReceiveError('Error in receive stream (command byte).  Waited 5 seconds then flushed input.  Try reissueing command and receive response.')

        status = self.ser.read(2)       # non-blocking
        if len(status) != 2:
            time.sleep(5)
            self.ser.flushInput()
            raise M5e_ReceiveError('Error in receive stream (status bytes).  Waited 5 seconds then flushed input.  Try reissueing command and receive response.')

        data = self.ser.read(ord(length))
        if len(data) != ord(length):
            time.sleep(5)
            self.ser.flushInput()
            raise M5e_ReceiveError('Error in receive stream (data bytes).  Waited 5 seconds then flushed input.  Try reissueing command and receive response.')

        CRC = self.ser.read(2)
        if len(CRC) != 2:
            time.sleep(5)
            self.ser.flushInput()
            raise M5e_ReceiveError('Error in receive stream (CRC bytes).  Waited 5 seconds then flushed input.  Try reissueing command and receive response.')
        
        # Validate the CRC
        validateCRC = length + command + status + data
        if self.CalculateCRC(validateCRC) != CRC:
            raise M5e_CRCError('Received response CRC failed')
        
        # Check if return status was OK (0x0000 is success)
        if status != '\x00\x00':
            raise M5e_CommandStatusError('Received response returned non-zero status',status)
            
        return (start, length, command, status, data, CRC)
        
    def QueryEnvironment(self, timeout=50):
        # Read Tag ID Multiple
        timeoutHighByte = chr((timeout & 0xFFFF) >> 8)
        timeoutLowByte = chr(timeout & 0x00FF)
        try:
            self.TransmitCommand('\x04\x22\x00\x00'+timeoutHighByte+timeoutLowByte)
            self.ReceiveResponse()
        except M5e_CommandStatusError, inst:
            flag = False
            # Read & Get returns non-zero status when no tags found.  This is 
            #   an expected event... so ignore the exception.
            if inst[1] == '\x04\x00':   
                return []
            else:
                raise

        # Get Tag Buffer
        #   Get # Tags remaining
        self.TransmitCommand('\x00\x29')
        (start, length, command, status, data, CRC) = self.ReceiveResponse()
        
        readIndex = (ord(data[0]) << 8) + ord(data[1])
        writeIndex = (ord(data[2]) << 8) + ord(data[3])
        numTags = writeIndex - readIndex
        
#         tagEPCs = []
#         while numTags > 0:
#             numFetch = min([numTags, 13])
#             numFetchHighByte = chr((numFetch & 0xFFFF) >> 8)
#             numFetchLowByte = chr(numFetch & 0x00FF)
#             self.TransmitCommand('\x02\x29' + numFetchHighByte + numFetchLowByte)
#             (start, length, command, status, data, CRC) = self.ReceiveResponse()
            
#             # each tag occupies 18 bytes in the response, regardless of tag size
#             for i in range(numFetch): # NOTE: first 4 bytes in GEN2 are size & protocol.  Last 2 are CRC
#                 tagEPCs.append(data[i*18+4:(i+1)*18-2])
#                 #tagEPCs.append(data[i*18:(i+1)*18])
            
#             numTags = numTags - numFetch

        results = []   # stored as (ID, RSSI)
        while numTags > 0:
            self.TransmitCommand('\x03\x29\x00\x02\x00')
            (start, length, command, status, data, CRC) = self.ReceiveResponse()

            tagsRetrieved = ord(data[3])
            for i in xrange(tagsRetrieved):
                rssi = ord(data[4 + i*19])
                tagID = data[4 + i*19 + 5 : 4 + i*19 + 5 + 12]
                results.append( (tagID, rssi) )
            
            numTags = numTags - tagsRetrieved
            
        # Reset/Clear the Tag ID Buffer for next Read Tag ID Multiple
        self.TransmitCommand('\x00\x2A')
        self.ReceiveResponse()
            
        return results
    
    def TrackSingleTag(self, tagID, timeout=50, safe_response = True):
        # Make sure TagID is 60-bits
        if len(tagID) != 12:
            raise M5e_Error('Only 96-bit tags supported by TrackSingleTag')
        
        timeoutHighByte = chr((timeout & 0xFFFF) >> 8)
        timeoutLowByte = chr(timeout & 0x00FF)
        try:
            self.TransmitCommand('\x12\x21'+timeoutHighByte+timeoutLowByte+'\x11\x00\x02\x60'+tagID)
            if safe_response:
                (start, length, command, status, data, CRC) = self.ReceiveResponse()
                # data is in form OptionsFlags (1 byte) MetaFlags (2 bytes) RSSI (1 byte), EPC (12 bytes data[-14:-2]) TAG CRC (2 bytes)
            else:  # for when you want to query _really_ fast.
                return self.QuickTrackResponse()

        except M5e_CommandStatusError, inst:
            flag = False
            # Read & Get returns non-zero status when no tags found.  This is 
            #   an expected event... so ignore the exception.
            if inst[1] == '\x04\x00':   
                return -1   # Requested tag not found
            else:
                raise
            
        return ord(data[3])

    def ChangeTagID(self, newTagID, timeout=50):
        # Note: This will rewrite the EPC on the first tag detected!
        
        # Make sure TagID is 60-bits
        if len(newTagID) != 12:
            raise M5e_Error('Only 96-bit tags supported by TrackSingleTag')
        
        timeoutHighByte = chr((timeout & 0xFFFF) >> 8)
        timeoutLowByte = chr(timeout & 0x00FF)
        try:
            cmdLen = chr(len(newTagID)+4)
            self.TransmitCommand(cmdLen+'\x23'+timeoutHighByte+timeoutLowByte+'\x00\x00'+newTagID)
            (start, length, command, status, data, CRC) = self.ReceiveResponse()
        except M5e_CommandStatusError, inst:
            return False
            
        return True


class M5e_error(Exception):
    "General Exception"
    pass

class M5e_SerialPortError(M5e_error):
    "If pyserial throws error"
    pass
    
class M5e_CRCError(M5e_error):
    "If CRC check from Mercury packet is incorrect (data corruption)"
    pass
    
class M5e_CommandStatusError(M5e_error):
    "If return response from Mercury is non-zero (error status)"
    pass

class M5e_TransmitTimeoutExceeded(M5e_error):
    "Something happened (probably power failure) to disable serial transmission"
    pass

class M5e_ReceiveError(M5e_error):
    "Serial input out of synch.  Try waiting a few seconds, flush input stream, and reissue command"
    pass


## THIS IS THE "DIRECT" METHOD...
##if __name__ == '__main__':
##    import time
##    import numpy as np
##    
##    read = M5e(portSTR='/dev/ttyUSB0', baudrate=230400)   
##
##    # Demonstrate Querying the environment
##    for i in range(5):
##        print read.QueryEnvironment()
##        time.sleep(1)
##        
##    # To track tags' RSSI
##    tags = read.QueryEnvironment()
##    for tag in tags:
##        print [tag], read.TrackSingleTag(tag)


## METHOD USING THREADS (more generally useful)
if __name__ == '__main__':
    import time
    
    def P1(M5e):
        M5e.ChangeAntennaPorts(1,1)
        return 'AntPort1'

    def P2(M5e):
        M5e.ChangeAntennaPorts(2,2)
        return 'AntPort2'

    def PrintDatum(data):
        ant, ids, rssi = data
        print data

    print 'Starting with read power 2300 centi-dBm.  Change to 3000 for M5e-full'
    r = M5e(readPwr=2300)
    q = M5e_Poller(r, antfuncs=[P1, P2], callbacks=[PrintDatum])

    q.query_mode()
    
    t0 = time.time()
    while time.time() - t0 < 3.0:
        blah = 0

    q.track_mode('RedBottle   ')

    t0 = time.time()
    while time.time() - t0 < 3.0:
        blah = 0

    q.stop()

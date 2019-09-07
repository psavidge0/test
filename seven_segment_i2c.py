import smbus

class SevenSegmentI2c():
    '''
    Controls a Sparkfun 7 Segment display via i2c
    Requires the python-smbus package to be installed
    '''
    def __init__(self, i2cbusnum, address=0x71):
        self.address = address
        self.bus = smbus.SMBus(i2cbusnum)

    def write_byte(self, value):
        #Adding a retry as this seems to fail quite
        #frequently on my setup
        retry_count = 2
        while retry_count > 0:
            try:
                self.bus.write_byte(self.address, value)
                #this will break us out of the loop
                #if the previous line didn't generate
                #an exception
                retry_count = 0
            except IOError as ex:
                retry_count = retry_count - 1
                #add a delay in case the bus was busy
                time.sleep(0.1)
                print 'caught exception writing', hex(value), 'remaining:', retry_count
                #raise
                if retry_count <= 0:
                    #rethrow the exception if we're done retrying
                    raise
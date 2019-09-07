

class DisplaySegment:
    '''
    Defines the individual display segments of the LED
    '''
    MIDDLE_TOP    = 0b00000001 # overline
    RIGHT_TOP     = 0b00000010 
    RIGHT_BOTTOM  = 0b00000100
    MIDDLE_BOTTOM = 0b00001000 # _
    LEFT_BOTTOM   = 0b00010000
    LEFT_TOP      = 0b00100000
    MIDDLE_MIDDLE = 0b01000000 # -

class DotEnum:
    '''
    Defines the 'dots' on the display
    '''
    #Decimal points are numbered left to right
    DECIMAL_1  = 0b00000001 # n.nnn
    DECIMAL_2  = 0b00000010 # nn.nn
    DECIMAL_3  = 0b00000100 # nnn.n
    DECIMAL_4  = 0b00001000 # nnnn.
    COLON      = 0b00010000 # nn:nn
    APOSTROPHE = 0b00100000 # nnn'n




class SevenSegmentDisplay(object):
    '''
    Controls a Sparkfun 7 Segment display via i2c
    https://learn.sparkfun.com/tutorials/using-the-serial-7-segment-display/firmware-overview
    Note: this code is not a Sparkfun product, use at your own risk!
    '''
    def __init__(self, databus):
        '''
        Takes either a data bus to communicate over, should
        be either an SPI or I2C bus that implements write_byte
        '''
        #data bus that will be used to communicate with the display
        self.databus = databus
        #segment control registers for each digit
        #starting with the leftmost digit
        self.segment_addresses = [0x7B, 0x7C, 0x7D, 0x7E]

    
    def __write_byte(self, value):
        self.databus.write_byte(value)


    def __validate_digit(self, cmd):
        '''
        Ensure that the digit data we're being
        provided does not evaluate to one
        of the control commands
        '''
        if (cmd < 0x76) or (cmd > 0x81):
            return False
        else:
            return True

    def restore_factory_defaults(self):
        '''
        Restore factory defaults of the display
        '''
        self.__write_byte(0x81)

    def clear_display(self):
        '''
        blanks the display
        '''
        self.__write_byte(0x76)

    def set_brightness_level(self, percent):
        '''
        Sets the brightness level as a percentage
        of total brightness
        '''
        if (percent < 0) or (percent > 100):
            print 'invalid percentage for brightness, setting to medium level'
            percent = 50
        
        #brightness level can be set from 0 to 255
        val = int((percent / 100.0) * 255)
        #write the control address
        self.__write_byte(0x7A)
        #then write the brightness level
        self.__write_byte(val)


    def set_cursor_position(self, position):
        ''' 
        sets the cursor position with 0 being the leftmost write_digit
        and 3 being the rightmost
        '''
        if (position >= 0) and (position <= 3):
            self.__write_byte(0x79)
            self.__write_byte(position)
        else:
            print 'invalid position', position


    def set_nondigits(self, dots=[]):
        '''
        enables decimal points, the colon,
        and the apostrophe, takes a list 
        of DotEnums
        '''
        #default to all special "dot" LEDs off
        val = 0
        for dot in dots:
            val = val | dot

        #write the command first
        self.__write_byte(0x77)
        #then the bitmask comprising all of the
        #enabled items
        #TODO: can we read the existing mask to 
        #just turn on additional things???
        self.__write_byte(val)

    def write_digit(self, digit):
        '''
        Writes a digit to the display at
        the current cursor position. Each
        time as digit is written the cursor
        moves one to the right
        '''
        if not self.__validate_digit(digit):
            self.__write_byte(digit)

    def write_digit_to_position(self, position, digit):
        '''
        Write a digit to the display at a specified
        position
        '''
        self.set_cursor_position(position)
        self.write_digit(digit)

    def write_segments(self, position, segments=[]):
        '''
        Controls the individual LED segments, the
        segments list should be of type DisplaySegment
        '''
        if (position >= 0) and (position <= 3):
            #default to all segments off
            val = 0
            for seg in segments:
                val = val | seg
            #write to the control register for the digit whose segments
            #we're updating
            self.__write_byte(self.segment_addresses[position])
            self.__write_byte(val)
        else:
            print 'invalid position', position

    def write_int(self, val, fill_char=' '):
        '''
        write an integer to the display
        fill_char will pad numbers up to 
        four digits. pad occurs on the left
        '''
        #write an integer value across all of the digits of the display
        strval = str(val)
        #need to convert this to a string in case
        #we are passed a number as a fill character
        strfillchar = str(fill_char)
        if len(strval) > 4:
            #value has too many digits to be displayed
            print 'value is too large to fit on display'
            return
        if len(strfillchar)> 1:
            print 'must use a single character for filling'
            return

        digits_to_fill = 4 - len(strval)
        #create a string long enough to fill the empty space
        fill_str = digits_to_fill * strfillchar
        write_str = fill_str + strval

        for i in range(len(write_str)):
            self.write_digit_to_position(i, ord(write_str[i]))

def main():
    print 'press enter to execute each command'
    try:
        #raspberry pi rev. B version 1.0 uses bus 0
        display = SparkfunSevenSegment(0)
        raw_input('clear display')
        display.clear_display()
        
        raw_input('set some zeros')
        for i in range(4):
            display.write_digit(0)

        raw_input('set nondigits')
        nondigits = [DotEnum.DECIMAL_1, DotEnum.COLON]
        display.set_nondigits(nondigits)

        raw_input('clear nondigits')
        display.set_nondigits()

        
        raw_input('set min brightness')
        display.set_brightness_level(0)
        raw_input('set med brightness')
        display.set_brightness_level(50)
        raw_input('set max brightness')
        display.set_brightness_level(100)

        raw_input('writing 6 to position 2')
        display.write_digit_to_position(2, 6)

        raw_input('segment control')
        segments = [DisplaySegment.RIGHT_TOP, DisplaySegment.RIGHT_BOTTOM, 
        DisplaySegment.LEFT_TOP, DisplaySegment.LEFT_BOTTOM]
        display.write_segments(1, segments)

    except IOError as ex:
        print 'got an i/o error'
        print ex

if  __name__ =='__main__':
    main()


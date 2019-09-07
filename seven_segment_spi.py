import spidev



class SevenSegmentSpi():
    '''
    SPI interface for communicating with a Sparkfun 7 Segment display
    Requires the py-spidev module to be installed: https://github.com/doceme/py-spidev
    This is basically an adapter that allows communicating over SPI by the SparkfunSevenSegment class.
    '''
    def __init__(self, bus = 0, chip_enable = 0):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, chip_enable)

    def write_byte(self, value):
        value_list = [value]
        self.spi.xfer(value_list)

import time
import seven_segment_display
import seven_segment_i2c




def main():    
    try:
        #model b rev 1.0
        bus = seven_segment_i2c.SevenSegmentI2c(0)
        #model b rev 2.0
        #bus = seven_segment_i2c.SevenSegmentI2c(1)
        display = seven_segment_display.SevenSegmentDisplay(bus)
        display.clear_display()
        enable_colon = False
        display_military = False
        #store the previous time, so that
        #we only update the display when the time
        #changes
        prev_time = 0
        while True:
            if display_military:
                #24 hour format
                h = int(time.strftime("%H", time.localtime()))
            else:
                #12 hour format
                h = int(time.strftime("%I", time.localtime()))
            m = time.localtime().tm_min
            #time value to write to device
            val = h * 100 + m
            
            #make the colon blink every other cycle
            enable_colon = not enable_colon
            nondigits = []
            if enable_colon:
                nondigits.append(seven_segment_display.DotEnum.COLON)
            display.set_nondigits(nondigits)
            if prev_time != val:
                display.write_int(val)
            #save the current time as previous for the next iteration
            #of the loop so we can check if we actually need to update
            #the display with the new time
            prev_time = val
            time.sleep(1)
    except IOError as ex:
        print ex

        
if  __name__ =='__main__':
    main()


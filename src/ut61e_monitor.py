'''
Created on Sep 22, 2017

@author: Dmitry Melnichansky 4X1MD ex 4X5DM, 4Z7DTF
         https://github.com/4x1md
         http://www.qrz.com/db/4X1MD
         
@note: The script uses UT61E class which to reads data from UNI-T UT61E using
       serial interface and displays it in human readable form.
'''

from src.ut61e import UT61E
import sys
import time
import datetime
from serial import SerialException

SLEEP_TIME = 0.25
PORT = "COM3"

if __name__ == '__main__':
    print "Starting UT61E monitor..."
    
    try:
        if len(sys.argv) > 1:
            port = sys.argv[1]
        else:
            port = PORT
            
        dmm = UT61E(PORT)
        
        while True:
            meas = dmm.get_readable(disp_norm_val=True)
            print
            print datetime.datetime.now()
            print meas
    
            time.sleep(SLEEP_TIME)
            
    except SerialException, e:
        print "Serial port error."
        print e
        
    except KeyboardInterrupt:
        print
        print "Extiting UT61E monitor."
        sys.exit()

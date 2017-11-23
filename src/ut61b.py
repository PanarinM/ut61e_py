"""
Created on Sep 22, 2017

@author: Dmitry Melnichansky 4X1MD ex 4X5DM, 4Z7DTF
         https://github.com/4x1md
         http://www.qrz.com/db/4X1MD
         
@note: UT61E class which reads data packets from UNI-T UT61E using serial
       interface, parses them and returns as dictionary or as string in
       human readable form.
"""

from __future__ import print_function
import serial

# Settings constants
BAUD_RATE = 2400
BITS = serial.EIGHTBITS
PARITY = serial.PARITY_NONE
STOP_BITS = serial.STOPBITS_ONE
DTR = True
RTS = False
TIMEOUT = 1
# Data packet ends with CR LF (\r \n) characters
EOL = b'\x0D\x0A'
RAW_DATA_LENGTH = 14
READ_RETRIES = 3

# BYTE 0 sign
PLUS = 0x2b
NEG = 0x2d

# Bytes containing digits
DIGIT_BYTES = (1, 2, 3, 4)

# in addition, byte 1 can be "?" (0x3f) indicating value overflow
QST = 0x3f

# BYTE 5 white space
WHITE_SPACE = 0b00000100

# BYTE 6 Decimal point
POS_1 = 0x31  # 0b10001100
POS_2 = 0x32  # 0b01001100
POS_3 = 0x33  # 0b00011100
POS_4 = 0x34  # 0b00001100

# BYTE 7  Settings/ Pressend Buttons/ Binary flags, can be combined
BAR_GRPH = 0x1    # 0b10000000
HOLD = 0x2        # 0b01000000
RELATIVE = 0x4    # 0b00100000
AC = 0x8          # 0b00010000
DC = 0x10         # 0b00001000
AUTORANGE = 0x20  # 0b00000100


# TODO: ...
"""
┌────────────────────────────────────────────┐
│ Byte 8: MIN/MAX                            │
├────────────┬───┬───┬───┬───┬───┬───┬───┬───┤
│            │ 0 │ 1 │ 2 │ 3 │ 4 │ 5 │ 6 │ 7 │
├────────────┼───┼───┼───┼───┼───┼───┼───┼───┤
│ nano       │   │   │   │   │   │   │ x │   │ 0x2
│ low bat    │   │   │   │   │   │ x │   │   │ 0x4
│autopoweroff│   │   │   │   │ x │   │   │   │ 0x8
│ min        │   │   │   │ x │   │   │   │   │ 0x10
│ max        │   │   │ x │   │   │   │   │   │ 0x20
└────────────┴───┴───┴───┴───┴───┴───┴───┴───┘

┌────────────────────────────────────────────┐
│ Byte 9: Unit prefix                        │
├────────────┬───┬───┬───┬───┬───┬───┬───┬───┤
│            │ 0 │ 1 │ 2 │ 3 │ 4 │ 5 │ 6 │ 7 │
├────────────┼───┼───┼───┼───┼───┼───┼───┼───┤
│ percent    │   │   │   │   │   │   │ x │   │ 0x2
│ diode      │   │   │   │   │   │ x │   │   │ 0x4
│ beep       │   │   │   │   │ x │   │   │   │ 0x8
│ mega       │   │   │   │ x │   │   │   │   │ 0x10
│ kilo       │   │   │ x │   │   │   │   │   │ 0x20
│ milli      │   │ x │   │   │   │   │   │   │ 0x40
│ micro      │ x │   │   │   │   │   │   │   │ 0x80
└────────────┴───┴───┴───┴───┴───┴───┴───┴───┘

┌────────────────────────────────────────────┐
│ Byte 10: Unit                              │
├────────────┬───┬───┬───┬───┬───┬───┬───┬───┤
│            │ 0 │ 1 │ 2 │ 3 │ 4 │ 5 │ 6 │ 7 │
├────────────┼───┼───┼───┼───┼───┼───┼───┼───┤
│ Fahrenheit │   │   │   │   │   │   │   │ x │ 0x1
│ Degree     │   │   │   │   │   │   │ x │   │ 0x2
│ Farad      │   │   │   │   │   │ x │   │   │ 0x4
│ Hertz      │   │   │   │   │ x │   │   │   │ 0x8
│ hFE        │   │   │   │ x │   │   │   │   │ 0x10
│ Ohm        │   │   │ x │   │   │   │   │   │ 0x20
│ Ampere     │   │ x │   │   │   │   │   │   │ 0x40
│ Volt       │ x │   │   │   │   │   │   │   │ 0x80
└────────────┴───┴───┴───┴───┴───┴───┴───┴───┘

┌────────────────────────────────────────────┐
│ Byte 11: bargraph value                    │
│   bit 0   : positive (0) or negative (1)   │
│   bit 1-6 : value as 7bit unsigned int     │
└────────────────────────────────────────────┘

┌────────────────────────────────────────────┐
│ Byte 12: Marks end of frame                │
├────────────┬───┬───┬───┬───┬───┬───┬───┬───┤
│            │ 0 │ 1 │ 2 │ 3 │ 4 │ 5 │ 6 │ 7 │
├────────────┼───┼───┼───┼───┼───┼───┼───┼───┤
│ CR         │   │   │   │   │ x │ x │   │ x │ 0x0d
└────────────┴───┴───┴───┴───┴───┴───┴───┴───┘

┌────────────────────────────────────────────┐
│ Byte 13: Marks end of frame                │
├────────────┬───┬───┬───┬───┬───┬───┬───┬───┤
│            │ 0 │ 1 │ 2 │ 3 │ 4 │ 5 │ 6 │ 7 │
├────────────┼───┼───┼───┼───┼───┼───┼───┼───┤
│ LF         │   │   │   │   │ x │   │ x │   │ 0x0a
└────────────┴───┴───┴───┴───┴───┴───┴───┴───┘
"""


class UT61E(object):
    
    def __init__(self, port):
        self._port = port
        self._ser = serial.Serial(self._port, BAUD_RATE, BITS, PARITY, STOP_BITS, timeout=TIMEOUT)
        self._ser.setDTR(DTR)
        self._ser.setRTS(RTS)

    def read_raw_data(self):
        """Reads a new data packet from serial port.
        If the packet was valid returns array of integers.
        if the packet was not valid returns empty array.
        
        In order to get the last reading the input buffer is flushed
        before reading any data.
        
        If the first received packet contains less than 14 bytes, it is
        not complete and the reading is done again. Maximum number of
        retries is defined by READ_RETRIES value.
        """
        self._ser.reset_input_buffer()

        for x in range(READ_RETRIES):
            raw_data = self._ser.read_until(EOL, RAW_DATA_LENGTH)
            # If 14 bytes were read, the packet is valid and the loop ends.
            if len(raw_data) == RAW_DATA_LENGTH:
                break

        res = []
        
        # Check data validity
        if self.is_data_valid(raw_data):
            res = [ord(c) for c in bytes(raw_data).decode()]
        
        return res

    def is_data_valid(self, raw_data):
        """Checks data validity:
        1. 14 bytes long
        2. Footer bytes 0x0D 0x0A"""
        # Data length
        if len(raw_data) != RAW_DATA_LENGTH:
            return False
        
        # End bytes
        if not raw_data.endswith(EOL):
            return False
        
        return True
    
    def read_hex_str_data(self):
        """Returns raw data represented as string with hexadecimal values."""
        data = self.read_raw_data()
        codes = ["%02X" % c for c in data]
        return " ".join(codes)
    
    def get_meas(self):
        """Returns received measurement as dictionary"""
        res = MEAS_RES.copy()
        
        raw_data = self.read_raw_data()
        
        # If raw data is empty, return
        if len(raw_data) == 0:
            res['data_valid'] = False
            return res

        # Percent
        res['percent'] = True if raw_data[7] & PERCENT else False
        
        # Minus
        minus = True if raw_data[7] & NEG else False
        res['minus'] = minus
        
        # Low battery
        res['low_bat'] = True if raw_data[7] & PERCENT else False
        
        # Overload
        res['ovl'] = True if raw_data[7] & OL else False
        
        # Delta
        res['delta'] = True if raw_data[8] & DELTA else False
        
        # UL
        res['ul'] = True if raw_data[9] & UL else False
        
        # MAX
        res['max'] = True if raw_data[9] & MAX else False
        
        # MIN
        res['min'] = True if raw_data[9] & MIN else False
        
        # DC
        res['dc'] = True if raw_data[10] & DC else False
        
        # AC
        res['ac'] = True if raw_data[10] & AC else False
        
        # AUTO
        res['auto'] = True if raw_data[10] & AUTO else False
        
        # Herz
        res['hz'] = True if raw_data[10] & HZ else False
        
        # Hold
        res['hold'] = True if raw_data[11] & HOLD else False
        
        # Measurement mode, range and units
        meas_type = MEAS_TYPE[raw_data[6] & 0x0F]
        range_id = raw_data[0] & 0b00000111
        # If Herz or % is chosen in voltage or current measurement
        # mode, corresponding range tuple is chosen.
        if res['percent']:
            meas_range = RANGE_PERCENT[range_id]
        elif res['hz']:
            meas_range = RANGE_F[range_id]
        else:
            meas_range = meas_type[1][range_id]
        res['mode'] = meas_type[0]
        res['range'] = meas_range[0]
        res['units'] = meas_range[1]
        multiplier = meas_range[2]
        
        # Value
        val = 0
        for n in DIGIT_BYTES:
            digit = raw_data[n] & DIGIT_MASK
            val = val * 10 + digit
        val *= multiplier
        val = -val if minus else val
        res['val'] = val
        
        # Normalize value
        nval = self.normalize_val(res['val'], res['units'])
        res['norm_val'] = nval[0]
        res['norm_units'] = nval[1]
        
        res['data_valid'] = True
        
        return res
    
    def normalize_val(self, val, units):
        """Normalizes measured value to standard units. Voltage 
        is normalized to Volt, current to Ampere, resistance to Ohm,
        capacitance to Farad and frequency to Herz.
        Other units are not changed."""
        val = val * NORM_RULES[units][0]
        units = NORM_RULES[units][1]
        return (val, units) 

    def get_readable(self, disp_norm_val=False):
        """Prints measurement details in human readable form.
        disp_norm_val: if True, normalized values will also be displayed.
        """
        data = self.get_meas()
        
        if not data.get('data_valid', False):
            return "UT61E is not connected."
        
        res = ""
        
        # AC/DC, HOLD, REL and low battery
        ac_dc = ''
        if data['dc']:
            ac_dc = 'DC'
        elif data['ac']:
            ac_dc = 'AC'
        peak = ''
        if data['min']:
            peak = 'MIN'
        elif data['max']:
            peak = 'MAX'
        hold = 'HOLD' if data['hold'] else ''
        rel = 'REL' if data['delta'] else ''
        low_bat = 'LOW BAT' if data['low_bat'] else ''
        res += "%s\t%s\t%s\t%s\t%s\n" % (ac_dc, peak, hold, rel, low_bat)
        
        # Mode and range
        if data['auto']:
            res += "MODE: %s\tAUTO\n" % (data['mode'])
        else:
            res += "MODE: %s\t%s %s\n" % (data['mode'], data['range'], data['units'])
        
        # Displayed value        
        if data['ovl']:
            res += "OL\n"
        elif data['ul']:
            res += "UL\n"
        else:
            res += "%s %s\n" % (data['val'], data['units'])

        # Display normalized values
        # If the DMM displayes OL or UL these values will be displayed
        if disp_norm_val:
            if data['ovl']:
                res += "OL"
            elif data['ul']:
                res += "UL"
            else:
                res += "= %s %s" % (data['norm_val'], data['norm_units'])
        
        return res
    
    def __del__(self):
        if hasattr(self, '_ser'):
            self._ser.close()

if __name__ == '__main__':
    pass

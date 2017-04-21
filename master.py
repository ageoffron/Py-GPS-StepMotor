#!/usr/bin/python2
import smbus
import math
import time
import sys
from time import sleep
#https://github.com/Knio/pynmea2
import pynmea2
import RPi.GPIO as GPIO
import serial
import re
import datetime

# Bearings: http://www.mathsteacher.com.au/year7/ch08_angles/07_bear/bearing.htm
# Coordinates basic 101: http://www.satsig.net/lat_long.htm


# from https://gist.github.com/jeromer/2005586
def getbearing(pointA,pointB):
    if (type(pointA) != tuple) or (type(pointB) != tuple):
        raise TypeError("Only tuples are supported as arguments")

    lat1 = math.radians(pointA[0])
    lat2 = math.radians(pointB[0])
    diffLong = math.radians(pointB[1] - pointA[1])

    x = math.sin(diffLong) * math.cos(lat2)
    y = (math.cos(lat1) * math.sin(lat2)) - ((math.sin(lat1) * math.cos(lat2) * math.cos(diffLong)))

    initial_bearing = math.atan2(x, y)

    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360

    return compass_bearing

class NEO6MGPS:

    def __init__(self, serialport, baudratespeed):

        self.gpsdevice = serial.Serial(port=serialport, baudrate=baudratespeed, timeout=5)
	self.gpsdevice.flush()
        self.init()

    def flush(self):
        self.gpsdevice.flush()

    def init(self):

        if self.isOpen():
            return True

        return False

    def open(self):
        self.gpsdevice.open()

    def isOpen(self):
        return self.gpsdevice.isOpen()

    def readBuffer(self):

        try:
            data = self.gpsdevice.read(1)
            n = self.gpsdevice.inWaiting()
            if n:
                data = data + self.gpsdevice.read(n)
            return data

        except Exception, e:
            print "Big time read error, what happened: ", e
            sys.exit(1)

	
class Motor(object):
    def __init__(self, pins, mode=2):
        """Initialise the motor object.

        pins -- a list of 4 integers referring to the GPIO pins that the IN1, IN2
                IN3 and IN4 pins of the ULN2003 board are wired to
        mode -- the stepping mode to use:
                1: wave drive (not yet implemented)
                2: full step drive
                3: half step drive (default)

        """
        self.P1 = pins[0]
        self.P2 = pins[1]
        self.P3 = pins[2]
        self.P4 = pins[3]
        self.mode = mode
        self.deg_per_step = 5.625 / 64  # for half-step drive (mode 3)
        self.steps_per_rev = int(360 / self.deg_per_step)  # 4096
        self.step_angle = 0  # Assume the way it is pointing is zero degrees
        for p in pins:
            GPIO.setup(p, GPIO.OUT)
            GPIO.output(p, 0)

    def _set_rpm(self, rpm):
        """Set the turn speed in RPM."""
        self._rpm = rpm
        # T is the amount of time to stop between signals
        self._T = (60.0 / rpm) / self.steps_per_rev

    # This means you can set "rpm" as if it is an attribute and
    # behind the scenes it sets the _T attribute
    rpm = property(lambda self: self._rpm, _set_rpm)

    def move_to(self, angle):
        """Take the shortest route to a particular angle (degrees)."""
        # Make sure there is a 1:1 mapping between angle and stepper angle
        target_step_angle = 8 * (int(angle / self.deg_per_step) / 8)
        steps = target_step_angle - self.step_angle
        steps = (steps % self.steps_per_rev)
        if steps > self.steps_per_rev / 2:
            steps -= self.steps_per_rev
            #print "moving " + `steps` + " steps"
            if self.mode == 2:
                self._move_acw_2(-steps / 8)
            else:
                self._move_acw_3(-steps / 8)
        else:
           # print "moving " + `steps` + " steps"
            if self.mode == 2:
                self._move_cw_2(steps / 8)
            else:
                self._move_cw_3(steps / 8)
        self.step_angle = target_step_angle

    def __clear(self):
        GPIO.output(self.P1, 0)
        GPIO.output(self.P2, 0)
        GPIO.output(self.P3, 0)
        GPIO.output(self.P4, 0)

    def _move_acw_2(self, big_steps):
        self.__clear()
        for i in range(big_steps):
            GPIO.output(self.P3, 0)
            GPIO.output(self.P1, 1)
            sleep(self._T * 2)
            GPIO.output(self.P2, 0)
            GPIO.output(self.P4, 1)
            sleep(self._T * 2)
            GPIO.output(self.P1, 0)
            GPIO.output(self.P3, 1)
            sleep(self._T * 2)
            GPIO.output(self.P4, 0)
            GPIO.output(self.P2, 1)
            sleep(self._T * 2)

    def _move_cw_2(self, big_steps):
        self.__clear()
        for i in range(big_steps):
            GPIO.output(self.P4, 0)
            GPIO.output(self.P2, 1)
            sleep(self._T * 2)
            GPIO.output(self.P1, 0)
            GPIO.output(self.P3, 1)
            sleep(self._T * 2)
            GPIO.output(self.P2, 0)
            GPIO.output(self.P4, 1)
            sleep(self._T * 2)
            GPIO.output(self.P3, 0)
            GPIO.output(self.P1, 1)
            sleep(self._T * 2)

    def _move_acw_3(self, big_steps):
        self.__clear()
        for i in range(big_steps):
            GPIO.output(self.P1, 0)
            sleep(self._T)
            GPIO.output(self.P3, 1)
            sleep(self._T)
            GPIO.output(self.P4, 0)
            sleep(self._T)
            GPIO.output(self.P2, 1)
            sleep(self._T)
            GPIO.output(self.P3, 0)
            sleep(self._T)
            GPIO.output(self.P1, 1)
            sleep(self._T)
            GPIO.output(self.P2, 0)
            sleep(self._T)
            GPIO.output(self.P4, 1)
            sleep(self._T)

    def _move_cw_3(self, big_steps):
        self.__clear()
        for i in range(big_steps):
            GPIO.output(self.P3, 0)
            sleep(self._T)
            GPIO.output(self.P1, 1)
            sleep(self._T)
            GPIO.output(self.P4, 0)
            sleep(self._T)
            GPIO.output(self.P2, 1)
            sleep(self._T)
            GPIO.output(self.P1, 0)
            sleep(self._T)
            GPIO.output(self.P3, 1)
            sleep(self._T)
            GPIO.output(self.P2, 0)
            sleep(self._T)
            GPIO.output(self.P4, 1)
            sleep(self._T)


class hmc5883l:

    __scales = {
        0.88: [0, 0.73],
        1.30: [1, 0.92],
        1.90: [2, 1.22],
        2.50: [3, 1.52],
        4.00: [4, 2.27],
        4.70: [5, 2.56],
        5.60: [6, 3.03],
        8.10: [7, 4.35],
    }

    def __init__(self, port=1, address=0x1E, gauss=1.3, declination=(0,0)):
        self.bus = smbus.SMBus(port)
        self.address = address

        (degrees, minutes) = declination
        self.__declDegrees = degrees
        self.__declMinutes = minutes
        self.__declination = (degrees + minutes / 60) * math.pi / 180

        (reg, self.__scale) = self.__scales[gauss]
        self.bus.write_byte_data(self.address, 0x00, 0x70) # 8 Average, 15 Hz, normal measurement
        self.bus.write_byte_data(self.address, 0x01, reg << 5) # Scale
        self.bus.write_byte_data(self.address, 0x02, 0x00) # Continuous measurement

    def declination(self):
        return (self.__declDegrees, self.__declMinutes)

    def twos_complement(self, val, len):
        # Convert twos compliment to integer
        if (val & (1 << len - 1)):
            val = val - (1<<len)
        return val

    def __convert(self, data, offset):
        val = self.twos_complement(data[offset] << 8 | data[offset+1], 16)
        if val == -4096: return None
        return round(val * self.__scale, 4)

    def axes(self):
        data = self.bus.read_i2c_block_data(self.address, 0x00)
        #print map(hex, data)
        x = self.__convert(data, 3)
        y = self.__convert(data, 7)
        z = self.__convert(data, 5)
        return (x,y,z)

    def heading(self):
        (x, y, z) = self.axes()
        headingRad = math.atan2(y, x)
        headingRad += self.__declination

        # Correct for reversed heading
        if headingRad < 0:
            headingRad += 2 * math.pi

        # Check for wrap and compensate
        elif headingRad > 2 * math.pi:
            headingRad -= 2 * math.pi

        # Convert to degrees from radians
        headingDeg = headingRad * 180 / math.pi
        return headingDeg

    def degrees_minutes(self, headingDeg):
        degrees = math.floor(headingDeg)
        minutes = round((headingDeg - degrees) * 60)
        return (degrees, minutes)

    def degrees(self, headingDeg):
        degrees = math.floor(headingDeg)
        return (degrees)


    def __str__(self):
        (x, y, z) = self.axes()
        return "Axis X: " + str(x) + "\n" \
               "Axis Y: " + str(y) + "\n" \
               "Axis Z: " + str(z) + "\n" \
               "Declination: " + self.degrees(self.declination()) + "\n" \
               "Heading: " + self.degrees(self.heading()) + "\n"


if __name__ == "__main__":

    GPIO.setmode(GPIO.BCM)
    m = Motor([17,18,23,22])
    m.rpm = 10
    print "Pause in seconds: " + `m._T` 
    compass = hmc5883l(gauss = 2.5, declination = (13,0))
    heading = compass.degrees(compass.heading())
    print("\rHeading:" + str(heading))
    m.move_to(heading)
    time.sleep(0.2)

    device = NEO6MGPS("/dev/ttyAMA0", 9600)
    newdata = ""
    line = ""

    # Get a reading of GPS data
    while device.isOpen(): 
	if newdata:
		line = newdata
        	newdata = ""
    	line = line + device.readBuffer()

	if re.search("\r\n", line):
        	data, newdata = line.split("\r\n")

	        #print "----" + str(datetime.datetime.now()) + "----"
		#        print data
		try:
                        pNMEA = pynmea2.parse(data)
                        if isinstance(pNMEA, pynmea2.types.talker.GGA):
                                print "\r\nLAT:" + pNMEA.lat + pNMEA.lat_dir
                                print "LONG:" + pNMEA.lon + pNMEA.lon_dir
                                print "SAT:" + str(pNMEA.num_sats)
                                print "GPS Sig:" + str(pNMEA.gps_qual)
				long_degWhole = float(int(float(pNMEA.lon)/100))
				long_degDec = (float(pNMEA.lon) - long_degWhole*100)/60
				long_deg = long_degWhole + long_degDec
				if pNMEA.lon_dir == 'W':
					long_deg = (-1) * long_deg
				lat_degWhole = float(int(float(pNMEA.lat)/100))
				lat_degDec = (float(pNMEA.lat) - lat_degWhole*100)/60
				lat_deg = lat_degWhole + lat_degDec
				if pNMEA.lat_dir == 'S':
                                        lat_deg = (-1) * lat_deg

				p = (lat_deg,long_deg)
				print p
				NorthPole = (47.53643693,-110.91472199)
			#	NorthPole = (-113.4,82.3)
				bearing = int(getbearing(p,NorthPole))
				print "Moving to: " + str(bearing)
				#m.move_to(bearing)
				device.flush()
                except:
                        pass
                line = ""

GPIO.cleanup()


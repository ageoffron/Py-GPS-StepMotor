#!/usr/bin/python2

import sys
import time
from flask import Flask
from flask import request, Response,redirect, jsonify, url_for, render_template, abort
import json
from flask_api import FlaskAPI
import RPi.GPIO as GPIO
import smbus,math,serial,re,datetime
import pynmea2

print(sys.version)

app = Flask(__name__)
app.debug = True

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

    def close(self):
	self.gpsdevice.close()
	
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
        if (val & (1 << len - 1)):
            val = val - (1<<len)
        return val

    def __convert(self, data, offset):
        val = self.twos_complement(data[offset] << 8 | data[offset+1], 16)
        if val == -4096: return None
        return round(val * self.__scale, 4)

    def axes(self):
        data = self.bus.read_i2c_block_data(self.address, 0x00)
        x = self.__convert(data, 3)
        y = self.__convert(data, 7)
        z = self.__convert(data, 5)
        return (x,y,z)

    def heading(self):
        (x, y, z) = self.axes()
        headingRad = math.atan2(y, x)
        headingRad += self.__declination

        if headingRad < 0:
            headingRad += 2 * math.pi

        elif headingRad > 2 * math.pi:
            headingRad -= 2 * math.pi

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



@app.route("/")
def indexpage():
        return render_template("index.html")

@app.route("/gpsdata")
def gpsdata():
	return render_template("gpsdata.html")

@app.route("/compass")
def compass():
	return render_template("compass.html")

@app.route("/api/v1.0/compass/get")
def getheading():
	try:
		compass = hmc5883l(gauss = 2.5, declination = (13,0))
    		heading = compass.degrees(compass.heading())
    		response = jsonify(error="",heading = heading,status="SUCCESS")
		response.status_code = 200
		return response
	except Exception,e:
		print e
		error = str(e)
		error = error.replace("'","")
		response = jsonify(error=error,heading="",status="FAILED")
		response.status_code = 404
		return response


@app.route("/api/v1.0/gps/get")
def gps():
	try:
		device = NEO6MGPS("/dev/ttyAMA0", 9600)
    	except Exception,e:
		print e
		response = jsonify({})
                response.status_code = 404
                return response
	
	newdata = ""
    	line = ""
	gga = False
	while not gga:
        	if newdata:
                	line = newdata
                	newdata = ""
        	line = line + device.readBuffer()

        	if re.search("\r\n", line):
                	data, newdata = line.split("\r\n")

                	try:
                        	pNMEA = pynmea2.parse(data)
                        	if isinstance(pNMEA, pynmea2.types.talker.GGA):
					gga = True
                                	print "\r\nLAT:" + pNMEA.lat + pNMEA.lat_dir
                                	print "LONG:" + pNMEA.lon + pNMEA.lon_dir
                                	print "SAT:" + str(pNMEA.num_sats)
                                	print "GPS Sig:" + str(pNMEA.gps_qual)
					if (pNMEA.lon) and (pNMEA.lat):	
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
					else:
						lat_deg=""
						long_deg=""

					response = jsonify(timestamp=str(pNMEA.timestamp),latitude=pNMEA.lat + pNMEA.lat_dir,longitude=pNMEA.lon + pNMEA.lon_dir,num_sats = pNMEA.num_sats,signal=pNMEA.gps_qual,altitude=pNMEA.altitude,altitude_units=pNMEA.altitude_units,geosep=pNMEA.geo_sep,age_gps_data=pNMEA.age_gps_data,ref_station_id=pNMEA.ref_station_id,latitude_deg=str(lat_deg),longitude_deg=str(long_deg))
                                        response.status_code = 200
					device.close()
                                        return response
	

                                #	p = (lat_deg,long_deg)
                               # 	print p
                               # 	NorthPole = (47.53643693,-110.91472199)
                               # 	bearing = int(getbearing(p,NorthPole))
                               # 	print "Moving to: " + str(bearing)
                                	#m.move_to(bearing)
                                	device.flush()
                	except Exception,e:
				print e
                        	pass

	device.close()

@app.route("/api/v1.0/move")
def MoveRotor():
	
       	steps = 4096
       	direction = request.args.get('direction') 
        delayms = 2
        arg_steps = int(steps)
        arg_direction = int(direction)
        arg_delayms = int(delayms)

        GPIO.setmode(GPIO.BCM)

        StepPins = [17,18,23,22]

        for pin in StepPins:
                GPIO.setup(pin,GPIO.OUT)
                GPIO.output(pin, False)

        Seq = [[1,0,0,0],
               [0,1,0,0],
               [0,0,1,0],
               [0,0,0,1]]

        StepCount = len(Seq)
        StepDir = arg_direction
                # Set to 1 or 2 for clockwise
                # Set to -1 or -2 for anti-clockwise

        WaitTime = arg_delayms/float(1000)

        StepCounter = 0
        TotalSteps = 0

        while (TotalSteps != arg_steps):

        	for pin in range(0, 4):
                        TotalSteps += 1
                        xpin = StepPins[pin]
                        if Seq[StepCounter][pin]!=0:
                                GPIO.output(xpin, True)
                        else:
                                GPIO.output(xpin, False)

                StepCounter += StepDir
        
                if (StepCounter>=StepCount):
                        StepCounter = 0
                if (StepCounter<0):
                        StepCounter = StepCount+StepDir

                time.sleep(WaitTime)

        return ''




if __name__ == "__main__":
    app.run(port=80,debug=True,host="0.0.0.0")



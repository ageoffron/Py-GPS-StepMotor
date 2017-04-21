#!/usr/bin/python2

import serial 
import sys
import re
import datetime
import pynmea2

class NEO6MGPS:
    
    def __init__(self, serialport, baudratespeed):

        self.gpsdevice = serial.Serial(port=serialport, baudrate=baudratespeed, timeout=5)
        
        self.init()

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



device = NEO6MGPS("/dev/ttyAMA0", 9600)

newdata = ""
line = ""
gpsdata = ""

while device.isOpen():
	if newdata: 
		line = newdata
        	newdata = ""
         
	line = line + device.readBuffer()
        print line   
	if re.search("\r\n", line):
		data, newdata = line.split("\r\n")

        	#print "----" + str(datetime.datetime.now()) + "----"
        	#print data
		try:
    			pNMEA = pynmea2.parse(data)
    			if isinstance(pNMEA, pynmea2.types.talker.GGA):
				print "\r\nLAT:" + pNMEA.lat + pNMEA.lat_dir
				print "LONG:" + pNMEA.lon + pNMEA.lon_dir
				print "SAT:" + str(pNMEA.num_sats)
				print "GPS Sig:" + str(pNMEA.gps_qual)
			else:
				print ""				

		except Exception,e:
			print e
    			pass
        	line = ""

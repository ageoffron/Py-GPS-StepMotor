#!/usr/bin/python
from i2clibraries import i2c_hmc5883l
import time
hmc58831 = i2c_hmc5883l.i2c_hmc5883l(1)

hmc58831.setContinuousMode()
hmc58831.setDeclination(13)

while True:
	(heading_degrees, heading_minutes) = hmc58831.getHeading()
	print(str(heading_degrees)+"\u00b0 "+str(heading_minutes)+"'")
	WaitTime = 1
	time.sleep(WaitTime)        

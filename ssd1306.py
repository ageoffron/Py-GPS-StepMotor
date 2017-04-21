#!/usr/bin/python2

import time,math
from PIL import Image
from PIL import ImageOps
from PIL import ImageFont
from PIL import ImageDraw
from oled.device import ssd1306
from oled.render import canvas


import Adafruit_SSD1306

device = ssd1306(port=1, address=0x3C)

def Display(title,message,x,y):
        disp = Adafruit_SSD1306.SSD1306_128_64(24)
        disp.begin()
        disp.clear()
        disp.display()
        padding = 2
        shape_width = 20
        bottom = device.height - padding - 1
	draw.rectangle((0, 0, device.width-1, device.height-1), outline=255, fill=0)

#	with canvas(device) as draw:
	font = ImageFont.truetype('fonts/C&C Red Alert [INET].ttf', 14)
	pad = font.getsize(title)    		
	draw.text((x+padding, y+padding),title,  font=font, fill=255)
	draw.text((x+padding+pad[0]+2, y+padding),message, font=font, fill=255)


Display("Long:","test1",0,0)
Display("Lat:","test2",0,10)

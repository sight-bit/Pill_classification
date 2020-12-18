import RPi.GPIO as gp
import os
import cv2 as cv 
import time



adapter_info = {   "A":{   "i2c_cmd":"i2cset -y 1 0x70 0x00 0x04",
                            "gpio_sta":[0,0,1],
                    },
                "B":{
                        "i2c_cmd":"i2cset -y 1 0x70 0x00 0x05",
                        "gpio_sta":[1,0,1],
                    },
                "C":{
                        "i2c_cmd":"i2cset -y 1 0x70 0x00 0x06",
                        "gpio_sta":[0,1,0],
                    },
                "D":{
                        "i2c_cmd":"i2cset -y 1 0x70 0x00 0x07",
                        "gpio_sta":[1,1,0],
                    },
             }

def save_pic(Camera):
	camera = cv.VideoCapture(0)
	width = 320
	height = 240 

	gp.setwarnings(False)
	gp.setmode(gp.BOARD)
	gp.setup(7, gp.OUT)
	gp.setup(11,gp.OUT)
	gp.setup(12,gp.OUT)

	channel_info = adapter_info.get(Camera)
	if channel_info == None:
	    print("Can't get this info")
	os.system(channel_info["i2c_cmd"]) # i2c write
	gpio_sta = channel_info["gpio_sta"] # gpio write
	gp.output(7, gpio_sta[0])
	gp.output(11, gpio_sta[1])
	gp.output(12, gpio_sta[2])


	camera.set(3,width)
	camera.set(4,height)
	ret, frame = camera.read()

	if ret == True:
	    print("camera %s Saved" %(Camera))
	    pname = "image_"+Camera+".jpg"
	    cv.imwrite(pname,frame)
	    time.sleep(1)
	else:
	    print("camera %s init FAILED" %(Camera))
	    time.sleep(1)

	camera.release()
	gp.cleanup()




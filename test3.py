import RPi.GPIO as gp
import os
import cv2 as cv
import numpy as np
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
    start_time = time.time()
    
    channel_info = adapter_info.get(Camera)

    if channel_info == None:
        print("Can't get this info")
    os.system(channel_info["i2c_cmd"]) # i2c write
    gpio_sta = channel_info["gpio_sta"] # gpio write
    gp.output(7, gpio_sta[0])
    gp.output(11, gpio_sta[1])
    gp.output(12, gpio_sta[2])
    end_time = time.time()
    #print("%.2f msec" %((end_time-start_time)*1000))

gp.setwarnings(False)
gp.setmode(gp.BOARD)
gp.setup(7, gp.OUT)
gp.setup(11,gp.OUT)
gp.setup(12,gp.OUT)
i=0

### 위카메라
setup_time = time.time()
cam = cv.VideoCapture(0)
width = 800
height = 800 

cam.set(3,width)
cam.set(4,height)

save_pic('A') # 9.52 옴
ret, frame1 = cam.read()

if ret == True:
    time.sleep(1)
    print("frame1 success")
else:
    print("camera UP init FAILED")
cam.release()




# 아래
cam = cv.VideoCapture(0)
cam.set(3,width)
cam.set(4,height)

save_pic('C')
ret, frame2 = cam.read()

if ret == True:
    time.sleep(1)
    print("frame2 success")
else:
    print("camera DOWN init FAILED")

dst = np.hstack((frame1,frame2))
# dst = np.array(dst)

while os.path.isfile('./image/'+str(i)+'_Pair.jpg'):
    i+=1
cv.imwrite('./image/'+str(i)+'_Pair.jpg', dst)

cv.imshow('frame1',frame1)
cv.imshow('frame2',frame2)
cv.imshow('dst',dst)
cv.waitKey()
cv.destroyAllWindows()

cam.release()
gp.cleanup()



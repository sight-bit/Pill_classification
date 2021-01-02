import numpy as np
import cv2 as cv
from readcsv import r
# innit net
tensorflowNet = cv.dnn.readNet('frozen_graph.pb')

# background image for pre-processing   
bak = cv.imread('bak.jpg') 

def infer(frame_up, frame_down):
    net_input = np.hstack((frame_up,frame_down))    
    tensorflowNet.setInput(cv.dnn.blobFromImage(net_input, 1/255, size=(224,224))) # 1 3 224 224 # NCHW  #swapRB=True, crop=False
    networkOutput = tensorflowNet.forward()
    idx = np.argmax(networkOutput)+1 # pill index
    pn = r[idx][1] # pill name
    pg = r[idx][5] # pill guide 0 1 2

    return pn, pg

import cv2 
import numpy as np
import serial
from time import sleep
import time
#arduinoData = serial.Serial('com4',115200, timeout=.1)
#arduinoData.write()


#creating a black image
img = np.zeros((512,512,3),np.uint8)

#draw a straight line 
cv2.line(img,(256,0),(256,511),(0,0,255),5)
x1=0 
y1=0
x2=511
y2=511

#draw the other line 
cv2.line(img,(x1,y1),(x2,y2),(255,0,0),5)



cv2.imshow('imag',img)
cv2.waitKey(0) 
cv2.destroyAllWindows()


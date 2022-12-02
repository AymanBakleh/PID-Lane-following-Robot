import cv2
import numpy as np
import math
import sys
import time
import RPi.GPIO as GPIO
import serial
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)  
from time import sleep

from picamera import PiCamera
from picamera.array import PiRGBArray

output=1
mystring=""
deviation=1
#testing git


import threading
'''
class SerialPort:
	message='' 
	def __init__(self,port,buand):
		super(SerialPort, self).__init__()
		self.port=serial.Serial(port,buand)
		self.port.close()
		if not self.port.isOpen():
			self.port.open()
	def port_open(self):
		if not self.port.isOpen():
			self.port.open()
	def port_close(self):
		self.port.close()
	def send_data(self):
			data = input ( "Please enter the data (non-Chinese) to send and receive data simultaneously:")
			n=self.port.write((data+'\n').encode())
			return n
	def read_data(self):
		while True:
			self.message=self.port.readline()
			print(self.message)

'''




from PID import PID



# PID Controller Parameters
DIRECT = 1
REVERSE = 0

AUTOMATIC = 1
MANUAL = 0
# PID controller's output limits
OUTMIN = -250.0
OUTMAX = 250.0
# Default tuning constances
Kp = 25
Ki = 2.5
Kd = 0.25
pid = PID(Kp,Ki,Kd,0,DIRECT)
pid.SetOutputLimits(OUTMIN, OUTMAX)
pid.SetSampleTime(1000)
pid.SetMode(AUTOMATIC)
pid.SetTunings(Kp, Ki, Kd) # Change tuning parameters


     
     
   
     
     
     
GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#GPIO.wait_for_edge(21, GPIO.FALLING)





serialPort = "/dev/ttyACM0" # Serial
baudRate = 115200 # baud
 



time.sleep(1)

        

def write_read():
    global mystring
    time.sleep(1)
    arduino.write(b'h')
    time.sleep(1)
    while(1) :
        arduino.write(mystring.encode()+b'\r\n')
        print("I sent",mystring)
        time.sleep(0.05)

def PIDF() :
    global output
    global deviation
    

def detect_edges(frame):
    # filter for blue lane lines
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    cv2.imshow("HSV",hsv)
    lower_blue = np.array([90, 120, 0], dtype = "uint8")
    upper_blue = np.array([150, 255, 255], dtype="uint8")
    mask = cv2.inRange(hsv,lower_blue,upper_blue)
    cv2.imshow("mask",mask)
    
    # detect edges
    edges = cv2.Canny(mask, 50, 100)
    cv2.imshow("edges",edges)
    
    return edges

def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)

    # only focus lower half of the screen
    polygon = np.array([[
        (0,     height),
        (0,  (height)/2),
        (width,(height)/2),
        (width , height),
    ]], np.int32)
    
    cv2.fillPoly(mask, polygon, 255)
    
    cropped_edges = cv2.bitwise_and(edges, mask)
    #cv2.imshow("roi",cropped_edges)
    
    return cropped_edges

def detect_line_segments(cropped_edges):
    rho = 1  
    theta = np.pi / 180  
    min_threshold = 10  
    
    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold, 
                                    np.array([]), minLineLength=20, maxLineGap=100)

    return line_segments


def average_slope_intercept(frame, line_segments):
    lane_lines = []
    
    if line_segments is None:
        print("no line segments detected")
        return lane_lines

    height, width,_ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1/3
    left_region_boundary = width * (1 - boundary)
    right_region_boundary = width * boundary
    
    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                print("skipping vertical lines (slope = infinity")
                continue
            
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)
            
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    return lane_lines

def make_points(frame, line):
    height, width, _ = frame.shape
    
    slope, intercept = line
    
    y1 = height  # bottom of the frame
    y2 = int(y1 / 2)  # make points from middle of the frame down
    
    if slope == 0:
        slope = 0.1
        
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    
    return [[x1, y1, x2, y2]]

def display_lines(frame, lines, line_color=(255, 255, 0), line_width=2):
    line_image = np.zeros_like(frame)
    
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
                
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    
    return line_image


def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5 ):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape
    
    steering_angle_radian = steering_angle / 180.0 * math.pi
    
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)
    
    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)
    
    return heading_image

def get_steering_angle(frame, lane_lines):
    
    height,width,_ = frame.shape
    
    if len(lane_lines) == 2:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)
        
    elif len(lane_lines) == 1:
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
        y_offset = int(height / 2)
        
    elif len(lane_lines) == 0:
        x_offset = 0
        y_offset = int(height / 2)
        
    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  
    steering_angle = angle_to_mid_deg + 90
    
    return steering_angle
'''
video = cv2.VideoCapture(0)
video.set(cv2.CAP_PROP_FRAME_WIDTH,320)
video.set(cv2.CAP_PROP_FRAME_HEIGHT,240)
    
'''

##fourcc = cv2.VideoWriter_fourcc(*'XVID')
##out = cv2.VideoWriter('Original15.avi',fourcc,10,(320,240))
##out2 = cv2.VideoWriter('Direction15.avi',fourcc,10,(320,240))

speed = 400
lastTime = 0
lastError = 0

#kp = 0.6
#kd = kp * 0.65
'''
while True:
    
    ret,frame = video.read()
    frame = cv2.flip(frame,-1)'''
camera = PiCamera()
camera.framerate= 32
camera.rotation = 180
camera.resolution=(400,288)
rawCapture = PiRGBArray(camera)










def cv():
    global mystring
    global output
    global deviation
    for frame in camera.capture_continuous(rawCapture,format="bgr", use_video_port = True):
    
        image = frame.array
        cv2.waitKey(1)
        rawCapture.truncate(0)
        frame = image
    
        cv2.imshow("original",frame)
        edges = detect_edges(frame)
        cv2.imshow("edges",edges)
        roi = region_of_interest(edges)
        line_segments = detect_line_segments(roi)
        lane_lines = average_slope_intercept(frame,line_segments)
        lane_lines_image = display_lines(frame,lane_lines)
        steering_angle = get_steering_angle(frame, lane_lines)
        heading_image = display_heading_line(lane_lines_image,steering_angle)
        cv2.imshow("heading line",heading_image)

  #  now = time.time()
   # dt = now - lastTime
        deviation = steering_angle - 90
        
        pid.myInput = deviation
        pid.mySetpoint = 0
    
        if pid.Compute() == True:
            output= pid.myOutput
            print("output is:",output)
            
 
            
        
        print("deviation is: ",deviation)
          
        
   # error = abs(deviation)
        key = cv2.waitKey(1)& 0xFF
        if key == 32:
            arduino.write(b"x")
            break
        
        
        
        
        
        mystring=str(output)
        
    
      
    
        
        
    
        
    
    
    
    
    
    '''No PID code!
    if deviation < 10 and deviation > -10: # do not steer if there is a 10-degree error range
        deviation = 0
        error = 0
        arduino.write(b"w")
    elif deviation > 10: # steer right if the deviation is positive
      arduino.write(b"k")

    elif deviation < -10: # steer left if deviation is negative
       arduino.write(b"j")
        
    arduino.reset_output_buffer()'''

    
    
    
   
    
    
    

if __name__ == '__main__':

    arduino = serial.Serial('/dev/ttyACM0', 115200,

timeout = 1 )
    
    
    t3=threading.Thread(target=PIDF)
    t2=threading.Thread(target=cv)
    t1=threading.Thread(target=write_read)
    
    t3.start()
    
    t1.start()
    t2.start()
     # wait until thread 1 is completely executed
    t1.join()
    # wait until thread 2 is completely executed
    t2.join()
    
#arduino=SerialPort(serialPort,baudRate)


arduino.close()
cv2.destroyAllWindows()
#video.release()

#habd zone be careful
'''
'''           
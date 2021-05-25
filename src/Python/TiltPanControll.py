#!/usr/bin/env python
#
#  Pan Tilt Servo Control 
#  Execute with parameter ==> sudo python3 servoCtrl.py <pan_angle> <tilt_angle>
#
#  MJRoBot.org 01Feb18
  
from time import sleep
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
import keyboard
panx=90
tiltx=90
pan = 27
tilt = 22

GPIO.setup(tilt, GPIO.OUT) # white => TILT
GPIO.setup(pan, GPIO.OUT) # gray ==> PAN

def setServoAngle(servo, angle):
	assert angle >=30 and angle <= 150
	pwm = GPIO.PWM(servo, 50)
	pwm.start(8)
	dutyCycle = angle / 18. + 3.
	pwm.ChangeDutyCycle(dutyCycle)
	sleep(0.3)
	pwm.stop()
	

if __name__ == '__main__':  

        
    setServoAngle(pan, 90)
    setServoAngle(tilt, 120)
    print('I got Here!')
    while True:  # making a loop
        a=input("where to go?") 
        if a=="q" :  # if key 'q' is pressed 
            print('You Pressed q Key!')
            break  # finishing the loop
        elif a=="a":  # if key 'q' is pressed 
            print("You Pressed A Key!")
            panx=panx+10
        elif a=="d":  # if key 'q' is pressed 
            print("You Pressed d Key!")
            panx=panx-10
        elif a=="s":  # if key 'q' is pressed 
            print("You Pressed s Key!")
            tiltx=tiltx+10
        elif a=="w":  # if key 'q' is pressed 
            print('You Pressed w Key!')
            tiltx=tiltx-10
        if pan>=180 :
            print('that is enough!')
        if tilt<=0 :
            print('that is enough!')
        if pan>=180 :
            print('that is enough!')
        if tilt<=0 :
            print('that is enough!')
        setServoAngle(pan,panx)
        setServoAngle(tilt,tiltx)
        print('pan is')
        print(panx)
        print('titl is')
        print(tiltx)
         
    GPIO.cleanup()
    
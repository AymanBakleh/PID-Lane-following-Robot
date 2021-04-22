import cv 
import numpy as np
import serial
from time import sleep
import time
arduinoData = serial.Serial('com4',115200, timeout=.1)
arduinoData.write()

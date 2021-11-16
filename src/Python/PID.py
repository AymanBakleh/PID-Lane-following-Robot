"""
***************************************************************
* This is a Python port (runs on Raspberry PI) of:
* 
* Arduino PID Library - Version 1.2.1
* by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
*
* This Library is licensed under the MIT License
***************************************************************

 - For an ultra-detailed explanation of why the code is the way it is, please visit: 
   http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

"""
   
import time
class PID:
    """
    Discrete PID control
    """
    DIRECT = 0
    REVERSE = 1
    AUTOMATIC = 1
    MANUAL = 0

    def __init__(self, Setpoint, Kp, Ki, Kd, ControllerDirection):

        self.inAuto = False
        self.SampleTime = 100
        self.SampleTimeInSec = self.SampleTime/1000.0
        
        self.myInput = 0.0
        self.myOutput = 0.0
        self.mySetpoint = Setpoint

        self.ControllerDirection = ControllerDirection

        self.LastTime = int(round(time.time() * 1000)) - self.SampleTime
        
        self.SetControllerDirection(ControllerDirection)
        self.SetTunings(Kp, Ki, Kd)


    """ The PID will either be connected to a DIRECT acting process (+Output leads
    to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
    know which one, because otherwise we may increase the output when we should
    be decreasing.  This is called from the constructor.
    """

    def SetControllerDirection(self,Direction):
        if Direction != self.ControllerDirection:
            self.myKp = ( 0.0 - self.myKp )
            self.myKi = ( 0.0 - self.myKi )
            self.myKd = ( 0.0 - self.myKd )
        self.ControllerDirection = Direction

    """  The function SetTunings allows the controller's dynamic performance to be adjusted.
    It is called automatically from the constructor, but tunings can also
    be adjusted on the fly during normal operation.
    """

    def SetTunings(self, Kp, Ki, Kd):
        if (Kp < 0 or Ki < 0 or Kd <0):
            return
        self.dispKp = Kp
        self.dispKi = Ki
        self.dispKd = Kd

        self.SampleTimeInSec = self.SampleTime/1000.0
        self.myKp = Kp
        self.myKi = Ki * self.SampleTimeInSec
        self.myKd = Kd / self.SampleTimeInSec

        if self.ControllerDirection == PID.REVERSE:
            self.myKp = ( 0.0 - self.myKp )
            self.myKi = ( 0.0 - self.myKi )
            self.myKd = ( 0.0 - self.myKd )

    """  The function SetSampleTime sets the period, in Milliseconds,
    at which the PID calculation is performed.
    """
    
    def SetSampleTime(self, NewSampleTime):
        if NewSampleTime > 0:
            self.ratio = NewSampleTime / self.SampleTime
            self.myKi *= self.ratio
            self.myKd /= self.ratio
            self.SampleTime = NewSampleTime

    """ The function SetOutputLimits is used to scale the PID controller output
    """

    def SetOutputLimits(self, Min, Max):
        if Min >= Max:
            return
        self.outMin = Min
        self.outMax = Max
        if self.inAuto == True:
            if myOutput > self.outMax:
                self.myOutput = self.outMax
            elif self.myOutput < self.outMin:
                self.MyOutput = self.outMin
            if self.outputSum > self.outMax:
                self.outputSum = self.outMax
            elif self.outputSum < self.outMin:
                self.outputSum = self.outMin

    """  The function SetMode Allows the controller Mode to be set to manual (0)
    or Automatic (non-zero) when the transition from manual to auto occurs,
    the controller is automatically initialized.
    """

    def SetMode(self, Mode):
        if (self.inAuto == False and Mode == PID.AUTOMATIC):
            # We just went from manual to auto
            self.Initialize()
        self.inAuto = (Mode == 1)

    """  Initialize() does all the things that need to happen to ensure a
    bumpless transfer from manual to automatic mode.
    """

    def Initialize(self):
        self.outputSum = self.myOutput
        self.lastInput = self.myInput
        if self.outputSum > self.outMax:
            self.outputSum = self.outMax
        elif self.outputSum < self.outMin:
            self.outputSum = self.outMin

    """  The function Compute() should be called every main cycle.
    The function will decide for itself whether a new
    pid Output needs to be computed. Returns true when the output is computed,
    false when nothing has been done.
    """

    def Compute(self):
        if self.inAuto == False:
            return False
        self.now = int(round(time.time() * 1000))
        self.timeChange = self.now - self.LastTime
        if self.timeChange >= self.SampleTime:
            # Compute all the working error variables
            self.error = self.mySetpoint - self.myInput
            self.dInput = self.myInput - self.lastInput
            self.outputSum += self.myKi * self.error

            if self.outputSum > self.outMax:
                self.outputSum = self.outMax
            elif self.outputSum < self.outMin:
                self.outputSum = self.outMin

            # Add Proportional on Error
            self.myOutput = self.myKp * self.error

            # Compute Rest of PID Output
            self.myOutput += self.outputSum - self.myKd * self.dInput

            if self.myOutput > self.outMax:
                self.myOutput = self.outMax
            elif self.myOutput < self.outMin:
                self.myOutput = self.outMin

            # Remember some variables for next time
            self.lastInput = self.myInput
            self.LastTime = self.now
            return True
        
        else:
            return False

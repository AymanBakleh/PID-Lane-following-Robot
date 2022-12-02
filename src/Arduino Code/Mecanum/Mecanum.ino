//Hi! just testing the PID branch!
#include <AccelStepper.h>
#include <Wire.h>

AccelStepper LeftBackWheel(1, 2, 5);   // x(Type:driver, STEP, DIR) - Stepper1
AccelStepper LeftFrontWheel(1, 3, 6);  //y Stepper2
AccelStepper RightBackWheel(1, 4, 7);  // zStepper3
AccelStepper RightFrontWheel(1, 12, 13); //a Stepper4

int wheelSpeed = 400;
int f = 1;
int g = 0;
int h = 0;
char order;
String InputAyman = "";
int mspeed = 10;
int turnspeed = 300;
int16_t Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];
float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180 / 3.141592654;
float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;
////////////////////////PID CONSTANST/////////////////////
float kp = 25;
float ki = 0;
float kd = 0.8;
float desired_angle = 0;
int speedPID = 0;

void setup() {


  pinMode(11, OUTPUT);
  digitalWrite(11, 1);
  Wire.begin(); /////////////TO BEGIN I2C COMMUNICATIONS///////////////
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  // put your setup code here, to run once:
  LeftFrontWheel.setMaxSpeed(wheelSpeed);
  //LeftFrontWheel.setAcceleration(1000);
  LeftBackWheel.setMaxSpeed(wheelSpeed);
  // LeftBackWheel.setAcceleration(1000);
  RightFrontWheel.setMaxSpeed(wheelSpeed);
  // RightFrontWheel.setAcceleration(1000);
  RightBackWheel.setMaxSpeed(wheelSpeed);
  //RightBackWheel.setAcceleration(1000);

  Serial.begin(115200);
  time = millis();
}

void loop() {
  while (f == 1) {
    if (Serial.available() > 0) {
      order = Serial.read();
      //    InputAyman = Serial.readStringUntil(10);


   
      if (order == 'w')
      {    digitalWrite(11, 0);
        moveForward();
        Serial.println("Forward") ;
      }
      else if (order == 'x')
      {   digitalWrite(11, 0);
        moveBackward() ;
        Serial.println("Backward") ;
      }
      else if (order == 'd')
      {   digitalWrite(11, 0);
        moveSidewaysRight();
        Serial.println("side ways Right") ;
      }
      else if (order == 'a')
      {    digitalWrite(11, 0);
        moveSidewaysLeft();
        Serial.println("Side ways Left") ;
      }
      else if (order == 'j')
      {    digitalWrite(11, 0);
        rotateLeft();
        Serial.println("Rotate Left") ;
      }
      else if (order == 'k')
      {    digitalWrite(11, 0);
        rotateRight();
        Serial.println("Rotate Right") ;
      }
      else if (order == 'e')
      {   digitalWrite(11, 0);
        moveRightForward() ;
        Serial.println("Forward Right") ;
      }
      else if (order == 'c')
      { moveRightBackward();
        Serial.println("Backward Right") ;
      }
      else if (order == 'q')
      {    digitalWrite(11, 0);
        moveLeftForward();
        Serial.println("Forward Left") ;
      }
      else if (order == 'z')
      {    digitalWrite(11, 0);
        moveLeftBackward();
        Serial.println("Backward Left") ;
      }
      else if (order == 's')
      {   
        stopMoving();
        Serial.println("Stop!") ;
      }

      else if (order == 'f')
      { f = 1;
        g = 0;
        h = 0;
      }

      else if (order == 'g')
      { g = 1;
        f = 0;
        h = 0;
      }

      else if (order == 'h')
      { h = 1;
        f = 0;
        g = 0;
      }
    }
    LeftFrontWheel.runSpeed();
    LeftBackWheel.runSpeed();
    RightFrontWheel.runSpeed();
    RightBackWheel.runSpeed();
  }
  while (g == 1) {
    if (Serial.available() > 0) {
      InputAyman = Serial.readStringUntil(10);

      if (InputAyman != "") {

        order = InputAyman.toInt() ;
        digitalWrite(11, 0);
      }
      if (order == 'f')
      { f = 1;
        g = 0;
        h = 0;
      }

      else if (order == 'g')
      { g = 1;
        f = 0;
        h = 0;
      }

      else if (order == 'h')
      { h = 1;
        f = 0;
        g = 0;
      }
    }

    timePrev = time;
    time = millis();
    elapsedTime = (time - timePrev) / 1000;
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);

    Acc_rawX = Wire.read() << 8 | Wire.read();
    Acc_rawY = Wire.read() << 8 | Wire.read();
    Acc_rawZ = Wire.read() << 8 | Wire.read();

    Acceleration_angle[0] = (Acc_rawY/ 16384.0) * rad_to_deg;

    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 4, true);
    Gyr_rawY= Wire.read() << 8 | Wire.read();
    Gyro_angle[0] = Gyr_rawY / 131.0;
    Total_angle[0] = 0.98 * (Total_angle[0] + Gyro_angle[0] * elapsedTime) + 0.02 * Acceleration_angle[0];
    error = Total_angle[0] - desired_angle; /////////////////ERROR CALCULATION////////////////////
    pid_p = kp * error;
    pid_i = pid_i + (ki * error);

    pid_d = kd * ((error - previous_error) / elapsedTime);

    PID = pid_p + pid_d;
    previous_error = error;
    mspeed = abs(PID);
    if (Total_angle[0] < -3)
    {digitalWrite(11,0);
     rotateRightG();
    }
    if (Total_angle[0] > 3)
    {digitalWrite(11,0);
   
      
        rotateLeftG();
    }
    if (Total_angle[0] < -3 && Total_angle[0] > 3)
    {digitalWrite(11,1);
      stopMoving();
    }
    LeftFrontWheel.runSpeed();
    LeftBackWheel.runSpeed();
    RightFrontWheel.runSpeed();
    RightBackWheel.runSpeed();
  }
  while (h == 1) {

    if (Serial.available() > 0) {
      InputAyman = Serial.readStringUntil(10);

      if (InputAyman != "") {

        speedPID = InputAyman.toInt() ;
        digitalWrite(11, 0);
      }
      control();
    }
    LeftFrontWheel.runSpeed();
    LeftBackWheel.runSpeed();
    RightFrontWheel.runSpeed();
    RightBackWheel.runSpeed();
  }
}

void control() {
  LeftFrontWheel.setSpeed(wheelSpeed + speedPID);
  LeftBackWheel.setSpeed(wheelSpeed + speedPID);
  RightFrontWheel.setSpeed(wheelSpeed - speedPID);
  RightBackWheel.setSpeed(wheelSpeed - speedPID);


}

void moveForward() {
  LeftFrontWheel.setSpeed(wheelSpeed);
  LeftBackWheel.setSpeed(wheelSpeed);
  RightFrontWheel.setSpeed(wheelSpeed);
  RightBackWheel.setSpeed(wheelSpeed);
}
void moveBackward() {
  LeftFrontWheel.setSpeed(-wheelSpeed);
  LeftBackWheel.setSpeed(-wheelSpeed);
  RightFrontWheel.setSpeed(-wheelSpeed);
  RightBackWheel.setSpeed(-wheelSpeed);
}
void moveSidewaysRight() {
  LeftFrontWheel.setSpeed(wheelSpeed);
  LeftBackWheel.setSpeed(-wheelSpeed);
  RightFrontWheel.setSpeed(-wheelSpeed);
  RightBackWheel.setSpeed(wheelSpeed);

}
void moveSidewaysLeft() {
  LeftFrontWheel.setSpeed(-wheelSpeed);
  LeftBackWheel.setSpeed(wheelSpeed);
  RightFrontWheel.setSpeed(wheelSpeed);
  RightBackWheel.setSpeed(-wheelSpeed);
}
void rotateLeft() {
  LeftFrontWheel.setSpeed(-wheelSpeed);
  LeftBackWheel.setSpeed(-wheelSpeed);
  RightFrontWheel.setSpeed(wheelSpeed);
  RightBackWheel.setSpeed(wheelSpeed);

}

void rotateLeftG() {
  LeftFrontWheel.setSpeed(-mspeed );
  LeftBackWheel.setSpeed(-mspeed );
  RightFrontWheel.setSpeed( mspeed );
  RightBackWheel.setSpeed( mspeed );

}
void rotateRight() {
  LeftFrontWheel.setSpeed(wheelSpeed);
  LeftBackWheel.setSpeed(wheelSpeed);
  RightFrontWheel.setSpeed(-wheelSpeed);
  RightBackWheel.setSpeed(-wheelSpeed);

}

void rotateRightG() {
  LeftFrontWheel.setSpeed( mspeed );
  LeftBackWheel.setSpeed(mspeed);
  RightFrontWheel.setSpeed(-mspeed);
  RightBackWheel.setSpeed(-mspeed);

}
void moveRightForward() {
  LeftFrontWheel.setSpeed(wheelSpeed);
  LeftBackWheel.setSpeed(0);
  RightFrontWheel.setSpeed(0);
  RightBackWheel.setSpeed(wheelSpeed);

}
void moveRightBackward() {
  LeftFrontWheel.setSpeed(0);
  LeftBackWheel.setSpeed(-wheelSpeed);
  RightFrontWheel.setSpeed(-wheelSpeed);
  RightBackWheel.setSpeed(0);

}
void moveLeftForward() {
  LeftFrontWheel.setSpeed(0);
  LeftBackWheel.setSpeed(wheelSpeed);
  RightFrontWheel.setSpeed(wheelSpeed);
  RightBackWheel.setSpeed(0);

}
void moveLeftBackward() {
  LeftFrontWheel.setSpeed(-wheelSpeed);
  LeftBackWheel.setSpeed(0);
  RightFrontWheel.setSpeed(0);
  RightBackWheel.setSpeed(-wheelSpeed);

}
void stopMoving() {
  digitalWrite(11, 1);
  LeftFrontWheel.setSpeed(0);
  LeftBackWheel.setSpeed(0);
  RightFrontWheel.setSpeed(0);
  RightBackWheel.setSpeed(0);

}
